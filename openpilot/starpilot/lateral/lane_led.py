"""Lane-led centring (lane version 8): the controller drives from the lane lines, the model vetoes.

Blocks, as in PLAN.md section 4:
  A  BoundaryGate    accept lines/edges with adopt/drop hysteresis; never jumps the reference on a switch
  B  CentreTracker   tracked lane-centre cubic in the vehicle frame, propagated on ego motion, innovation-limited
  B2 guards          every adopted line or road edge gives a line the car must not cross (car half width + clearance)
  C  DelayState      ring buffer of published curvature; predicts the error state forward by the lateral delay
  C2 LeverLearner    learns the camera's effective lever (distance ahead of the axle, less rear slip) from steady bends
  D  LaneController  feedforward road curvature + delay-predicted feedback + slow bias, jerk/accel limited
  E  Arbiter         slewed trust weight between the lane request and the model's request; never steps

Frame: x forward, y RIGHT, positive curvature turns toward +y. Verified on the car from route 00000024's lane changes
(openpilot's left line, laneLines[1], sits at y = -1.7 and moves toward 0 during a left-blinker lane change; the
model's curvature is positive where the lines bend toward +y). The simulator uses the same frame numerically.
Pure numpy; no openpilot imports, so the same code runs in the unit tests, the simulator, replay and on the device.
"""
from collections import deque
from dataclasses import dataclass, field
import math
import numpy as np

try:
  from openpilot.starpilot.lateral.lane_led_settings import LaneLedSettings, SettingsSlew, valid as settings_valid
except ImportError:  # PC tools run this module flat, next to its settings module
  from lane_led_settings import LaneLedSettings, SettingsSlew, valid as settings_valid

VERSION = 8
ALGORITHM = "lane_led"

STATIONS = np.linspace(0.0, 60.0, 25)                     # m; a convenient x grid for synthetic boundaries in tests
_DT_PREDICT = 0.01
_MAX_DT = 0.5
_FRAME_GRACE = 0.1                                         # s without a model frame before anything fades
_LONG_GAP = 3.0                                            # s beyond the hold after which the tracker is cleared (lateral active)
_INACTIVE_GAP = 1.0                                        # s without a usable observation while lateral is inactive before clearing
_LOST_OFFSET = 3.0                                         # m: a tracked centre further away than this is not a lane estimate
_LOST_HEADING = math.radians(30.0)                         # a tracked lane skewed more than this is not a lane estimate
_SNAP_INNOVATION = 1.0                                     # m: an innovation this large is a lost track, so snap instead of limiting
_MEAS_TAU = 0.1                                            # s, filter on the measured (steering-angle) curvature
_FRESH_CONF = 0.05
_CURV_INNOVATION = 0.0                                     # extra share of each frame's innovation taken by the curvature coefficients (0: none, 1: all)
_TRACK_INNOVATION_DIV = 5.0                                # settled tracker gain = w / (w + this * conf): 5 takes a sixth of each frame (tau 0.3 s), 2 a third (0.15 s)
_RESIDUAL_FULL, _RESIDUAL_ZERO = 0.08, 0.20                # m rms fit residual on the near half of the horizon: full trust below, none above
# (heading innovation is now the heading_innovation setting; 0.5 lagged the lane heading and hurt closed-loop centring)
_CONFIDENCE_FULL = 0.5                                     # line probability at which trust from confidence is full
ROAD_JERK = 2.0                                            # m/s^3 rate bound on the feedforward (host ISO limit is 5)
_EDGE_CONFIDENCE = 0.7                                     # an edge has no probability; a line+edge pair is a trusted pair
_LINE_BRIDGE_S = 2.0                                       # s a two-line track carries on from one line plus the last width when the other line drops (change 32; 0 off; 1.5 on the first road day, 2.0 from 4 Nov)
_RECOVERY_FLOOR = 2.8                                      # s: the Recovery slider is floored here (change 33): 1.5 rang at 2 Hz on the road (3 Nov), 2.0-2.5 ring in replay, 2.8 is road-proven
_CURV_AGREE_FRAMES = 0                                     # probe: frames of same-sign curvature innovation before _CURV_INNOVATION applies (0: never)
_HEADING_SCHEDULE = (99.0, 99.0, 1.0)                      # probe: (v_full, v_min, share_at_v_min): heading innovation share falls linearly from 1 at v_full to share at v_min
# Change 37, Chestnut obstacle detector: an object inside the ego lane hands the car fully to the model until it is passed;
# while the detector is healthy the model no longer needs the divergence escape to steer round things, so lanes lead harder.
_OBSTACLE_PASS_M = 4.0                                     # m past the obstacle's far end (camera to rear bumper about 3.5 m)
_OBSTACLE_MIN_HOLD_S = 0.5                                 # s held after the last in-lane detection whatever the distance
_OBSTACLE_RETURN_S = 2.0                                   # s for the hand-back to the lines after an obstacle (fade_in is 0.75)
_DETECTOR_DIVERGENCE_M = 1.2                               # m: divergence escape threshold while the detector is healthy (gross line errors only)


# ----------------------------------------------------------------------------- inputs and outputs

@dataclass(frozen=True)
class Boundary:
  """One lane line or road edge from the model, in the vehicle frame."""
  side: int                 # 0 left, 1 right
  kind: str                 # "line" or "edge"
  x: np.ndarray
  y: np.ndarray
  std: float
  prob: float | None = None  # lines only


@dataclass(frozen=True)
class LaneLedObservation:
  t: float                            # s, monotonic
  speed: float                        # m/s
  measured_curvature: float           # 1/m from the steering angle
  model_curvature: float              # modelV2.action.desiredCurvature
  delay: float                        # s, lateralDelay
  boundaries: tuple = ()              # Boundary objects for this model frame
  model_sample_id: int = 0            # changes when a new model frame arrived
  published_curvature: float = math.nan   # curvature the host actually sent last tick (NaN: unknown)
  yaw_rate: float = math.nan          # rad/s measured; NaN -> measured_curvature * speed
  model_path_x: np.ndarray | None = None
  model_path_y: np.ndarray | None = None
  model_path_ystd: np.ndarray | None = None
  lateral_active: bool = True
  lane_change: bool = False
  driver_override: bool = False
  inputs_stale: bool = False
  limited_by_safety: bool = False
  obstacle_ok: bool = False           # the Chestnut obstacle detector is running and fresh (change 37)
  obstacle_blocked: bool = False      # an object sits inside the ego lane within its look-ahead
  obstacle_distance: float = math.inf # m ahead of the camera to its nearest in-lane point
  obstacle_length: float = 0.0        # m, assumed length of that object (by class)


@dataclass
class LaneLedResult:
  candidate_curvature: float
  kappa_lane: float
  kappa_model: float
  weight: float
  state: str                 # lane_led | blended | model_led | inactive
  reason: str
  e_y: float
  e_psi: float
  kappa_road: float
  bias: float
  trust: float
  width: float
  mode: str                  # two | one_left | one_right | none
  limited: bool
  diagnostics: dict = field(default_factory=dict)

  @property
  def correction(self) -> float:
    return self.candidate_curvature - self.kappa_model


def boundaries_from_model(model) -> tuple:
  """Adapter for a modelV2-shaped object (capnp or SimpleNamespace): inner lines 1 and 2, edges 0 and 1."""
  out = []
  for kind, index, side in (("line", 1, 0), ("line", 2, 1), ("edge", 0, 0), ("edge", 1, 1)):
    try:
      src = (model.laneLines if kind == "line" else model.roadEdges)[index]
      x, y = np.asarray(src.x, dtype=float), np.asarray(src.y, dtype=float)
      std = float((model.laneLineStds if kind == "line" else model.roadEdgeStds)[index])
      prob = float(model.laneLineProbs[index]) if kind == "line" else None
    except (AttributeError, IndexError, TypeError, ValueError):
      continue
    if x.ndim != 1 or x.shape != y.shape or len(x) < 2 or not np.isfinite(x).all() or not np.isfinite(y).all():
      continue
    out.append(Boundary(side, kind, x, y, std, prob))
  return tuple(out)


# ----------------------------------------------------------------------------- A. boundary gate

@dataclass
class _Slot:
  adopted: bool = False
  qualified_s: float = 0.0
  unqualified_s: float = 0.0
  boundary: Boundary | None = None


class BoundaryGate:
  """Adopt/drop hysteresis per (side, kind). A boundary must qualify for switch_hold before adoption
  and fail for switch_hold before it is dropped, so frame-to-frame flapping never reaches the tracker."""

  KEYS = ((0, "line"), (1, "line"), (0, "edge"), (1, "edge"))

  def __init__(self):
    self.slots = {k: _Slot() for k in self.KEYS}

  def reset(self):
    self.__init__()

  def _qualifies(self, b: Boundary | None, s: LaneLedSettings, adopted: bool) -> bool:
    if b is None or not math.isfinite(b.std) or b.std < 0.0:
      return False
    if b.kind == "line":
      if b.prob is None or not math.isfinite(b.prob):
        return False
      threshold = s.drop_confidence if adopted else s.adopt_confidence
      return b.prob >= threshold and b.std < s.max_line_std
    return b.std < s.edge_std

  def update(self, boundaries, dt: float, s: LaneLedSettings):
    seen = {(b.side, b.kind): b for b in boundaries}
    for key, slot in self.slots.items():
      b = seen.get(key)
      slot.boundary = b
      if self._qualifies(b, s, slot.adopted):
        slot.qualified_s += dt
        slot.unqualified_s = 0.0
        if not slot.adopted and slot.qualified_s >= s.switch_hold:
          slot.adopted = True
      else:
        slot.unqualified_s += dt
        slot.qualified_s = 0.0
        if slot.adopted and slot.unqualified_s >= s.switch_hold:
          slot.adopted = False

  def selected(self) -> tuple[Boundary | None, Boundary | None]:
    """Per side: the adopted painted line, or None. Road edges never define the centre (user decision, 1 November);
    they only push, through the clearance guards (change 25), see selected_edges()."""
    out = []
    for side in (0, 1):
      line = self.slots[(side, "line")]
      out.append(line.boundary if line.adopted and line.boundary is not None else None)
    return out[0], out[1]

  def selected_edges(self) -> tuple[Boundary | None, Boundary | None]:
    out = []
    for side in (0, 1):
      edge = self.slots[(side, "edge")]
      out.append(edge.boundary if edge.adopted and edge.boundary is not None else None)
    return out[0], out[1]


def _sample(b: Boundary, xs: np.ndarray):
  """Boundary y on the stations it covers; weight 1 inside coverage, 0 outside."""
  if not np.all(np.diff(b.x) > 0):
    order = np.argsort(b.x)
    bx, by = b.x[order], b.y[order]
  else:
    bx, by = b.x, b.y
  y = np.interp(xs, bx, by)
  cover = (xs >= bx[0]) & (xs <= bx[-1])
  return y, cover


# ----------------------------------------------------------------------------- B. centre tracker

class CentreTracker:
  """Lane centre as a quartic in u = x / H in the vehicle frame, H being 2.5 s of travel clipped to 15..60 m.
  A fixed 60 m cubic cannot follow a 100 m radius bend; a 2.5 s quartic can. With one line the tracked curve is
  that line's guard boundary (line plus car half width plus clearance), which carries the lane's heading and
  curvature; there is no remembered width (change 25)."""

  DEGREE = 4
  U = np.linspace(0.0, 1.0, 25)
  PHI = np.vander(U, DEGREE + 1, increasing=True)
  PINV = np.linalg.pinv(PHI)
  U_PRED = np.linspace(-0.1, 1.25, 33)
  PHI_PRED = np.vander(U_PRED, DEGREE + 1, increasing=True)
  HORIZON_S, H_MIN, H_MAX = 2.5, 15.0, 60.0

  def __init__(self):
    self.reset()

  def reset(self):
    self.c = np.zeros(self.DEGREE + 1)
    self.H = 30.0
    self.conf = 0.0             # 0 fresh .. 1 settled
    self.width = 3.5            # observed two-line width, filtered (2 s): never a centre source, only a CAP on guard pushes
    self.width_age_s = math.inf # observed frames since two lines were last seen
    self.bridge_s = 0.0         # s spent bridging a dropped second line with the last width (change 32)
    self.residual = 0.0         # filtered (0.5 s) rms fit residual: fit quality, not frame noise
    self.innovation_limited = False
    self.mode = "none"          # last usable observation's mode; "none" until one arrives
    self.boundary_confidence = 0.0
    self._pending = np.zeros(self.DEGREE + 1)  # innovation still to be applied, spread over the frame period
    self._pending_s = 0.0
    self._agree = 0; self._agree_sign = 0   # sign-agreement run of the curvature innovation (probe _CURV_AGREE_FRAMES)
    self.speed = 0.0                       # set by set_horizon; the heading schedule probe reads it

  @property
  def stations(self) -> np.ndarray:
    return self.U * self.H

  # -- horizon follows speed; coefficients are re-expressed so the curve itself does not change
  def set_horizon(self, speed: float):
    self.speed = speed
    H = float(np.clip(self.HORIZON_S * speed, self.H_MIN, self.H_MAX))
    if H != self.H:
      scale = (H / self.H) ** np.arange(self.DEGREE + 1)
      self.c = self.c * scale
      self._pending = self._pending * scale
      self.H = H

  # -- prediction on ego motion
  def predict(self, ds: float, dpsi: float, lever: float = 0.0):
    """Carry the tracked curve through the car's motion: the axle moved ds along the heading and the car yawed dpsi.
    The tracked frame is the camera's, `lever` ahead of the axle; in a turn that point also slides sideways by
    lever * dpsi, and ignoring that drags the tracked centre toward the inside of every bend by about
    v * lever * kappa * (the tracker's 0.3 s filter time): 3 cm at R 300 and 18 m/s."""
    if ds == 0.0 and dpsi == 0.0:
      return
    x = self.U_PRED * self.H
    y = self.PHI_PRED @ self.c
    dx = ds * math.cos(dpsi / 2.0) + lever * (math.cos(dpsi) - 1.0)
    dy = ds * math.sin(dpsi / 2.0) + lever * math.sin(dpsi)
    cs, sn = math.cos(dpsi), math.sin(dpsi)
    xx = cs * (x - dx) + sn * (y - dy)
    yy = -sn * (x - dx) + cs * (y - dy)
    if not np.all(np.diff(xx) > 0):
      return
    # Refit on the transported nodes themselves. Interpolating onto fixed stations first would bend the
    # curve inward a little every tick and drift the curvature estimate.
    self.c = np.linalg.lstsq(np.vander(xx / self.H, self.DEGREE + 1, increasing=True), yy, rcond=None)[0]

  def apply_pending(self, dt: float):
    if self._pending_s <= 0.0:
      return
    f = min(1.0, dt / self._pending_s)
    self.c = self.c + self._pending * f
    self._pending = self._pending * (1.0 - f)
    self._pending_s -= dt

  def decay(self, dt: float):
    """No usable observation this tick: confidence fades, nothing resets."""
    self.conf = max(0.0, self.conf - 0.3 * dt)
    self.innovation_limited = False

  # -- observation
  def observe(self, left: Boundary | None, right: Boundary | None, dt_frame: float, s: LaneLedSettings, one_line_d=(math.nan, math.nan)) -> bool:
    """Build the observed centre from the selected lines and update the tracked curve.
    Returns False when nothing usable was observed; the state is then left for the caller's gap handling."""
    xs = self.stations
    yl = yr = None
    if left is not None:
      yl, cl = _sample(left, xs)
    if right is not None:
      yr, cr = _sample(right, xs)
    mode, centre, weight, width_obs = "none", None, None, None
    width_seen = None
    if yl is not None and yr is not None:
      width_obs = float(yr[0] - yl[0])
      width_seen = width_obs           # recorded (as a cap on guard pushes) even when too narrow to be a lane
      if s.min_width <= width_obs <= s.max_width:
        mode = "two"
        centre = 0.5 * (yl + yr)
        weight = (cl & cr).astype(float) * self._std_weight(left, s) * self._std_weight(right, s)
      else:
        # Implausible pair: keep the more confident line as a single boundary.
        pl = left.prob if left.prob is not None else -1.0
        pr = right.prob if right.prob is not None else -1.0
        if pl >= pr:
          right = None
        else:
          left = None
        width_obs = None
    if (mode == "none" and (left is not None) != (right is not None) and self.mode == "two" and _LINE_BRIDGE_S > 0.0
        and self.bridge_s < _LINE_BRIDGE_S and self.width_age_s <= _LINE_BRIDGE_S + 2.0 and s.min_width <= self.width <= s.max_width):
      # Bridge (change 32): a two-line track whose second line has just dropped keeps the two-line centre from the
      # remaining line and the width seen a moment ago, for _LINE_BRIDGE_S at most. Falling straight into one-line
      # mode moves the tracked curve from the centre to that line's guard, half a metre or more, and the fit
      # residual that jump leaves takes the trust to zero: on the fifth drive two thirds of the hand-over cycles were
      # a line gone for under two seconds (65 of 66 such episodes came back). A lane does not change width in a
      # second, so the last width is a better centre than the guard for that long; after it the one-line rules apply.
      one = left if left is not None else right
      y1, c1 = _sample(one, xs)
      sign = 1.0 if one.side == 0 else -1.0
      centre = y1 + sign * 0.5 * self.width
      weight = c1.astype(float) * self._std_weight(one, s)
      mode = "two"
      self.bridge_s += dt_frame
    elif mode == "two":
      self.bridge_s = 0.0
    if mode == "none" and (left is not None) != (right is not None):
      # One line: track its guard boundary, the nearest position the car may hold (car half width plus clearance
      # toward +y for the left line at -y, toward -y for the right line). The controller holds its lateral position
      # and follows this curve's heading and curvature; it moves only when a guard is crossed (change 25).
      one = left if left is not None else right
      y1, c1 = _sample(one, xs)
      sign = 1.0 if one.side == 0 else -1.0
      d = one_line_d[one.side] if math.isfinite(one_line_d[one.side]) else 0.5 * s.car_width + s.line_clearance
      centre = y1 + sign * d
      weight = c1.astype(float) * self._std_weight(one, s)
      mode = "one_left" if one.side == 0 else "one_right"
    self.innovation_limited = False
    if centre is None or weight is None or not np.any(weight > 0):
      return False
    self.mode = mode
    # Update: per-station blend toward the observation, then refit.
    y_pred = self.PHI @ self.c
    # A lost track (blown up while blind, or an observation a metre or more away from it) is re-acquired, never nudged.
    if self.lost() or (float(np.max(np.abs((centre - y_pred)[weight > 0]))) > _SNAP_INNOVATION if np.any(weight > 0) else False):
      self.c = np.zeros(self.DEGREE + 1); self.conf = 0.0; self._pending[:] = 0.0; self._pending_s = 0.0
      y_pred = self.PHI @ self.c
    fresh = self.conf < _FRESH_CONF
    # A settled tracker takes about a sixth of each frame's innovation (tau about 0.3 s). This filters the
    # reference, not the loop: the car's own motion is carried exactly by predict(), so only real changes in
    # the lane geometry see the lag. A fresh tracker snaps to the first observation.
    gain = weight / (weight + _TRACK_INNOVATION_DIV * self.conf) if not fresh else (weight > 0).astype(float)
    y_new = y_pred + gain * (centre - y_pred)
    c_new = self.PINV @ y_new
    dc = c_new - self.c
    if not fresh and _CURV_INNOVATION > 0.0:
      # The curvature coefficients may take more of the frame's innovation than the offset and heading: the bend
      # ahead is what the feedforward and the delay predictor read, and a lagged bend makes the feedback fight the
      # turn-in for the tracker's time constant, while the offset and heading at the car are what the model's lines
      # wobble in. 0 keeps the single gain; 1 snaps the curvature to each frame. Taken unconditionally it passed the
      # model's frame noise (3 Nov replay); with _CURV_AGREE_FRAMES it applies only while the full innovation of the
      # quadratic term has kept one sign for that many frames, which noise (alternating) does not and a bend does.
      dc_full = self.PINV @ (y_pred + np.where(weight > 0, centre - y_pred, 0.0)) - self.c
      sgn = 1 if dc_full[2] > 0 else -1 if dc_full[2] < 0 else 0
      self._agree = self._agree + 1 if sgn != 0 and sgn == self._agree_sign else (1 if sgn != 0 else 0)
      self._agree_sign = sgn
      if _CURV_AGREE_FRAMES <= 0 or self._agree >= _CURV_AGREE_FRAMES:
        dc[2:] += (dc_full[2:] - dc[2:]) * _CURV_INNOVATION
    if not fresh:
      # Real model lines wobble in heading frame to frame far more than in offset; the heading term is the
      # damping and the dominant source of request jerk, so its innovation is taken more slowly (reference side).
      v_full, v_min, share_min = _HEADING_SCHEDULE
      sched = 1.0 if self.speed <= v_full else share_min if self.speed >= v_min else 1.0 + (share_min - 1.0) * (self.speed - v_full) / max(v_min - v_full, 1e-6)
      dc[1] *= s.heading_innovation * sched * (1.0 if mode == "two" else _ONE_LINE_INNOVATION)
      d0 = abs(dc[0])
      d1 = abs(math.atan(c_new[1] / self.H) - math.atan(self.c[1] / self.H))
      scale = min(1.0, s.innovation_limit_m / d0 if d0 > 0 else 1.0,
                  math.radians(s.innovation_limit_deg) / d1 if d1 > 0 else 1.0)
      if scale < 1.0:
        dc = dc * scale
        self.innovation_limited = True
    if fresh:
      self.c = self.c + dc
      self._pending[:] = 0.0; self._pending_s = 0.0
    else:
      # Spread the innovation over the coming frame period (plain interpolation between 20 Hz frames) so a
      # frame's worth of change never lands in one 10 ms tick and trips the jerk limiter.
      self._pending = self._pending + dc
      self._pending_s = dt_frame
    fit = self.PHI @ (self.c + self._pending)
    # Fit quality where it matters: the near half of the horizon feeds e_y, e_psi and the feedforward preview.
    near_w = weight * (self.U <= 0.5)
    w_sum = float(near_w.sum())
    residual = float(math.sqrt(float((near_w * (centre - fit) ** 2).sum()) / w_sum)) if w_sum > 0 else 0.0
    # Filter over 0.5 s: a per-frame residual is mostly line noise plus tracker lag, not fit quality.
    alpha_r = 1.0 if self.conf < _FRESH_CONF else 1.0 - math.exp(-dt_frame / 0.5)
    self.residual += (residual - self.residual) * alpha_r
    self.conf = min(1.0, self.conf + (1.0 - self.conf) * 0.3)
    probs = [b.prob if b.prob is not None else _EDGE_CONFIDENCE for b in (left, right) if b is not None]
    # Confidence of the lines defining the centre: their mean, not the weaker one (change 24). A kerb-side line at
    # 0.25 beside a centre line at 0.95 still gave a usable midpoint on the third drive (two-line bends 5 cm inside
    # whatever its probability); taking the minimum blended the model in for 23% of that drive and the model cut
    # the bends. The adopt/drop hysteresis still decides whether a line is used at all; the stds weight the fit.
    self.boundary_confidence = float(np.mean(probs)) if probs else 0.0
    if width_seen is not None and 2.0 < width_seen < s.max_width:
      if not math.isfinite(self.width_age_s):
        self.width = width_seen        # first pair ever seen: take it, do not filter from the 3.5 m placeholder
      self.width += (width_seen - self.width) * (1.0 - math.exp(-dt_frame / 2.0))
      self.width_age_s = 0.0
    else:
      self.width_age_s += dt_frame
    return True

  @staticmethod
  def _std_weight(b: Boundary, s: LaneLedSettings) -> float:
    limit = s.max_line_std if b.kind == "line" else s.edge_std
    return float(np.clip(1.0 - b.std / max(limit, 1e-6), 0.1, 1.0))

  # -- outputs
  def heading(self) -> float:
    return math.atan(float(self.c[1]) / self.H)

  def lost(self) -> bool:
    """True when the tracked state cannot be a lane any more (blown up while propagating blind, or non-finite)."""
    return (not np.isfinite(self.c).all() or abs(float(self.c[0])) > _LOST_OFFSET or abs(self.heading()) > _LOST_HEADING)

  def centre_at(self, x: float) -> float:
    return float(np.polyval(self.c[::-1], x / self.H))

  def slope_at(self, x: float) -> float:
    u = x / self.H; c = self.c
    return float((c[1] + 2.0 * c[2] * u + 3.0 * c[3] * u * u + 4.0 * c[4] * u ** 3) / self.H)

  def curvature_at(self, x: float) -> float:
    u = x / self.H
    c = self.c
    dy = (c[1] + 2.0 * c[2] * u + 3.0 * c[3] * u * u + 4.0 * c[4] * u ** 3) / self.H
    ddy = (2.0 * c[2] + 6.0 * c[3] * u + 12.0 * c[4] * u * u) / (self.H * self.H)
    return float(ddy / (1.0 + dy * dy) ** 1.5)

  def trust(self, s: LaneLedSettings) -> float:
    if self.mode == "none":
      return 0.0
    f_conf = float(np.clip((self.boundary_confidence - s.drop_confidence) / max(_CONFIDENCE_FULL - s.drop_confidence, 1e-6), 0.0, 1.0))
    f_res = float(np.clip(1.0 - (self.residual - _RESIDUAL_FULL) / (_RESIDUAL_ZERO - _RESIDUAL_FULL), 0.0, 1.0))
    f_inn = 0.7 if self.innovation_limited else 1.0
    return self.conf * f_conf * f_res * f_inn


# ----------------------------------------------------------------------------- C. delay state

class DelayState:
  """Ring buffer of the curvature actually published, seeded from the measured curvature, and the
  forward prediction of the error state over the lateral delay (Smith-predictor form)."""

  def __init__(self, horizon_s: float = 1.0):
    self.horizon = horizon_s
    self.reset()

  def reset(self):
    self.t = np.zeros(0)
    self.k = np.zeros(0)

  def record(self, t: float, kappa: float):
    if not math.isfinite(kappa):
      return
    if len(self.t) and t <= self.t[-1]:
      self.k[-1] = kappa
      return
    self.t, self.k = np.append(self.t, t), np.append(self.k, kappa)
    keep = self.t >= t - self.horizon - 0.1
    if not keep.all():
      self.t, self.k = self.t[keep], self.k[keep]

  def seed(self, t: float, kappa: float):
    if len(self.t) == 0 and math.isfinite(kappa):
      self.t, self.k = np.array([t - self.horizon, t]), np.array([kappa, kappa])

  def commanded_at(self, times: np.ndarray) -> np.ndarray:
    if len(self.t) == 0:
      return np.zeros(len(times))
    return np.interp(times, self.t, self.k, left=self.k[0], right=self.k[-1])

  def predict(self, now: float, e_y: float, e_psi: float, kappa_meas: float, speed: float,
              delay: float, tau_steer: float, road_curvature, lever: float = 0.0) -> tuple[float, float, float]:
    """Integrate the already-commanded curvature through a first-order steering response over [0, delay].
    Returns (e_y_hat, e_psi_hat, kappa_applied_at_delay).

    e_y and e_psi are the lane centre's offset and heading as seen at the camera, `lever` metres ahead of the rear
    axle. The camera point does not move along the car's heading: in a turn it moves along heading + lever * kappa.
    So the heading error that drives the lateral error, and the one the controller should damp, is
    e_psi - lever * kappa; with lever = 0 a steady bend leaves a heading error of lever * kappa and the position
    term has to cancel it, which parks the car inside every bend by 2 * damping * v * lever * kappa / omega."""
    n = max(1, int(round(delay / _DT_PREDICT)))
    h = delay / n
    taus = np.arange(n) * h
    # What the steering model says is applied now, from the commands that have had time to act (those older
    # than the delay). The gap to the measured curvature is a disturbance (camber, wind, plant mismatch) that
    # is assumed to persist over the delay; without this the predictor invents a heading error and the bias
    # integrator winds up against it.
    tau = max(tau_steer, 1e-3)
    hist_h = 0.02
    m = max(2, int(round(self.horizon / hist_h)))
    hist = self.commanded_at(now - delay - (m - 1 - np.arange(m)) * hist_h)
    a = hist_h / tau
    weights = a * (1.0 - a) ** np.arange(m - 1)[::-1]
    applied_now = float(weights @ hist[1:] + (1.0 - a) ** (m - 1) * hist[0])
    offset = kappa_meas - applied_now
    cmd = self.commanded_at(now - delay + taus) + offset   # the command that takes effect at now + tau
    kappa = kappa_meas
    for i in range(n):
      kappa += (cmd[i] - kappa) * h / tau
      e_psi += speed * (road_curvature(speed * taus[i]) - kappa) * h
      e_y += speed * (e_psi - lever * kappa) * h
    return e_y, e_psi - lever * kappa, kappa


# ----------------------------------------------------------------------------- C2. lever learner

_LEVER_RATE = 0.2                                          # m/s: the applied lever follows the learned (or switched) value at this rate
_LEVER_MAX_KAPPA = 1.0 / 60.0                              # bends tighter than this are not used (lines least reliable, slip largest)
_LEVER_MIN_SPEED = 5.0
_LEVER_SETTLE_S = 1.0                                      # lane-led this long before a frame counts (past the blend-in)
_LEVER_RATE_S = 1.0                                        # s of tracked-offset history the lateral drift rate is fitted over
_LEVER_MAX_DRIFT = 0.3                                     # m/s: faster across the lane than this is a lane change or a snap, not a sample
_LEVER_MAX_EY = 0.5
_GUARD_YIELD = 0.3                                         # m: a model path this much closer to the only line than the guard allows is an obstacle, not a hug
_BIAS_LEARN_EY = 0.15                                      # m: the bias integrates only this close to the target (change 27)
_FF_MODEL_MARGIN = 1.5e-3                                  # 1/m: the feedforward may differ from the model's curvature by this, or
_FF_MODEL_MARGIN_REL = 0.3                                 # by this fraction of it, whichever is larger (change 28)
_ONE_LINE_HEADING_SHARE = 1.0                              # the line's share of the one-line heading reference at full confidence (change 29).
                                                           # 1.0 = OFF: the closed-loop replay of routes 25 and 28 showed the model path's heading
                                                           # adding one-line request jerk (p95 1.43 -> 1.60) however the bend was removed; kept for the road
_PSI_MODEL_TAU = 0.15                                      # s: filter on the model path's heading (one-line reference)
_ONE_LINE_BEND = "model"                                   # the bend removed from the model path's heading: the model's own curvature, or the tracked curve's
_SHARE_RATE = 4.0                                          # 1/s: the heading reference's line share slews at this rate across mode switches
_ONE_LINE_INNOVATION = 1.0                                 # extra factor on the tracked heading's innovation with one line (1: none)
_FF_MODEL_MARGIN_ACCEL = 0.3                               # m/s^2: the feedforward clamp's margin is at least this much lateral acceleration


class LeverLearner:
  """Learns the camera's effective lever from steady bends while driving.

  The camera point moves, relative to the body, along L * kappa (it is L ahead of the axle the car yaws about) less
  the rear slip angle, which grows with lateral acceleration (v^2 * kappa). The lane heading the camera reads, e_psi,
  is that motion heading plus the car's true heading error at the lane's tangent, and the true heading error is
  exactly the camera's drift rate across the lane over the speed: e_y_dot = v * (e_psi - L_eff * kappa). So on every
  frame, steady bend or not,
      e_psi - e_y_dot / v = b + L0 * kappa + L2 * kappa * (v^2 - V_REF^2)
  with b a heading bias (camera yaw, model bias), and the lever the controller needs is L0 + L2 * (v^2 - V_REF^2):
  L0 is the lever at V_REF (18 m/s, where the slider's 1.5 m was fitted) and L2 the speed term. None of this
  depends on what the controller is doing: a wrong lever changes where the car parks across the lane, not its heading,
  which is what makes the lever learnable on the road (the second v8 drive: 1.9 m at 12 m/s, 1.4 at 16, 1.2 at 20,
  i.e. L2 about -0.0027). The drift rate is a one-second slope of the tracked offset, so tracker noise averages out
  in the regression rather than gating frames away (a stillness gate kept 25 s of a 40-minute drive). A ridge
  regression over lightly gated frames (see LaneLedController.update): the slider is the
  prior for L0, worth N_PRIOR frames at R 300; zero is the prior for b and for L2, the speed term being the one that
  could run away, so it is bounded (L2_RANGE) and its prior is set so one drive's worth of mixed-speed
  bends moves it most of the way. Centring the speed term on V_REF keeps the two levers from
  competing: with little speed variety L2 stays near zero and L0 carries the lever at the speeds driven. Memory is
  about an hour of R 300 bends (forgetting is paid per unit of bend information, so straights forget nothing)
  and the state is persisted by the host, so it carries across drives."""

  N_PRIOR = 1200.0                     # 60 s of accepted frames at 20 Hz
  K_REF, V_REF = 1.0 / 300.0, 18.0
  MEMORY = 20.0 * 3600.0               # memory: an hour's worth of R 300 frames. Forgetting is paid per unit of bend
                                       # information, so straights (which teach nothing) forget nothing and the
                                       # estimate never relaxes toward the slider during a long motorway run.
  L0_RANGE = (0.0, 3.0)
  L2_RANGE = (-0.006, 0.001)
  LEVER_RANGE = (0.0, 3.0)
  L_FLOOR = 0.5                        # the speed term may take the lever down to this fraction of L0 (change 30): on the fourth
                                       # drive L2 learned on country roads took the motorway lever to 0.1 m, an extrapolation

  def __init__(self):
    self.A = np.zeros((3, 3))
    self.b = np.zeros(3)
    self.n = 0.0                       # effective (decayed) accepted frames
    self.accepted = 0                  # accepted frames, monotonic (the host stores the state when this grows)
    self._theta = None
    self._prior = math.nan

  def observe(self, e_psi: float, kappa: float, speed: float):
    if not (math.isfinite(e_psi) and math.isfinite(kappa) and math.isfinite(speed)):
      return
    phi = np.array([1.0, kappa, kappa * (speed * speed - self.V_REF ** 2)])
    lam = 1.0 - min(1.0, (kappa / self.K_REF) ** 2) / self.MEMORY
    self.A = lam * self.A + np.outer(phi, phi)
    self.b = lam * self.b + phi * e_psi
    self.n = lam * self.n + 1.0
    self.accepted += 1
    self._theta = None

  def theta(self, prior: float) -> np.ndarray:
    """(b, L0, L2) for the given slider prior; solved only when the data or the prior changed."""
    if self._theta is None or prior != self._prior:
      scale = np.array([1.0, self.K_REF, self.K_REF * self.V_REF ** 2])
      reg = self.N_PRIOR * scale ** 2 * np.array([1.0, 1.0, 0.4])
      fallback = np.array([0.0, prior, 0.0])
      try:
        th = np.linalg.solve(self.A + np.diag(reg), self.b + reg * fallback)
      except np.linalg.LinAlgError:
        th = fallback
      if not np.isfinite(th).all():
        th = fallback
      th[1] = float(np.clip(th[1], *self.L0_RANGE))
      th[2] = float(np.clip(th[2], *self.L2_RANGE))
      self._theta, self._prior = th, prior
    return self._theta

  def lever(self, speed: float, prior: float) -> float:
    th = self.theta(prior)
    lo = max(self.LEVER_RANGE[0], self.L_FLOOR * float(th[1]))
    return float(np.clip(th[1] + th[2] * (speed * speed - self.V_REF ** 2), lo, self.LEVER_RANGE[1]))

  def state(self) -> dict:
    return {"v": 1, "A": self.A.tolist(), "b": self.b.tolist(), "n": self.n, "accepted": self.accepted}

  def load(self, state) -> bool:
    """Restore a persisted state; anything malformed is ignored and the learner starts from the prior."""
    try:
      A = np.asarray(state["A"], dtype=float); b = np.asarray(state["b"], dtype=float)
      n = float(state["n"]); accepted = int(state.get("accepted", 0))
    except (KeyError, TypeError, ValueError):
      return False
    if A.shape != (3, 3) or b.shape != (3,) or not np.isfinite(A).all() or not np.isfinite(b).all() or not math.isfinite(n):
      return False
    if n < 0.0 or (np.diag(A) < 0.0).any() or not np.allclose(A, A.T, rtol=1e-6, atol=1e-9):
      return False
    # n counts accepted frames less what bends forgot; straights forget nothing, so after a long day it passes MEMORY.
    # The old upper bound (MEMORY * 1.01) threw a whole day's learning away at the next drive (3 November, 85,000 frames).
    self.A, self.b, self.n, self.accepted = A, b, min(n, 4.0 * self.MEMORY), max(accepted, 0)
    self._theta = None
    return True


# ----------------------------------------------------------------------------- D. lane controller

class LaneController:
  def __init__(self):
    self.reset()

  def reset(self, keep_bias: bool = False):
    self.prev = math.nan
    self.prev_added = math.nan
    self.prev_ff = math.nan
    self.ff_step = 0.0
    self.sigma = 1.0
    if not keep_bias:
      self.bias = 0.0
    self.flags = {"accel_limited": False, "jerk_limited": False, "total_limited": False}

  @staticmethod
  def omega(s: LaneLedSettings) -> float:
    # Critically damped: a 30 cm step is within 10 cm after about 3.3/omega seconds.
    return 3.3 / max(s.recovery_time, _RECOVERY_FLOOR, 1e-3)

  def follow(self, kappa: float):
    """While not driving, track the given request so a hand-over starts from it with nothing added."""
    self.prev = kappa
    self.prev_ff = kappa
    self.prev_added = 0.0
    self.ff_step = 0.0

  def update(self, e_y: float, e_psi: float, kappa_ff: float, speed: float, dt: float,
             s: LaneLedSettings, *, learn_bias: bool, driving: bool, kappa_meas: float = math.nan) -> float:
    v = max(speed, 0.1)
    v_g = max(speed, s.gain_floor_speed)
    w = self.omega(s)
    k_y = w * w / (v_g * v_g)
    k_psi = 2.0 * s.damping * w / v_g
    added_limit = s.max_added_accel / (v * v)
    # The position term may use at most 60% of the feedback budget. Clipping the sum instead would let a large
    # offset saturate the heading term, which is the damping, and the recovery would overshoot and ring.
    e_max = 0.6 * added_limit / k_y
    clipped = abs(e_y) > e_max
    # Rate awareness. The demand's own curvature rate has to fit the jerk budget; otherwise the limiter turns
    # the loop into bang-bang with lag and the recovery rings. Find the bandwidth scale sigma (k_y ~ sigma^2,
    # k_psi ~ sigma, damping ratio unchanged) that keeps the implied rate within 90% of the budget, let it fall
    # at once and recover with a 0.5 s time constant. In normal driving sigma stays at 1.
    rate_budget = 0.9 * s.max_jerk / (v * v)
    dkappa = (kappa_ff - kappa_meas) if math.isfinite(kappa_meas) else 0.0
    a = 0.0 if clipped else abs(k_y * v * e_psi)      # rate of the position term
    b = 0.0 if s.rate_aware_road_exempt else abs(k_psi * v * dkappa)   # rate of the heading term (the road's own, if counted)
    if a > 1e-12:
      sigma = (-b + math.sqrt(b * b + 4.0 * a * rate_budget)) / (2.0 * a)
    elif b > 1e-12:
      sigma = rate_budget / b
    else:
      sigma = 1.0
    sigma = float(np.clip(sigma, 0.2, 1.0))
    if sigma < self.sigma:
      self.sigma = sigma
    else:
      self.sigma += (sigma - self.sigma) * (1.0 - math.exp(-dt / 0.5))
    k_y *= self.sigma * self.sigma
    k_psi *= self.sigma
    e_max = 0.6 * added_limit / k_y
    fb = k_y * float(np.clip(e_y, -e_max, e_max)) + k_psi * e_psi
    if s.bias_enabled and learn_bias:
      self.bias += k_y * e_y * dt / max(s.bias_time, 1e-3)
      bias_limit = s.bias_limit_accel / (v_g * v_g)     # a lateral-acceleration limit: camber needs more curvature at low speed
      self.bias = float(np.clip(self.bias, -bias_limit, bias_limit))
    elif not driving:
      self.bias *= math.exp(-dt / 60.0)
    if not s.bias_enabled:
      self.bias = 0.0
    added = fb + self.bias
    self.flags["accel_limited"] = abs(added) > added_limit
    added = float(np.clip(added, -added_limit, added_limit))
    # The comfort jerk limit applies to what the controller adds. The feedforward follows the road's own
    # curvature rate; limiting the sum would throttle bend entries and the correction together.
    if math.isnan(self.prev_added):
      self.prev_added = added
    step = s.max_jerk / (v * v) * dt
    added_limited = float(np.clip(added, self.prev_added - step, self.prev_added + step))
    self.flags["jerk_limited"] = added_limited != added
    self.prev_added = added_limited
    # The feedforward follows the road, bounded by its own rate limit (a bend sharper than this at this speed
    # is not something to follow anyway). Added plus feedforward stays well inside the host's 5 m/s^3 ISO limit.
    if math.isnan(self.prev_ff):
      self.prev_ff = kappa_ff
    step_ff = ROAD_JERK / (v * v) * dt
    ff_limited = float(np.clip(kappa_ff, self.prev_ff - step_ff, self.prev_ff + step_ff))
    self.ff_step = abs(ff_limited - self.prev_ff)
    self.prev_ff = ff_limited
    total = ff_limited + added_limited
    total_limit = s.max_lateral_accel / (v * v)
    self.flags["total_limited"] = abs(total) > total_limit
    total = float(np.clip(total, -total_limit, total_limit))
    self.prev = total
    return total


# ----------------------------------------------------------------------------- E. arbiter

class Arbiter:
  def __init__(self):
    self.reset()

  def reset(self):
    self.weight = 0.0
    self.prev = math.nan
    self.prev_model = math.nan
    self.low_trust_s = 0.0

  def update(self, kappa_lane: float, kappa_model: float, trust: float, veto: bool, speed: float,
             dt: float, s: LaneLedSettings, ff_step: float = 0.0, fade_in: float | None = None) -> tuple[float, float, bool]:
    target = 0.0 if veto else float(np.clip(trust / max(s.trust_full, 1e-3), 0.0, 1.0))
    if trust < 0.2:
      self.low_trust_s += dt
      if self.low_trust_s >= 0.5:
        target = 0.0
    else:
      self.low_trust_s = 0.0
    up, down = dt / max(s.fade_in if fade_in is None else max(fade_in, s.fade_in), 1e-3), dt / max(s.fade_out, 1e-3)
    self.weight = float(np.clip(target, self.weight - down, self.weight + up))
    candidate = self.weight * kappa_lane + (1.0 - self.weight) * kappa_model
    # Final guarantee: the published request changes by at most the comfort jerk budget plus whatever the road
    # part is doing at the current weight (the lane feedforward's step and the model's own change). A hand-over
    # therefore never steps however far apart the two requests are, and a model-led tick never throttles the model.
    v = max(speed, 0.1)
    model_step = abs(kappa_model - self.prev_model) if math.isfinite(self.prev_model) else 0.0
    self.prev_model = kappa_model
    if math.isnan(self.prev):
      self.prev = candidate
    allowed = s.max_jerk / (v * v) * dt + self.weight * ff_step + (1.0 - self.weight) * model_step
    limited = float(np.clip(candidate, self.prev - allowed, self.prev + allowed))
    jerk_limited = limited != candidate
    self.prev = limited
    return limited, self.weight, jerk_limited

  def follow(self, kappa: float):
    self.prev = kappa
    self.prev_model = kappa
    self.weight = 0.0


# ----------------------------------------------------------------------------- F. obstacle hold (change 37)

class ObstacleHold:
  """Hand the car to the model while an object is in the lane and until the car has driven past it.

  Each in-lane detection arms a hold to the odometer reading at which the object's far end plus _OBSTACLE_PASS_M is
  behind the camera; the narrow camera loses an object some metres before the car reaches it, so the distance, not
  the detection, decides when the lines may lead again. A detector that stops while a hold runs does not cut it short."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.odo = 0.0
    self.until = -math.inf
    self.last_t = -math.inf
    self.active = False
    self.return_s = 0.0          # time left of the slow hand-back after a hold ends

  def update(self, t: float, speed: float, dt: float, blocked: bool, distance: float, length: float) -> bool:
    self.odo += max(speed, 0.0) * dt
    if blocked and math.isfinite(distance):
      self.until = max(self.until, self.odo + max(distance, 0.0) + max(length, 0.0) + _OBSTACLE_PASS_M)
      self.last_t = t
    was = self.active
    self.active = blocked or self.odo < self.until or t - self.last_t < _OBSTACLE_MIN_HOLD_S
    if was and not self.active:
      self.return_s = _OBSTACLE_RETURN_S
    elif self.active:
      self.return_s = 0.0
    else:
      self.return_s = max(0.0, self.return_s - dt)
    return self.active

  @property
  def remaining(self) -> float:
    return max(0.0, self.until - self.odo) if math.isfinite(self.until) else 0.0


# ----------------------------------------------------------------------------- version 8 controller

class LaneLedController:
  """Lane version 8. Call update() every control tick (100 Hz); model-frame work runs when model_sample_id changes."""

  def __init__(self, settings: LaneLedSettings | None = None):
    self.saved = settings or LaneLedSettings()
    self.slew = SettingsSlew(self.saved)
    self.gate = BoundaryGate()
    self.tracker = CentreTracker()
    self.delay_state = DelayState()
    self.controller = LaneController()
    self.arbiter = Arbiter()
    self.lever_learner = LeverLearner()
    self.obstacle = ObstacleHold()
    self._lever = math.nan          # the lever in use, rate-limited so the toggle or a learned change never steps
    self._led_s = 0.0               # time lane-led without a veto (the learner waits for the loop to settle)
    self._ey_hist = deque()         # (t, e_y) over the last second or so: the learner needs the car's drift rate across the lane
    self._guard_l = math.nan        # clearance guards in the car frame (change 25): the car's y must stay >= _guard_l and <= _guard_r
    self._guard_r = math.nan
    self._psi_m = math.nan          # one-line heading reference from the model path (change 29), filtered
    self._psi_m_pending = 0.0; self._psi_m_pending_s = 0.0
    self._share_f = 1.0             # the line's share of the heading reference, slewed
    self._kappa_model_f = math.nan  # the model's curvature, filtered like the measured one (clamp reference, model bend)
    self._last_t = math.nan
    self._last_sample = None
    self._last_frame_t = math.nan
    self._gap_s = 0.0
    self._trust_f = 0.0
    self._kappa_meas_f = math.nan
    self._veto_reason = ""

  def reset(self):
    """Full reset: only for disengagement or a version switch, never for a data gap."""
    keep_bias = self.slew.effective.bias_persist
    self.gate.reset(); self.tracker.reset(); self.delay_state.reset()
    self.controller.reset(keep_bias=keep_bias); self.arbiter.reset(); self.obstacle.reset()
    self._last_t = math.nan; self._last_sample = None; self._last_frame_t = math.nan; self._gap_s = 0.0
    self._trust_f = 0.0; self._kappa_meas_f = math.nan; self._led_s = 0.0; self._ey_hist.clear()
    self._guard_l = math.nan; self._guard_r = math.nan
    self._psi_m = math.nan; self._psi_m_pending = 0.0; self._psi_m_pending_s = 0.0; self._share_f = 1.0; self._kappa_model_f = math.nan
    # The lever learner is a property of the car, not of this engagement: it survives resets like a persisted bias.

  def set_settings(self, settings: LaneLedSettings):
    if settings_valid(settings):
      self.saved = settings

  def lever_state(self) -> dict:
    return self.lever_learner.state()

  def load_lever_state(self, state) -> bool:
    return self.lever_learner.load(state)

  # -- helpers
  def _veto(self, obs: LaneLedObservation, s: LaneLedSettings) -> str:
    if obs.lane_change:
      return "lane_change"
    if obs.driver_override:
      return "driver_override"
    if obs.speed < s.min_speed:
      return "below_min_speed"
    if self.obstacle.active:
      return "obstacle"
    if obs.model_path_x is not None and obs.model_path_y is not None and self.tracker.mode != "none":
      x1 = obs.speed * 1.0
      try:
        my = float(np.interp(x1, obs.model_path_x, obs.model_path_y))
        ystd = float(np.interp(x1, obs.model_path_x, obs.model_path_ystd)) if obs.model_path_ystd is not None else 0.0
      except (TypeError, ValueError):
        return ""
      threshold = s.model_divergence_m if obs.speed >= 8.0 else min(s.model_divergence_m, 0.4)
      # Change 37: with the detector watching the lane, the model no longer needs this escape to steer round a parked
      # car or a cone (the obstacle hold does that, fully); only a gross disagreement, a wrong line, still vetoes.
      if obs.obstacle_ok:
        threshold = max(threshold, _DETECTOR_DIVERGENCE_M)
      dev = my - self.tracker.centre_at(x1)
      # With one line the tracked curve is the guard boundary, not a centre, and the lateral target follows the model's
      # path anyway (change 25), so the model's distance from it says nothing: on the fourth drive every lane wider than
      # two guard distances plus the threshold was vetoed with the car on target. What matters is the SHAPE: a model
      # path pulling away from the line's direction over the next second is a turn the line does not make. Toward the
      # line the absolute rule stays: a model path a little inside the guard is hugging, which the guard refuses; one
      # well inside it (_GUARD_YIELD) is the model steering round something on that side, a parked car, a cone, a
      # cyclist, which the model sees and the lines do not: the model wins.
      if self.tracker.mode in ("one_left", "one_right"):
        try:
          dev0 = float(np.interp(0.0, obs.model_path_x, obs.model_path_y)) - self.tracker.centre_at(0.0)
        except (TypeError, ValueError):
          return ""
        sign = 1.0 if self.tracker.mode == "one_left" else -1.0
        away, toward = max(0.0, sign * (dev - dev0)), max(0.0, -sign * dev)
      else:
        away, toward = abs(dev), 0.0
      if (away > threshold or (toward > _GUARD_YIELD and not obs.obstacle_ok)) and ystd < 0.3:
        return "model_divergence"
    return ""

  def update(self, obs: LaneLedObservation) -> LaneLedResult:
    # Time step; a long gap is a gap, not a reset.
    dt = obs.t - self._last_t if math.isfinite(self._last_t) else 0.01
    discontinuity = not (0.0 < dt <= _MAX_DT)
    if discontinuity:
      dt = 0.01
    self._last_t = obs.t
    s = self.slew.update(self.saved, dt)
    kappa_model = obs.model_curvature if math.isfinite(obs.model_curvature) else 0.0
    if not math.isfinite(self._kappa_model_f):
      self._kappa_model_f = kappa_model
    speed = max(obs.speed, 0.0) if math.isfinite(obs.speed) else 0.0
    kappa_raw = obs.measured_curvature if math.isfinite(obs.measured_curvature) else 0.0
    # The steering-angle curvature is quantised (a tenth of a degree is a step of about 5e-5 1/m); through the predictor's
    # heading gain that is nearly 1 m/s^3 of request jerk per step at 18 m/s. A 0.1 s filter removes it and lags only the
    # disturbance estimate; the tracked lane itself is carried by the (filtered) yaw rate.
    if not math.isfinite(self._kappa_meas_f):
      self._kappa_meas_f = kappa_raw
    self._kappa_meas_f += (kappa_raw - self._kappa_meas_f) * (1.0 - math.exp(-dt / _MEAS_TAU))
    kappa_meas = self._kappa_meas_f
    self._kappa_model_f += (kappa_model - self._kappa_model_f) * (1.0 - math.exp(-dt / _MEAS_TAU))
    kappa_model_f = self._kappa_model_f
    self.delay_state.seed(obs.t, kappa_meas)

    # Ego-motion propagation every tick, so the tracked centre stays in the current vehicle frame.
    yaw_rate = obs.yaw_rate if math.isfinite(obs.yaw_rate) else kappa_meas * speed
    # The lever in use: learned (the slider is its prior) or the slider itself, followed at a bounded rate.
    lever_target = self.lever_learner.lever(speed, s.axle_to_camera_m) if s.lever_learn else s.axle_to_camera_m
    if not math.isfinite(self._lever):
      self._lever = lever_target
    lever_step = _LEVER_RATE * dt
    self._lever = float(np.clip(lever_target, self._lever - lever_step, self._lever + lever_step))
    lever = self._lever
    self.obstacle.update(obs.t, speed, dt, bool(obs.obstacle_blocked), float(obs.obstacle_distance), float(obs.obstacle_length))
    self.tracker.set_horizon(speed)
    self.tracker.predict(speed * dt, yaw_rate * dt, lever)
    self.tracker.apply_pending(dt)

    # Model-frame work.
    new_frame = obs.model_sample_id != self._last_sample
    observed = False
    if new_frame and not obs.inputs_stale:
      dt_frame = obs.t - self._last_frame_t if math.isfinite(self._last_frame_t) else 0.05
      dt_frame = min(max(dt_frame, 1e-3), _MAX_DT)
      self._last_frame_t = obs.t
      self._last_sample = obs.model_sample_id
      self.gate.update(obs.boundaries, dt_frame, s)
      left, right = self.gate.selected()
      one_d = (self._guard_distance(left, s, obs) if left is not None else math.nan,
               self._guard_distance(right, s, obs) if right is not None else math.nan)
      if self.tracker.observe(left, right, dt_frame, s, one_d):
        self._gap_s = 0.0
        observed = True
      else:
        self._gap_s += dt          # a frame with nothing usable is a gap, not an observation of "no lane"
      self._update_guards(left, right, speed, dt_frame, s, obs)
    else:
      self._gap_s += dt
    if self._gap_s > _FRAME_GRACE:
      self.tracker.decay(dt)
    if self.tracker.lost() or (not obs.lateral_active and self._gap_s > _INACTIVE_GAP) or self._gap_s > s.hold_through_gap + _LONG_GAP:
      # The arbiter has long since faded to the model; a cubic propagated blind for this long is not a
      # lane estimate any more. Width memory survives (its own age limits apply).
      self.tracker.reset()
    # Trust holds for hold_through_gap while the tracker propagates on ego motion; beyond it the arbiter's
    # fade_out slew is the ramp to the model. Nothing resets.
    gap_factor = 1.0 if self._gap_s <= s.hold_through_gap else 0.0
    trust_raw = self.tracker.trust(s) * gap_factor * (0.5 if obs.limited_by_safety else 1.0)
    if not s.enabled:
      trust_raw = 0.0
    # A short filter so single-frame dips do not move the blend weight back and forth (that movement is jerk).
    # Vetoes and genuine hand-backs do not pass through here.
    self._trust_f += (trust_raw - self._trust_f) * (1.0 - math.exp(-dt / 0.3))
    trust = self._trust_f

    # Geometry the controller sees.
    # Geometry the controller sees. +y is right on the car, so a positive offset (Galaxy: right) puts the target centre
    # at -offset in the car frame. With one line there is no centre: the lateral target is where the model's own path
    # sits relative to the line's guard curve half a second ahead (the model knows the road, the line gives the
    # heading and curvature, the guards below keep the clearance); holding position instead drifted to the guard in
    # a bend, because nothing anchored the lateral position (change 25).
    centre = self.tracker.centre_at(0.0)
    if self.tracker.mode == "two":
      e_y_ref = centre + s.offset_m
    elif self.tracker.mode != "none" and obs.model_path_x is not None and obs.model_path_y is not None:
      xm = max(3.0, speed * 0.5)
      try:
        my = float(np.interp(xm, obs.model_path_x, obs.model_path_y))
        e_y_ref = centre + (my - self.tracker.centre_at(xm)) if math.isfinite(my) else 0.0
      except (TypeError, ValueError):
        e_y_ref = 0.0
    else:
      e_y_ref = 0.0
    # Clearance guards: the tracked curve is itself the guard for a single line; edges and the other line are raw.
    lo = self._guard_l if math.isfinite(self._guard_l) else -math.inf
    hi = self._guard_r if math.isfinite(self._guard_r) else math.inf
    if self.tracker.mode == "one_left":
      lo = max(lo, centre)
    elif self.tracker.mode == "one_right":
      hi = min(hi, centre)
    if lo > hi:
      e_y = 0.5 * (lo + hi)        # corridor narrower than the car plus clearances: aim for its middle
    else:
      e_y = float(np.clip(e_y_ref, lo, hi))
    guard_active = e_y != e_y_ref
    # The feedforward and the predictor's road curvature are the tracked curve's, bounded to the model's desired curvature
    # plus a margin (change 28): on a lane changing width (junctions, merges) the fit through converging lines reported a
    # 100 m bend the model did not see (fourth drive, 10e-3 against 0.5e-3) and the car swerved. In a real bend the two
    # agree within the margin, so nothing changes there. The reference is the filtered model curvature (the bound must not
    # carry the model's 20 Hz steps) and the margin is also at least _FF_MODEL_MARGIN_ACCEL of lateral acceleration, so
    # at town speed the model's own small corrections never bite.
    k_margin = max(_FF_MODEL_MARGIN, _FF_MODEL_MARGIN_REL * abs(kappa_model_f), _FF_MODEL_MARGIN_ACCEL / max(speed, 1.0) ** 2)
    k_lo, k_hi = kappa_model_f - k_margin, kappa_model_f + k_margin
    curvature_at = self.tracker.curvature_at

    def kappa_road(x: float) -> float:
      return min(max(curvature_at(x), k_lo), k_hi)

    e_psi_line = self.tracker.heading()
    if discontinuity:
      self._ey_hist.clear(); self._psi_m = math.nan
    e_psi = self._heading_reference(e_psi_line, obs, speed, observed, yaw_rate, kappa_road(0.0), kappa_model_f, dt, s)
    self._ey_hist.append((obs.t, e_y))
    while self._ey_hist and obs.t - self._ey_hist[0][0] > _LEVER_RATE_S + 0.05:
      self._ey_hist.popleft()
    if not obs.lateral_active:
      self._led_s = 0.0
      self.controller.follow(kappa_model)
      self.arbiter.follow(kappa_model)
      self.controller.update(0.0, 0.0, kappa_model, speed, dt, s, learn_bias=False, driving=False)
      self.delay_state.record(obs.t, obs.published_curvature if math.isfinite(obs.published_curvature) else kappa_model)
      return LaneLedResult(kappa_model, kappa_model, kappa_model, 0.0, "inactive", "lateral_inactive",
                           e_y, e_psi, kappa_road(0.0), self.controller.bias, trust, self.tracker.width,
                           self.tracker.mode, False, {"gap_s": self._gap_s, "lever": lever})

    # Delay-predicted state and feedforward.
    delay = obs.delay if math.isfinite(obs.delay) and 0.0 <= obs.delay <= 1.5 else 0.33
    e_y_hat, e_psi_hat, _ = self.delay_state.predict(obs.t, e_y, e_psi, kappa_meas, speed, delay, s.tau_steer, kappa_road,
                                                   lever=lever)
    x_p = speed * (delay + (s.tau_steer if s.ff_full_delay else 0.0)) * s.preview_factor
    kappa_ff = kappa_road(x_p)

    veto_reason = self._veto(obs, s)
    veto = bool(veto_reason)
    driving = self.arbiter.weight > 0.8 and not veto
    # The bias is for camber and a camera offset, a few 1e-4 of curvature. Learning it from every transient (a glitch
    # moving the target, the tail of a recovery) wound it up to its limit on the fourth drive and then held the car off
    # centre for ten seconds. Change 27: only near the target, and never while a limiter is shaping the request.
    learn_bias = (driving and self.tracker.mode == "two" and trust > 0.8 and abs(e_y) < _BIAS_LEARN_EY and
                  not any(self.controller.flags.values()))
    self._led_s = self._led_s + dt if driving else 0.0
    if observed and s.lever_learn:
      sample = self._lever_sample(obs.t, e_y, e_psi, yaw_rate, speed, trust, driving)
      if sample is not None:
        self.lever_learner.observe(sample[0], sample[1], speed)
    if self.arbiter.weight < 0.01:
      self.controller.follow(kappa_model)
    kappa_lane = self.controller.update(e_y_hat, e_psi_hat, kappa_ff, speed, dt, s, learn_bias=learn_bias, driving=driving,
                                        kappa_meas=kappa_meas)
    candidate, weight, jerk_limited = self.arbiter.update(kappa_lane, kappa_model, trust, veto, speed, dt, s,
                                                          ff_step=self.controller.ff_step,
                                                          fade_in=_OBSTACLE_RETURN_S if self.obstacle.return_s > 0.0 else None)
    self.delay_state.record(obs.t, obs.published_curvature if math.isfinite(obs.published_curvature) else candidate)

    state = "lane_led" if weight > 0.8 else "model_led" if weight < 0.2 else "blended"
    if veto:
      reason = veto_reason
    elif self._gap_s > _FRAME_GRACE:
      reason = "holding_through_gap" if gap_factor >= 1.0 else "gap_exceeded"
    elif self.tracker.mode == "none":
      reason = "no_boundaries"
    elif trust < 0.2:
      reason = "low_trust"
    elif obs.inputs_stale:
      reason = "inputs_stale"
    else:
      reason = "ok"
    limited = jerk_limited or any(self.controller.flags.values())
    return LaneLedResult(candidate, kappa_lane, kappa_model, weight, state, reason, e_y, e_psi, kappa_ff,
                         self.controller.bias, trust, self.tracker.width, self.tracker.mode, limited,
                         {"e_y_hat": e_y_hat, "e_psi_hat": e_psi_hat, "gap_s": self._gap_s, "residual": self.tracker.residual,
                          "innovation_limited": self.tracker.innovation_limited, "conf": self.tracker.conf, "bridge_s": self.tracker.bridge_s,
                          "guard_l": self._guard_l, "guard_r": self._guard_r, "guard_active": guard_active, "discontinuity": discontinuity,
                          "e_psi_line": e_psi_line, "ff_clamped": kappa_ff != curvature_at(x_p),
                          "boundary_confidence": self.tracker.boundary_confidence,
                          "omega": LaneController.omega(s), "bandwidth_scale": self.controller.sigma, "ff_step": self.controller.ff_step,
                          "lever": lever, "lever_theta": self.lever_learner.theta(s.axle_to_camera_m) if s.lever_learn else None,
                          "obstacle_ok": bool(obs.obstacle_ok), "obstacle_hold": self.obstacle.active,
                          "obstacle_left_m": self.obstacle.remaining,
                          "lever_n": self.lever_learner.n,
                          "flags": dict(self.controller.flags)})

  def _heading_reference(self, e_psi_line: float, obs: LaneLedObservation, speed: float, observed: bool, yaw_rate: float,
                         kappa_road0: float, kappa_model_f: float, dt: float, s: LaneLedSettings) -> float:
    """The heading reference. With two lines it is the tracked centre's heading. With one line that heading is twice as
    noisy (one line instead of the mean of two) and the heading term is the strong one at town speeds, so on the fourth
    drive one-line requests were twice as large and mostly clipped by the host. The model's path, which the lateral
    target already follows (change 25), is smooth. Change 29: with one line the reference is the model path's heading at
    the lateral target point, less the model's own bend out to that point (so a bend is not read as a heading error),
    blended with the line's heading, the line's share falling with its confidence and slewing at mode switches. The model heading is
    read once per frame, spread over the frame period and filtered, like the tracked curve, and carried between frames
    by the car's yaw relative to the road (the road turns too). It starts from the line's heading when one-line mode
    begins, so the switch has no step."""
    mode = self.tracker.mode
    one = mode in ("one_left", "one_right")
    have_path = obs.model_path_x is not None and obs.model_path_y is not None
    f_conf = float(np.clip((self.tracker.boundary_confidence - s.drop_confidence) / max(_CONFIDENCE_FULL - s.drop_confidence, 1e-6), 0.0, 1.0))
    target = _ONE_LINE_HEADING_SHARE * f_conf if (one and have_path and math.isfinite(self._psi_m)) else 1.0
    step = _SHARE_RATE * dt
    self._share_f = float(np.clip(target, self._share_f - step, self._share_f + step))   # no step at a mode switch
    if not have_path:
      self._psi_m = math.nan; self._psi_m_pending = 0.0; self._psi_m_pending_s = 0.0
      return e_psi_line
    # The model heading runs in every mode (it is cheap), so it is settled when a one-line stretch begins.
    if not math.isfinite(self._psi_m):
      self._psi_m = e_psi_line; self._psi_m_pending = 0.0; self._psi_m_pending_s = 0.0
    self._psi_m += (speed * kappa_road0 - yaw_rate) * dt
    if observed:
      xm = max(3.0, speed * 0.5)
      try:
        ya, yb = (float(v) for v in np.interp([xm - 1.0, xm + 1.0], obs.model_path_x, obs.model_path_y))
      except (TypeError, ValueError):
        ya = yb = math.nan
      if math.isfinite(ya) and math.isfinite(yb):
        # The path's heading at xm less the bend out to xm. The bend is the model's own (its filtered curvature times
        # the distance): the tracked curve's bend from a single line is as noisy as the heading this replaces.
        bend = kappa_model_f * xm if _ONE_LINE_BEND == "model" else (math.atan(self.tracker.slope_at(xm)) - e_psi_line)
        psi_model = math.atan(0.5 * (yb - ya)) - bend
        self._psi_m_pending = (psi_model - self._psi_m) * (1.0 - math.exp(-0.05 / _PSI_MODEL_TAU))
        self._psi_m_pending_s = 0.05
    if self._psi_m_pending_s > 0.0:
      f = min(1.0, dt / self._psi_m_pending_s)
      self._psi_m += self._psi_m_pending * f; self._psi_m_pending *= (1.0 - f); self._psi_m_pending_s -= dt
    share = self._share_f
    return share * e_psi_line + (1.0 - share) * self._psi_m

  def _guard_distance(self, b: Boundary, s: LaneLedSettings, obs: LaneLedObservation) -> float:
    """How far from boundary b the car's centre is held: car half width plus the line or edge clearance, CAPPED so the
    far side of the car stays inside the lane. The lane width for the cap is the two-line width seen within the last
    20 s of frames (narrow pairs count), else twice the model path's distance from this boundary (the model's path is
    about the lane's middle), never below min_width so a hugging model path cannot disarm the guard. Without the cap a
    single line on a lane narrower than the car plus two clearances would push the car over the unseen line. The cap
    only ever reduces a push away from a boundary."""
    d = 0.5 * s.car_width + (s.line_clearance if b.kind == "line" else s.edge_clearance)
    if self.tracker.width_age_s <= 20.0:   # seconds of observed frames since two lines were last seen
      w_ref = self.tracker.width
    elif obs.model_path_x is not None and obs.model_path_y is not None:
      try:
        xm = max(3.0, obs.speed * 0.5)
        w_ref = 2.0 * abs(float(np.interp(xm, obs.model_path_x, obs.model_path_y)) - float(np.interp(min(xm, float(b.x[-1])), b.x, b.y)))
      except (TypeError, ValueError):
        w_ref = s.min_width
      w_ref = max(w_ref, s.min_width) if math.isfinite(w_ref) else s.min_width   # a hugging model path cannot disarm the guard
    else:
      w_ref = s.min_width
    d_max = max(w_ref, 2.0) - 0.5 * s.car_width - 0.1
    return max(0.5 * s.car_width, min(d, d_max))

  def _update_guards(self, left, right, speed, dt_frame, s: LaneLedSettings, obs: LaneLedObservation):
    """Clearance guards from this frame's adopted boundaries (change 25). For each line (and, with edge_guard, each
    road edge) the car's side must stay car_width / 2 plus the clearance away from it, now and one second ahead.
    Left boundaries (at -y) give a lower bound on the car's y, right ones an upper bound; the values are filtered
    over 0.3 s so a jittery edge estimate cannot jolt the request, and vanish when no such boundary is adopted."""
    lo, hi = -math.inf, math.inf
    bounds = [b for b in (left, right) if b is not None]
    if s.edge_guard:
      bounds += [b for b in self.gate.selected_edges() if b is not None]
    # The look-ahead point is taken relative to the tracked lane shape, not along the car's heading: in a bend the
    # lines ahead swing toward +y or -y with the road, which is not a boundary closing in.
    x1 = max(speed, 1.0) * 1.0
    shape = (self.tracker.centre_at(x1) - self.tracker.centre_at(0.0)) if self.tracker.mode != "none" else 0.0
    for b in bounds:
      xb = min(x1, float(b.x[-1]))
      y0 = float(np.interp(0.0, b.x, b.y)); y1 = float(np.interp(xb, b.x, b.y)) - shape * (xb / x1)
      d = self._guard_distance(b, s, obs)
      if b.side == 0:
        lo = max(lo, max(y0, y1) + d)
      else:
        hi = min(hi, min(y0, y1) - d)
    a = 1.0 - math.exp(-dt_frame / 0.3)
    for name, value in (("_guard_l", lo), ("_guard_r", hi)):
      if not math.isfinite(value):
        setattr(self, name, math.nan)
      elif not math.isfinite(getattr(self, name)):
        setattr(self, name, value)
      else:
        setattr(self, name, getattr(self, name) + (value - getattr(self, name)) * a)

  def _lever_sample(self, t, e_y, e_psi, yaw_rate, speed, trust, driving):
    """The lever learner's sample for this frame, (heading of the camera point's own motion, yaw curvature), or None.
    The motion heading is the measured lane heading less the car's true heading error, which is the drift rate of the
    tracked offset over the speed (a one-second slope of the offset history). Exact at every instant, so no steady
    bend is needed; the gates only keep out frames whose geometry is not the lane's: lane-led past the blend-in, two
    trusted lines, no innovation clip or gap, within _LEVER_MAX_EY of centre, above walking pace, not a tight bend,
    not crossing the lane faster than _LEVER_MAX_DRIFT (a lane change or a snapped track)."""
    if not (driving and self._led_s >= _LEVER_SETTLE_S and speed >= _LEVER_MIN_SPEED and trust > 0.8):
      return None
    if self.tracker.mode != "two" or self.tracker.innovation_limited or self._gap_s > 0.0 or abs(e_y) > _LEVER_MAX_EY:
      return None
    kappa = yaw_rate / speed
    if not math.isfinite(kappa) or abs(kappa) > _LEVER_MAX_KAPPA:
      return None
    if len(self._ey_hist) < 10 or t - self._ey_hist[0][0] < 0.9 * _LEVER_RATE_S:
      return None
    ts = np.fromiter((h[0] for h in self._ey_hist), dtype=float, count=len(self._ey_hist))
    ys = np.fromiter((h[1] for h in self._ey_hist), dtype=float, count=len(self._ey_hist))
    ts = ts - ts.mean(); den = float(ts @ ts)
    if den <= 0.0:
      return None
    drift = float(ts @ (ys - ys.mean())) / den
    if not math.isfinite(drift) or abs(drift) > _LEVER_MAX_DRIFT:
      return None
    return e_psi - drift / speed, kappa
