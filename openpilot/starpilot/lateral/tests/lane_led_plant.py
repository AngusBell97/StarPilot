"""Kinematic plant for lane-led tests and probes: a road, a car with steering dead time and a first-order
steering response, and a synthetic model frame shaped like modelV2 (lines, edges, path, curvature).

Frame: x forward, y right, positive curvature turns toward +y. Same conventions as lane_led.py and the
v7 simulator's Road class.
"""
from dataclasses import dataclass, field
import math
import numpy as np

from openpilot.starpilot.lateral.lane_led import Boundary, LaneLedObservation, LaneLedController, ROAD_JERK

DT = 0.01
MODEL_DT = 0.05
X_IDXS = 192.0 * (np.arange(33) / 32.0) ** 2
_LINE_X = X_IDXS[X_IDXS <= 100.0]


class Road:
  def __init__(self, segments, width=3.5, ds=0.1):
    """segments: list of (length_m, curvature_start, curvature_end); 100 m straight before, 600 m after."""
    ks = [np.zeros(int(100 / ds))]
    for length, k0, k1 in segments:
      ks.append(np.linspace(k0, k1, max(1, int(round(length / ds))), endpoint=False))
    ks.append(np.zeros(int(600 / ds)))
    k = np.concatenate(ks)
    self.ds = ds
    self.k = k
    self.s = np.arange(len(k)) * ds - 100.0
    h = np.concatenate([[0.0], np.cumsum(k[:-1] * ds)])
    self.h = h
    # Midpoint rule: each 0.1 m step goes along the heading at its middle, so h[i] is the tangent AT point i (plain
    # Euler left the recorded heading 0.17 mrad ahead of the geometric tangent on R 300, which reads as 5 cm of lever).
    hm = h[:-1] + 0.5 * k[:-1] * ds
    self.X = np.concatenate([[0.0], np.cumsum(np.cos(hm) * ds)]) - 100.0
    self.Y = np.concatenate([[0.0], np.cumsum(np.sin(hm) * ds)])
    self.nx, self.ny = -np.sin(h), np.cos(h)          # +y normal: the side positive curvature turns toward. On the car that is RIGHT
                                                       # (side 0, openpilot's left line, sits at -y; verified from lane changes)
    self.width = width

  def start(self, offset=0.0, heading=0.0):
    i = int(round(100.0 / self.ds))
    return self.X[i] + self.nx[i] * offset, self.Y[i] + self.ny[i] * offset, self.h[i] + heading, i

  def project(self, X, Y, hint):
    lo, hi = max(0, hint - 50), min(len(self.s), hint + 400)
    d = (self.X[lo:hi] - X) ** 2 + (self.Y[lo:hi] - Y) ** 2
    return lo + int(np.argmin(d))

  def frame(self, X, Y, psi, idx, width=None, edge_offset=0.5, right_edge_offset=0.5):
    """Road geometry in the car frame: centre, left/right lines and edges on _LINE_X, truth at x = 0."""
    w = self.width if width is None else width
    lo, hi = max(0, idx - int(10 / self.ds)), min(len(self.s), idx + int(130 / self.ds))
    c, s = math.cos(psi), math.sin(psi)

    def to_car(px, py):
      return c * (px - X) + s * (py - Y), -s * (px - X) + c * (py - Y)

    cx, cy = to_car(self.X[lo:hi], self.Y[lo:hi])
    ok = np.diff(cx) > 0
    if not ok.all():
      cut = int(np.argmin(ok)) + 1
      cx, cy = cx[:cut], cy[:cut]
      lo_n = lo + cut
    else:
      lo_n = hi
    nx, ny = self.nx[lo:lo_n], self.ny[lo:lo_n]
    out = {"centre": np.interp(_LINE_X, cx, cy), "x": _LINE_X.copy()}
    for name, off in (("left", -w / 2), ("right", w / 2), ("edge_left", -w / 2 - edge_offset), ("edge_right", w / 2 + right_edge_offset)):
      px, py = to_car(self.X[lo:lo_n] + nx * off, self.Y[lo:lo_n] + ny * off)
      out[name] = np.interp(_LINE_X, px, py)
    out["e_y_true"] = float(np.interp(0.0, cx, cy))
    hx = self.h[lo:lo_n]
    out["e_psi_true"] = float(np.interp(0.0, cx, hx)) - psi
    out["k_road"] = lambda x: float(np.interp(x, cx, self.k[lo:lo_n]))
    return out


class Car:
  def __init__(self, X, Y, psi, speed, delay, tau_steer, kappa=0.0):
    self.X, self.Y, self.psi, self.speed = X, Y, psi, speed
    self.delay, self.tau = delay, tau_steer
    self.kappa = kappa                     # actual path curvature
    self.cmd_t, self.cmd_k = [-10.0], [kappa]

  def command(self, t, kappa):
    self.cmd_t.append(t); self.cmd_k.append(kappa)
    if len(self.cmd_t) > 400:
      del self.cmd_t[:200]; del self.cmd_k[:200]

  def step(self, t, dt, disturbance=0.0):
    effective = float(np.interp(t - self.delay, self.cmd_t, self.cmd_k))
    self.kappa += (effective + disturbance - self.kappa) * dt / max(self.tau, 1e-3)
    # Midpoint rule: move along the heading at mid-step, so the heading at the end of the step is the path's tangent
    # there (moving along the end heading left the car 0.3 mrad behind its own tangent at 18 m/s on R 300).
    dpsi = self.speed * self.kappa * dt
    psi_mid = self.psi + 0.5 * dpsi
    self.X += self.speed * math.cos(psi_mid) * dt
    self.Y += self.speed * math.sin(psi_mid) * dt
    self.psi += dpsi


@dataclass
class Scenario:
  road: Road
  speed: float = 18.0
  delay: float = 0.33                 # the car's real dead time
  assumed_delay: float | None = None  # what the controller is told (None: the truth)
  tau_steer: float = 0.15
  initial_offset: float = 0.0         # car position relative to centre, along +y (right on the car)
  initial_heading: float = 0.0
  camera_lead: float = 1.5            # m from the rear axle (the Car's point) to the camera that sees the lane
  slip_gain: float = 0.0              # rad per (m/s^2): rear slip rotates the body into the bend by slip_gain * v^2 * kappa
  probs: tuple = (0.95, 0.95)
  stds: tuple = (0.05, 0.05)
  noise: float = 0.0                  # m, per-frame lateral noise on each line
  dropouts: tuple = ()                # (t0, t1) windows with no boundaries at all
  stale: tuple = ()                   # (t0, t1) windows with no new model frame
  lane_changes: tuple = ()            # (t0, t1) windows with lane_change=True
  one_line_from: float | None = None  # after this time only the left line (and edges) are reported
  switch_left_to_edge_at: float | None = None  # left line vanishes; left edge sits 0.3 m further out
  switch_left_to_edge_until: float | None = None  # the left line returns at this time (None: never)
  disturbance: float = 0.0            # curvature the car follows that was never commanded (camber, wind)
  model_lean: float = 0.0             # model curvature bias
  model_offset: float | None = None   # model path aims here relative to centre (None: holds the car's start offset)
  activate_at: float = 0.0            # lateral_active from this time
  duration: float = 15.0
  width: float | None = None
  edge_offset: float = 0.5            # the LEFT road edge sits this far outside the left line (negative: inside, to exercise the edge guard)
  right_edge_offset: float = 0.5
  speed_fn: object = None             # optional t -> speed
  seed: int = 0


def model_frame(road, car, t, sc: Scenario, rng, geom):
  w = sc.width or road.width
  noise_l = rng.normal(0.0, sc.noise) if sc.noise else 0.0
  noise_r = rng.normal(0.0, sc.noise) if sc.noise else 0.0
  x = geom["x"]
  boundaries = []
  if not any(a <= t < b for a, b in sc.dropouts):
    left_gone = (sc.switch_left_to_edge_at is not None and t >= sc.switch_left_to_edge_at and
                 (sc.switch_left_to_edge_until is None or t < sc.switch_left_to_edge_until))
    if not left_gone:
      boundaries.append(Boundary(0, "line", x, geom["left"] + noise_l, sc.stds[0], sc.probs[0]))
    if not (sc.one_line_from is not None and t >= sc.one_line_from):
      boundaries.append(Boundary(1, "line", x, geom["right"] + noise_r, sc.stds[1], sc.probs[1]))
    edge_left = geom["edge_left"] - (0.3 if left_gone else 0.0) + noise_l
    boundaries.append(Boundary(0, "edge", x, edge_left, 0.3))
    boundaries.append(Boundary(1, "edge", x, geom["edge_right"] + noise_r, 0.3))
  model_offset = sc.model_offset if sc.model_offset is not None else sc.initial_offset
  path_y = geom["centre"] + model_offset
  k_model = geom["k_road"](car.speed * sc.delay) + sc.model_lean
  return tuple(boundaries), x, path_y, np.full(len(x), 0.1), k_model


def run(controller: LaneLedController, sc: Scenario, record_diag=False):
  rng = np.random.default_rng(sc.seed)
  road = sc.road
  X, Y, psi, idx = road.start(sc.initial_offset, sc.initial_heading)
  car = Car(X, Y, psi, sc.speed, sc.delay, sc.tau_steer)
  n = int(round(sc.duration / DT))
  cols = {k: np.zeros(n) for k in ("t", "e_y_true", "e_psi_true", "e_y", "candidate", "kappa_lane", "kappa_model",
                                   "weight", "trust", "bias", "kappa_car", "speed", "max_jerk", "k_road", "kappa_ff", "ff_step", "e_psi", "lever", "lever_n")}
  states, modes, reasons = [], [], []
  sample_id, next_model_t, frame = 0, 0.0, None
  published = math.nan
  for i in range(n):
    t = i * DT
    if sc.speed_fn is not None:
      car.speed = float(sc.speed_fn(t))
    # The model sees the lane from the camera, camera_lead ahead of the axle; truth (e_y_true, e_psi_true) is taken there too.
    # With rear slip the body points into the bend more than the path does; the camera and the lines move with the body.
    psi_body = car.psi + sc.slip_gain * car.speed * car.speed * car.kappa
    cX, cY = car.X + sc.camera_lead * math.cos(psi_body), car.Y + sc.camera_lead * math.sin(psi_body)
    idx = road.project(cX, cY, idx)
    geom = road.frame(cX, cY, psi_body, idx, sc.width, sc.edge_offset, sc.right_edge_offset)
    if t >= next_model_t - 1e-9:
      next_model_t += MODEL_DT
      if not any(a <= t < b for a, b in sc.stale):
        frame = model_frame(road, car, t, sc, rng, geom)
        sample_id += 1
    if frame is None:
      frame = model_frame(road, car, t, sc, rng, geom)
      sample_id += 1
    boundaries, px, py, pystd, k_model = frame
    obs = LaneLedObservation(
      t=t, speed=car.speed, measured_curvature=car.kappa, model_curvature=k_model,
      delay=sc.assumed_delay if sc.assumed_delay is not None else sc.delay,
      boundaries=boundaries, model_sample_id=sample_id, published_curvature=published,
      yaw_rate=car.kappa * car.speed, model_path_x=px, model_path_y=py, model_path_ystd=pystd,
      lateral_active=t >= sc.activate_at, lane_change=any(a <= t < b for a, b in sc.lane_changes))
    res = controller.update(obs)
    published = res.candidate_curvature
    car.command(t, published)
    car.step(t, DT, sc.disturbance)
    cols["t"][i] = t; cols["e_y_true"][i] = geom["e_y_true"]; cols["e_psi_true"][i] = geom["e_psi_true"]
    cols["e_y"][i] = res.e_y; cols["candidate"][i] = res.candidate_curvature; cols["kappa_lane"][i] = res.kappa_lane
    cols["kappa_model"][i] = res.kappa_model; cols["weight"][i] = res.weight; cols["trust"][i] = res.trust
    cols["bias"][i] = res.bias; cols["kappa_car"][i] = car.kappa; cols["speed"][i] = car.speed; cols["kappa_ff"][i] = res.kappa_road
    cols["ff_step"][i] = res.diagnostics.get("ff_step", 0.0)
    cols["e_psi"][i] = res.e_psi; cols["lever"][i] = res.diagnostics.get("lever", math.nan); cols["lever_n"][i] = res.diagnostics.get("lever_n", 0.0)
    cols["max_jerk"][i] = controller.slew.effective.max_jerk
    cols["k_road"][i] = geom["k_road"](car.speed * sc.delay)
    states.append(res.state); modes.append(res.mode); reasons.append(res.reason)
  cols["state"] = np.array(states); cols["mode"] = np.array(modes); cols["reason"] = np.array(reasons)
  return cols


def step_metrics(res, start_t: float, band=0.10):
  """Time to come within `band` of centre and stay there, overshoot, sign changes of e_y_true after start_t."""
  m = res["t"] >= start_t
  t, e = res["t"][m], res["e_y_true"][m]
  inside = np.abs(e) <= band
  settle = math.inf
  for j in range(len(t)):
    if inside[j:].all():
      settle = t[j] - start_t
      break
  sign0 = np.sign(e[0]) if e[0] != 0 else 1.0
  overshoot = float(max(0.0, (-sign0 * e).max()))
  s = np.sign(e[np.abs(e) > 0.01])
  changes = int(np.count_nonzero(np.diff(s) != 0)) if len(s) > 1 else 0
  return {"settle_s": settle, "overshoot_m": overshoot, "sign_changes": changes, "final_m": float(e[-1])}


def max_step_ratio(res):
  """Largest |candidate step| relative to what the tick allows: the comfort jerk limit in force (on what the
  controller adds) plus the road's own curvature change at the preview point (the feedforward follows the road).
  1.0 means at the limit."""
  dk = np.abs(np.diff(res["candidate"]))
  v = np.maximum(res["speed"][1:], 0.1)
  road = np.abs(np.diff(res["k_road"]))
  model = np.abs(np.diff(res["kappa_model"]))   # the model's own 20 Hz steps pass through at (1 - weight)
  w = res["weight"][1:]
  slack = 0.05 / (v * v) * DT          # 0.05 m/s^3 for the tracked feedforward not matching the road exactly
  # The feedforward the arbiter passes is the controller's rate-bounded (ROAD_JERK) feedforward step. On a hand-over
  # it starts from the model's curvature and catches up to the road at that bound, so during the fade it exceeds the
  # road's own step at the car; crediting the actual bounded step keeps the ratio honest. The road term remains the
  # allowance for a steady lane-led bend.
  ff = np.minimum(res["ff_step"][1:], ROAD_JERK / (v * v) * DT) if "ff_step" in res else 0.0
  allowed = res["max_jerk"][1:] / (v * v) * DT + w * np.maximum(1.5 * road, ff) + (1.0 - w) * model + slack
  return float((dk / allowed).max())
