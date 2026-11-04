"""Lane version 10 settings (a copy of the v8 settings at the split, 4 November 2026; the two diverge from here).

Lane-led centring (lane version 8): settings, ranges, validation and mid-drive slewing.

Every numeric field is a Galaxy slider (see PLAN.md section 8) except those marked fixed.
A changed slider never takes effect in one tick: SettingsSlew moves the effective value
linearly over `slew_time` so a mid-drive adjustment cannot itself cause a jerk.
"""
from dataclasses import dataclass, fields, replace
import math


@dataclass(frozen=True)
class LaneLedSettings:
  enabled: bool = True
  pause_on_signal: bool = True   # a turn signal hands to the model like a lane change
  # Position
  offset_m: float = 0.0            # target shift from the geometric centre along +y, which is RIGHT on the car (verified from lane changes)
  axle_to_camera_m: float = 1.5    # rear axle to camera: the lane heading is seen at the camera, the car's heading is set at the axle
  lever_learn: bool = True         # learn the effective lever from steady bends (the slider is the prior); off uses the slider as a constant
  # Recovery (defaults are the 1 November closed-loop replay tune v8-tune-1; see tune/ and REPLAY.md)
  recovery_time: float = 3.2       # s for a 30 cm offset to come within 10 cm
  damping: float = 0.9             # damping ratio; 1 is critical
  preview_factor: float = 1.2      # feedforward is read at preview_factor * delay ahead
  # Limits
  max_added_accel: float = 0.55     # m/s^2 feedback plus bias above the feedforward
  max_jerk: float = 0.7            # m/s^3 rate of change of the controller's request
  gain_floor_speed: float = 6.0    # m/s below which feedback gains stop rising
  max_lateral_accel: float = 3.0   # m/s^2 car limit (fixed, matches clip_curvature)
  # Trust
  adopt_confidence: float = 0.33
  drop_confidence: float = 0.15
  max_line_std: float = 0.6
  edge_std: float = 1.0
  switch_hold: float = 0.2        # s a boundary change must persist before it is adopted or dropped
  min_width: float = 2.5
  max_width: float = 4.8
  innovation_limit_m: float = 0.05     # largest centre move accepted from one model frame
  innovation_limit_deg: float = 0.5    # largest heading move accepted from one model frame
  # One line and clearance guards (change 25: no remembered width; a single line gives the heading and a minimum clearance)
  car_width: float = 1.85          # m, body without mirrors (Tesla Model 3)
  line_clearance: float = 0.6      # m between the car's side and a lane line the guard keeps it off
  edge_clearance: float = 0.5      # m between the car's side and a road edge the guard keeps it off
  edge_guard: bool = True          # road edges push (never attract): the 1 November decision that edges never define the centre stands
  # Hand-over
  fade_in: float = 0.75
  fade_out: float = 0.5
  model_divergence_m: float = 0.55
  hold_through_gap: float = 0.5
  # Bias learning
  bias_enabled: bool = True
  bias_time: float = 8.0
  bias_limit_accel: float = 0.1    # m/s^2 of lateral acceleration the bias may hold (change 26: a curvature limit starved it on slow cambered
                                   # roads; change 27: 0.3 let transients wind it up to 3e-3 on the fourth drive, 0.1 is twice the old cap at 11 m/s)
  bias_persist: bool = False
  # Readout
  readout: bool = True
  show_bias: bool = False
  # Bend handling and blend decisiveness (added 1 November after the first drive; defaults keep the installed behaviour)
  ff_full_delay: bool = True       # read the feedforward at lateral delay + steering lag (default since 1 Nov: halves bend cutting)
  rate_aware_road_exempt: bool = True   # rate-awareness ignores the road's own curvature rate (default since 1 Nov: keeps bandwidth in bends)
  trust_full: float = 1.0          # trust at or above this gives full lane-led weight; 1.0 is the plain mapping
  heading_innovation: float = 1.0  # fraction of each frame's heading innovation the tracker takes (0.5 lagged the lane heading and hurt centring in closed loop)
  # Fixed
  slew_time: float = 2.0
  tau_steer: float = 0.15          # first-order steering response used by the delay predictor
  min_speed: float = 1.0           # below this the model drives (all-speeds decision keeps this tiny)


# (low, high) inclusive, for validation and the Galaxy slider ranges.
RANGES = {
  "offset_m": (-0.30, 0.30),
  "axle_to_camera_m": (0.0, 3.0),
  "recovery_time": (1.5, 6.0),
  "damping": (0.7, 1.5),
  "preview_factor": (0.5, 2.0),
  "max_added_accel": (0.2, 1.0),
  "max_jerk": (0.2, 1.0),
  "gain_floor_speed": (2.0, 10.0),
  "max_lateral_accel": (1.0, 3.0),
  "adopt_confidence": (0.10, 0.60),
  "drop_confidence": (0.05, 0.40),
  "max_line_std": (0.3, 1.0),
  "edge_std": (0.5, 1.5),
  "switch_hold": (0.1, 1.0),
  "min_width": (2.0, 3.5),
  "max_width": (3.5, 6.0),
  "innovation_limit_m": (0.02, 0.15),
  "innovation_limit_deg": (0.1, 2.0),
  "car_width": (1.5, 2.5),
  "line_clearance": (0.2, 1.5),
  "edge_clearance": (0.2, 1.5),
  "fade_in": (0.5, 3.0),
  "fade_out": (0.2, 2.0),
  "model_divergence_m": (0.3, 1.2),
  "hold_through_gap": (0.2, 1.0),
  "bias_time": (3.0, 30.0),
  "bias_limit_accel": (0.05, 0.6),
  "slew_time": (0.5, 5.0),
  "tau_steer": (0.05, 0.5),
  "min_speed": (0.0, 5.0),
  "trust_full": (0.4, 1.0),
  "heading_innovation": (0.3, 1.0),
}

_NUMERIC = tuple(f.name for f in fields(LaneLedSettings) if f.type is float)
_BOOLEAN = tuple(f.name for f in fields(LaneLedSettings) if f.type is bool)


def problems(settings) -> list[str]:
  """Return every reason the settings are unusable; empty means valid."""
  out = []
  if not isinstance(settings, LaneLedSettings):
    return ["not LaneLedSettings"]
  for name in _NUMERIC:
    value = getattr(settings, name)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
      out.append(f"{name}: not finite")
      continue
    low, high = RANGES[name]
    if not low <= value <= high:
      out.append(f"{name}: {value} outside {low}..{high}")
  for name in _BOOLEAN:
    if not isinstance(getattr(settings, name), bool):
      out.append(f"{name}: not boolean")
  if not out:
    if settings.drop_confidence >= settings.adopt_confidence:
      out.append("drop_confidence must be below adopt_confidence")
    if settings.min_width >= settings.max_width:
      out.append("min_width must be below max_width")
  return out


def valid(settings) -> bool:
  return not problems(settings)


class SettingsSlew:
  """Effective settings that approach the saved settings linearly over slew_time."""

  def __init__(self, initial: LaneLedSettings):
    self.effective = initial
    self.target = initial
    self._rates: dict[str, float] = {}

  def update(self, target: LaneLedSettings, dt: float) -> LaneLedSettings:
    if not valid(target):
      target = self.target  # keep slewing toward the last valid target
    if target is not self.target and target != self.target:
      span = max(self.effective.slew_time, 1e-3)
      self._rates = {name: abs(getattr(target, name) - getattr(self.effective, name)) / span for name in _NUMERIC}
      self.target = target
    changes = {}
    for name in _NUMERIC:
      current, goal = getattr(self.effective, name), getattr(self.target, name)
      if current == goal:
        continue
      step = self._rates.get(name, abs(goal - current)) * max(dt, 0.0)
      changes[name] = goal if abs(goal - current) <= step else current + math.copysign(step, goal - current)
    for name in _BOOLEAN:
      if getattr(self.effective, name) != getattr(self.target, name):
        changes[name] = getattr(self.target, name)
    if changes:
      self.effective = replace(self.effective, **changes)
    return self.effective

  def settled(self) -> bool:
    return self.effective == self.target
