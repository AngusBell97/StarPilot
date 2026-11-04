"""Curve envelope and bounded target dynamics, without actuator authority."""

from dataclasses import dataclass
import math

from openpilot.starpilot.curve_speed.learning import LearnedCurve, MAX_CURVATURE, finite_number
from openpilot.starpilot.model_geometry import normalized_origin


MIN_SPEED = 25 * 0.44704
APPROACH_DECEL = 0.3
# 3 Nov 2026 review: the model's predicted path curvature jitters frame to frame, and with a 0.4 s filter and
# 3.0/2.5 m/s per second rate limits the ceiling swung 5 to 7 mph within a bend, reversing the acceleration request
# several times. The target now follows the envelope more slowly, slower still when easing, and may not rise above
# the car's own speed for RISE_HOLD_SECONDS after it has fallen.
TARGET_UP_RATE = 1.0
TARGET_DOWN_RATE = 1.5
FILTER_RC = 1.2
FILTER_RC_DOWN = 0.8
RISE_HOLD_SECONDS = 2.0
# 4 Nov 2026: the learned comfort gave a 32 mph apex in a 40 mph S-bend (about 2.0 m/s2 lateral) and the driver
# wanted 3 to 5 mph more. The envelope uses the learned comfort scaled by this factor (1.25 -> apex speed +12 %,
# 32 -> 36 mph); learning, nudges and the 4.0 m/s2 physical cap are unchanged.
COMFORT_SCALE = 1.25
EGO_HEADROOM = 2.0
RELEASE_DEBOUNCE = 0.25
MAX_LATERAL_ACCEL = 4.0
MAX_PROFILE_POINTS = 129
MAX_STEP_SECONDS = 0.2
OPENING_GAIN = 0.15
OPENING_NOISE = 0.01


@dataclass(frozen=True)
class CurveProfile:
  curvatures: tuple[float, ...]
  distances: tuple[float, ...]
  observed_ns: int

  def __post_init__(self):
    if (type(self.curvatures) is not tuple or type(self.distances) is not tuple or
        not 0 < len(self.curvatures) == len(self.distances) <= MAX_PROFILE_POINTS or
        type(self.observed_ns) is not int or self.observed_ns < 0):
      raise ValueError("invalid profile shape or timestamp")
    if (any(not finite_number(k) for k in self.curvatures) or
        any(not finite_number(d) or d < 0 for d in self.distances) or
        any(b < a for a, b in zip(self.distances, self.distances[1:], strict=False))):
      raise ValueError("invalid curvature or distance")

  @classmethod
  def from_model(cls, model, observed_ns: int) -> 'CurveProfile | None':
    try:
      rates = tuple(model.orientationRate.z)
      velocities = tuple(model.velocity.x)
      distances = tuple(model.position.x)
      if (not len(rates) == len(velocities) == len(distances) or
          any(not finite_number(v) or v < 0 for v in velocities) or any(not finite_number(rate) for rate in rates)):
        return None
      # The predicted origin can round slightly below zero (observed around
      # -7e-11 meters). Accept only sub-micrometer origin roundoff; the typed
      # profile still rejects negative future points and backwards geometry.
      distances = normalized_origin(distances)
      curvatures = tuple(min(abs(rate) / v, 0.1) if v >= 3.0 else 0.0 for rate, v in zip(rates, velocities, strict=True))
      return cls(curvatures, distances, observed_ns)
    except (AttributeError, TypeError, ValueError, OverflowError):
      return None


@dataclass(frozen=True)
class Envelope:
  speed_mps: float
  binding_distance_m: float
  apex_speed_mps: float | None = None


def evaluate(profile: CurveProfile, comfort: LearnedCurve, cruise_mps: float, *, weather_reduction: float = 0.0) -> Envelope:
  """Approach envelope only. The caller must qualify freshness and ownership."""
  if not finite_number(cruise_mps) or cruise_mps <= 0 or not finite_number(weather_reduction) or not 0 <= weather_reduction <= 1:
    raise ValueError("invalid cruise or weather input")
  result = Envelope(float(cruise_mps), 0.0)
  for curvature, distance in zip(profile.curvatures, profile.distances, strict=True):
    magnitude = abs(curvature)
    if magnitude >= 0.004 and distance >= 30.0:
      magnitude *= 1.23
    magnitude = min(magnitude, MAX_CURVATURE)
    denominator = max(magnitude, 1e-4)
    lateral = comfort.comfort(magnitude) * COMFORT_SCALE * (1 - weather_reduction)
    apex_speed = min(max(math.sqrt(lateral / denominator), MIN_SPEED), math.sqrt(MAX_LATERAL_ACCEL / denominator))
    allowed = math.sqrt(apex_speed**2 + (2 * APPROACH_DECEL) * distance)
    if allowed < result.speed_mps:
      result = Envelope(allowed, float(distance), apex_speed)
  return result


class TargetFilter:
  """Reset after a frame gap; never integrate a pause as control progress."""

  def __init__(self):
    self.target: float | None = None
    self._filtered = 0.0
    self._release_seconds = 0.0
    self._opening_samples: list[float] = []
    self._opening_active = False
    self._nominal_target: float | None = None
    self._since_fall = math.inf

  def clear_opening(self) -> None:
    self._opening_samples.clear()
    self._opening_active = False

  def reset(self) -> None:
    self.target = None
    self._filtered = 0.0
    self._release_seconds = 0.0
    self.clear_opening()
    self._nominal_target = None
    self._since_fall = math.inf

  def step(self, envelope: Envelope, *, ego_mps: float, cruise_mps: float, dt: float) -> float:
    if (not finite_number(ego_mps) or ego_mps < 0 or not finite_number(cruise_mps) or cruise_mps <= 0 or
        not finite_number(dt) or not 0 < dt <= MAX_STEP_SECONDS or
        not finite_number(envelope.speed_mps) or envelope.speed_mps <= 0 or envelope.speed_mps > cruise_mps):
      self.reset()
      raise ValueError("invalid target input or discontinuous time")
    # An early release is a temporary output adjustment, not new slowing
    # history. Restore the unmodified filter trajectory before the next step.
    if self.target is not None and self._nominal_target is not None:
      self.target = min(self.target, self._nominal_target)
    raw = envelope.speed_mps
    if self._opening_samples and raw < self._opening_samples[-1] - OPENING_NOISE:
      self._opening_active = False
    self._opening_samples = (self._opening_samples + [raw])[-3:]
    opening = (len(self._opening_samples) == 3 and
               all(b > a + OPENING_NOISE for a, b in zip(self._opening_samples, self._opening_samples[1:])) and
               raw - self._opening_samples[0] >= OPENING_GAIN)
    if self.target is None:
      self.target = min(cruise_mps, max(raw, ego_mps + EGO_HEADROOM))
      self._filtered = self.target
      self._since_fall = math.inf
    previous_target = self.target
    self._release_seconds = self._release_seconds + dt if raw >= ego_mps else 0.0
    alpha = dt / ((FILTER_RC_DOWN if raw < self._filtered else FILTER_RC) + dt)
    self._filtered = (1 - alpha) * self._filtered + alpha * raw
    desired = max(self._filtered, min(raw, ego_mps + EGO_HEADROOM))
    self.target = min(max(desired, self.target - TARGET_DOWN_RATE * dt), self.target + TARGET_UP_RATE * dt)
    if self._release_seconds >= RELEASE_DEBOUNCE:
      self.target = max(self.target, min(raw, ego_mps))
    # Three fresh improving envelopes may release filter lag immediately.
    # The complete current profile still bounds speed; the planner separately
    # shapes acceleration, so this is not a step in commanded acceleration.
    self._opening_active = self._opening_active or opening
    self._nominal_target = min(self.target, cruise_mps)
    if self._opening_active:
      self.target = max(self.target, min(raw, ego_mps + EGO_HEADROOM))
    # Rise hold: once the target has fallen it may rise back up to the car's current speed at once (never hold the
    # ceiling below the speed the envelope allows), but not above it until RISE_HOLD_SECONDS have passed without a
    # further fall. This stops one bend producing brake, accelerate, brake again as the prediction wobbles.
    if self.target < previous_target - OPENING_NOISE:
      self._since_fall = 0.0
    else:
      self._since_fall += dt
      if self._since_fall < RISE_HOLD_SECONDS:
        self.target = min(self.target, max(previous_target, min(raw, ego_mps)))
    # Driver lowering cruise is an immediate cap, independent of filter history.
    self.target = min(self.target, cruise_mps)
    return self.target
