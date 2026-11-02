"""Bounded curve-only braking, composed before the native lead/model minimum."""
import math

CURVE_BRAKE_MAX = 1.5
CURVE_ONSET_JERK = 1.0


def required_deceleration(ceiling, *, selected_speed, ego_speed):
  # The caller must first validate exact model-frame and driver/axis ownership.
  # Map ceilings have no visual apex metadata and keep their existing response.
  if ceiling is None:
    return None
  values = (ceiling.speed_mps, ceiling.apex_speed_mps, ceiling.binding_distance_m, selected_speed, ego_speed)
  if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
    return None
  target, apex, distance, selected, ego = values
  if not 0 < apex <= target < ego <= 100 or not max(20., ego) <= distance <= 1000 or abs(selected-target) > 1e-6:
    return None
  # Escalation is approach-only: inside the bend, ordinary tracking resumes.
  return min(CURVE_BRAKE_MAX, max(0., (ego**2-apex**2)/(2*distance)))


def compose_braking(native_accel, required, previous, dt):
  if (required is None or type(required) not in (int, float) or not math.isfinite(required) or
      not 0 < required <= CURVE_BRAKE_MAX or not math.isfinite(previous) or not 0 < dt <= .2):
    return native_accel
  # Never weaken existing braking. The added request follows the previous
  # selected acceleration; native positive jerk handles its eventual release.
  return min(native_accel, max(-required, previous-CURVE_ONSET_JERK*dt))


CURVE_RELEASE_JERK = 1.0
CURVE_ACCEL_JERK = 0.6
CURVE_ACCEL_START_JERK = 0.2
CURVE_ACCEL_EASE_END = 0.25


class CurveResponse:
  """Ease an actual visual-curve brake request back into ordinary cruise."""
  def __init__(self):
    self.recovering = False

  def step(self, target, previous, dt, native_jerk, *, active, enabled, goal=None, owns_request=True):
    goal = target if goal is None else goal
    if (not enabled or any(type(v) not in (int, float) or not math.isfinite(v)
                           for v in (target, previous, dt, native_jerk, goal)) or
        not 0 < dt <= .2 or native_jerk <= 0):
      self.recovering = False
      return target
    # Anchor a winning curve's recovery to the actual prior request, including
    # braking just released by another owner. Hidden positive cruise state must
    # not jump past that brake release. A losing curve cannot acquire recovery.
    if active and owns_request and min(target, previous) < 0:
      self.recovering = True
    if not self.recovering:
      return target
    # Separate brake release from positive acceleration: applying the slower
    # positive rate below zero would unnecessarily carry braking past the exit.
    release = min(CURVE_RELEASE_JERK, native_jerk)
    # Roll into acceleration at its actual current value, not elapsed time:
    # a zero hold or a stronger owner cannot pre-charge a sharp restart.
    fraction = min(1., max(0., previous) / CURVE_ACCEL_EASE_END)
    ease = fraction * fraction * (3. - 2. * fraction)
    positive = min(CURVE_ACCEL_START_JERK + (CURVE_ACCEL_JERK - CURVE_ACCEL_START_JERK) * ease, native_jerk)
    below_zero = min(dt, max(0., -previous) / release)
    upper = previous + release * below_zero + positive * (dt - below_zero)
    result = min(target, upper)
    # No timer or coast dwell: hand back once ordinary control joins the ramp.
    # Stronger braking passes immediately and also ends an inactive tail.
    if not active and goal <= upper:
      self.recovering = False
    return result
