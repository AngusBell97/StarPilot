"""SP-034: optional bar-scale steering-load speed/acceleration ceiling.

Only the extra request is slew-limited. A stronger ordinary braking request always
wins. No actuator, steering-limit, selected-cruise or persistence writes here.
"""
from dataclasses import dataclass
import math

KEY = 'CurveSteeringLoadSlowdown'
THRESHOLD = .95
MIN_SPEED = 5.0
MAX_SPEED = 60.0
MAX_BRAKING = 1.2
DOWN_JERK = 2.4
UP_JERK = .6
GAIN = .6
ACTIVATE_SECONDS = .15
INPUT_MAX_AGE_NS = 100_000_000
GRAVITY = 9.81


def supported(cp):
  return (getattr(cp, 'carFingerprint', '') in ('TESLA_MODEL_3', 'TESLA_MODEL_Y') and
          bool(getattr(cp, 'openpilotLongitudinalControl', False)) and
          not any(getattr(cp, n, False) for n in ('passive', 'dashcamOnly', 'notCar')))


def demand(curvature, roll, speed):
  # Same unfiltered angle-control estimate as ui/torque_feedback.py. Do not
  # clip to the displayed +/-100%: overload must retain its magnitude.
  return curvature * speed * speed - roll * GRAVITY * max(0., min(1., (speed - 5.) / 10.))


@dataclass(frozen=True)
class Observation:
  now_ns: int
  speed: float
  curvature: float
  roll: float
  maximum: float

  @property
  def utilization(self):
    return abs(demand(self.curvature, self.roll, self.speed)) / self.maximum

  def target_speed(self):
    # On the speed-sensitive side of the curve, find the highest speed inside
    # the 95% budget. Road banking changes with speed below 15 m/s in the UI.
    if abs(self.curvature) < 1e-6:
      return MAX_SPEED
    sign = math.copysign(1., self.curvature)
    budget = THRESHOLD * self.maximum
    lo, hi = MIN_SPEED, MAX_SPEED
    if sign * demand(self.curvature, self.roll, lo) >= budget:
      return lo
    if sign * demand(self.curvature, self.roll, hi) <= budget:
      return hi
    for _ in range(28):
      mid = (lo + hi) / 2
      if sign * demand(self.curvature, self.roll, mid) > budget:
        hi = mid
      else:
        lo = mid
    return lo


def project(sm, cp, now_ns):
  """Unavailable authority/telemetry is not a zero-load observation."""
  if not supported(cp) or type(now_ns) is not int or now_ns <= 0:
    return None
  try:
    for name in ('carState', 'carControl', 'controlsState', 'selfdriveState', 'vehicleParameters'):
      stamp = sm.logMonoTime[name]
      if (type(stamp) is not int or not 0 < stamp <= now_ns or now_ns - stamp > INPUT_MAX_AGE_NS or
          not sm.valid[name] or not sm.alive[name]):
        return None
    car, control, controls = sm['carState'], sm['carControl'], sm['controlsState']
    params = sm['vehicleParameters']
    if (not car.canValid or car.canTimeout or not control.latActive or not control.longActive or
        not sm['selfdriveState'].enabled or controls.forceDecel or not params.valid or
        car.gasPressed or car.brakePressed or car.steeringPressed or car.standstill or
        controls.lateralControlState.which() != 'angleState'):
      return None
    values = (float(car.vEgo), float(controls.desiredCurvature), float(params.roll), float(cp.maxLateralAccel))
    if not all(math.isfinite(x) for x in values):
      return None
    speed, curvature, roll, maximum = values
    if not MIN_SPEED <= speed <= MAX_SPEED or abs(curvature) > .1 or abs(roll) > .2 or not 1 <= maximum <= 5:
      return None
    # A bank-only/reversed demand is not a request to cure steering load by
    # slowing. The active side must become less loaded when speed decreases.
    if curvature * demand(curvature, roll, speed) < 0:
      return None
    return Observation(now_ns, *values)
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError, ZeroDivisionError):
    return None


class SteeringLoad:
  def __init__(self):
    self.reset()
    self.drive_id = None
    self.quiet_until_ns = 0
    self.clock_ns = 0
    self.requested = False
    self.last_diagnostic_ns = 0
    self.diagnostic_active = False
    self.ordinary_accel = None

  def reset(self):
    self.last_ns = 0
    self.observation = None
    self.dt = 0.
    self.above_seconds = 0.
    self.cap = None
    self.target = None
    self.enabled = False
    self.applied = False
    self.reason = 'off'

  @property
  def exclude_learning(self):
    return (self.cap is not None or self.above_seconds > 0 or
            self.clock_ns < self.quiet_until_ns)

  def prepare(self, sm, cp, now_ns, *, enabled, drive_id=0):
    self.clock_ns = now_ns
    self.requested = bool(enabled)
    if self.drive_id != drive_id:
      self.reset()
      self.quiet_until_ns = 0
      self.drive_id = drive_id
    observation = project(sm, cp, now_ns) if enabled or self.cap is not None else None
    old_ns = self.last_ns
    if observation is None or old_ns and not 0 < now_ns - old_ns <= 200_000_000:
      was_active = self.cap is not None
      self.reset()
      if was_active:
        self.quiet_until_ns = max(self.quiet_until_ns, now_ns + 5_000_000_000)
      self.reason = 'unavailable' if enabled else 'off'
      return
    self.dt = (now_ns - old_ns) / 1e9 if old_ns else 0.
    self.last_ns = now_ns
    self.observation = observation
    self.enabled = bool(enabled)
    self.above_seconds = self.above_seconds + self.dt if enabled and observation.utilization > THRESHOLD else 0.
    self.target = observation.target_speed() if enabled else None
    self.reason = 'holding' if self.cap is not None else 'qualifying' if self.above_seconds else 'below_threshold'

  def apply(self, ordinary_accel, previous_accel):
    self.applied = False
    self.ordinary_accel = ordinary_accel if math.isfinite(ordinary_accel) else None
    if self.observation is None or not all(math.isfinite(v) for v in (ordinary_accel, previous_accel)):
      self.cap = None
      return ordinary_accel
    obs = self.observation
    if self.cap is None:
      if self.above_seconds + 1e-9 < ACTIVATE_SECONDS:
        return ordinary_accel
      self.cap = max(-MAX_BRAKING, previous_accel)
    desired = (max(-MAX_BRAKING, GAIN * (self.target - obs.speed))
               if self.enabled and self.target is not None else ordinary_accel)
    # Do not capture braking stronger than our limit; if ordinary braking was stronger at entry,
    # preserve it through the minimum rather than copying it into our cap.
    desired = max(-MAX_BRAKING, desired)
    self.cap = max(self.cap - DOWN_JERK * self.dt, min(desired, self.cap + UP_JERK * self.dt))
    output = min(ordinary_accel, self.cap)
    self.applied = output < ordinary_accel - 1e-9
    if self.applied:
      self.reason = 'limiting' if desired <= self.cap else 'releasing'
      self.quiet_until_ns = obs.now_ns + 5_000_000_000
    elif not self.enabled or obs.utilization < THRESHOLD and desired >= ordinary_accel:
      self.cap = None
      self.reason = 'released'
    return output

  def diagnostic(self):
    """At most 1 Hz while configured/releasing, plus the final off sample."""
    active = self.requested or self.cap is not None
    if not active and not self.diagnostic_active:
      return None
    if 0 <= self.clock_ns - self.last_diagnostic_ns < 1_000_000_000:
      return None
    self.last_diagnostic_ns = self.clock_ns
    self.diagnostic_active = active
    obs = self.observation
    return dict(version=1, now_ns=self.clock_ns, configured=self.requested,
                reason=self.reason, applied=self.applied, utilization=obs.utilization if obs else None,
                target_mps=self.target, acceleration_ceiling=self.cap, ordinary_accel=self.ordinary_accel)
