"""SP-011: gentle Tesla acceleration during a selected-speed change."""
import math

UP_JERK = 0.2  # m/s^3


DOWN_JERK = 0.4  # Cruise slowing only; MPC/model/hazard braking keeps priority.


class SetSpeedComfort:
  def __init__(self):
    self.previous = None
    self.profile_owned = False
    self.active = False

  def update(self, cp, *, selected, speed, enabled, profile_owned=False):
    valid = (enabled and cp.carFingerprint in ('TESLA_MODEL_3', 'TESLA_MODEL_Y') and
             cp.openpilotLongitudinalControl and all(math.isfinite(v) for v in (selected, speed)) and
             0 < selected < 80 and speed > 1.)
    if not valid:
      self.previous, self.active, self.profile_owned = None, False, False
      return False
    previous, self.previous = self.previous, selected
    if previous is None:
      self.active = False
    elif abs(selected-previous) > .05:
      self.active = True
    if previous is None or abs(selected-previous) > .05 or not self.active:
      self.profile_owned = False
    if self.active and profile_owned:
      self.profile_owned = True
    return self.active


def braking_lower_bound(previous, dt, native_jerk, braking_jerk):
  """Release positive acceleration normally, then ramp into cruise braking."""
  release_time = min(dt, max(previous, 0.) / native_jerk)
  return previous - native_jerk * release_time - min(native_jerk, braking_jerk) * (dt-release_time)


def owns_selected_target(request, now_ns, selected):
  from openpilot.starpilot.longitudinal.slc_coast import EasingRequest
  return (isinstance(request, EasingRequest) and type(request.now_ns) is int and
          0 <= now_ns-request.now_ns <= 100_000_000 and
          math.isfinite(request.target) and math.isfinite(selected) and
          abs(request.target-selected) <= .01)


class CruiseTransition:
  """Shape the cruise-speed response before lead/model arbitration.

  State follows the previous final request, so a hidden cruise trajectory cannot
  accumulate behind stronger braking. Repeated target changes never reset it.
  Rates limit transitions, not the selected acceleration/braking profile.
  """
  UP_RATE = .35
  DOWN_RATE = .4
  RATE_CHANGE = .7

  def __init__(self):
    self.reset()

  def reset(self):
    self.time_ns = None
    self.output = None
    self.rate = 0.
    self.recovering = False

  def shape(self, *, selected, speed, desired, previous, now_ns, enabled,
            protected=False, response_time=.5, release_rate=1.):
    valid = (enabled and type(now_ns) is int and now_ns > 0 and
             all(math.isfinite(v) for v in (selected, speed, desired, previous, response_time, release_rate)) and
             0. < selected < 80. and speed > 1. and 0. <= response_time <= 2. and 0. < release_rate <= 3.)
    if not valid or protected:
      self.reset()
      self.recovering = True
      return desired
    dt = (now_ns-self.time_ns)/1e9 if self.time_ns is not None else .05
    if self.time_ns is None or not .01 <= dt <= .15:
      dt, self.rate = .05, 0.
    elif self.output is not None and abs(previous-self.output) > .02:
      # Lower final model/lead/stop output is authoritative. No stale ramp on
      # its release; restart from that actual request without storing a deficit.
      self.rate = 0.
      self.recovering |= previous < self.output-.02
    self.time_ns = now_ns
    if self.recovering and previous < 0.:
      # This braking came from another authority (or re-engagement), not an
      # ordinary cruise target. Retain the native release rate to avoid holding
      # the brakes after a lead/stop/model constraint has cleared. Stronger
      # requested braking is still immediate; positive build-up restarts at zero.
      release_time = min(dt, -previous/release_rate)
      remaining = dt-release_time
      self.rate = min(self.UP_RATE, self.RATE_CHANGE*remaining)
      ceiling = previous + release_rate*release_time + .5*self.rate*remaining
      self.output = min(desired, ceiling)
      if self.output >= 0.:
        self.recovering = False
      return self.output
    self.recovering = False
    error = selected-speed
    target = desired
    if target*error > 0.:
      # Ease out before reaching the speed, allowing for the actuator delay and
      # the time needed to turn the acceleration trend around. Symmetric on
      # slowing: don't keep braking hard until the target has been crossed.
      j = self.DOWN_RATE if target > 0. else self.UP_RATE
      horizon = response_time + 1.25
      arrival = max(0., -j*horizon + math.sqrt((j*horizon)**2+2*j*abs(error)))
      target = math.copysign(min(abs(target), arrival), target)
    goal = min(max((target-previous)/.5, -self.DOWN_RATE), self.UP_RATE)
    # Do not keep increasing acceleration after the target asks for less (or
    # keep deepening braking after it asks for release). Preserve acceleration,
    # restart only the opposing trend at zero, then build the new rate gently.
    if goal*self.rate <= 0.:
      self.rate = 0.
    rate = min(max(goal, self.rate-self.RATE_CHANGE*dt), self.rate+self.RATE_CHANGE*dt)
    result = previous + (self.rate+rate)*.5*dt
    # Avoid numerical overshoot of an acceleration target. This small final
    # convergence is not a promise of a globally bounded acceleration derivative.
    if (target-previous)*(target-result) < 0.:
      result, rate = target, 0.
    self.output, self.rate = result, rate
    return result
