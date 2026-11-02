"""Chill longitudinal mode for the duration of a close-gap lane change.

When the close-gap lane-change preference is on, the planner treats a signalled or moving lane change as Chill:
the end-to-end model is not a longitudinal candidate and the MPC runs in its following ('acc') mode, so the
model's braking for the car in the lane being left cannot veto the early acceleration. Experimental returns
once the manoeuvre is over, after a short hold. Only the upward direction is rate-limited while the window is
open; every brake request still passes straight through.
"""

import math

HOLD_NS = 1_000_000_000          # keep Chill this long after the manoeuvre ends
MAX_WINDOW_NS = 12_000_000_000   # never longer than this from the first qualifying frame (lane change max is 10 s)
MIN_SPEED_MPS = 20 * 0.44704
BRAKE_RELEASE_JERK = 2.0         # m/s^3 for negative requests rising towards zero
MANOEUVRE_STATES = ('preLaneChange', 'laneChangeStarting', 'laneChangeFinishing')
MOVING_STATES = ('laneChangeStarting', 'laneChangeFinishing')


def _finite(v):
  return type(v) in (int, float) and math.isfinite(v)


class LaneChangeChill:
  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.reason = 'idle'
    self.started_ns = None
    self.last_qualified_ns = None
    self.limit = None
    self.limited = False

  def step(self, policy, frame, *, long_active, now_ns):
    """Decide whether this cycle runs as Chill. `frame` is the lane-gap GapFrame (or None)."""
    if (policy is None or not getattr(policy, 'enabled', False) or not getattr(policy, 'close_gap', False) or
        long_active is not True or type(now_ns) is not int or now_ns <= 0):
      self.reset()
      self.reason = 'disabled'
      return False
    qualified = False
    if frame is not None:
      try:
        state = str(frame.lane_state)
        signal = str(frame.signal_direction)
        qualified = (not frame.standstill and not frame.gas_pressed and not frame.brake_pressed and
                     _finite(frame.speed_mps) and frame.speed_mps >= MIN_SPEED_MPS and
                     state in MANOEUVRE_STATES and (signal in ('left', 'right') or state in MOVING_STATES))
      except (AttributeError, TypeError, ValueError):
        qualified = False
      if frame is not None and (getattr(frame, 'gas_pressed', False) or getattr(frame, 'brake_pressed', False)):
        # The driver took over the speed: hand the real mode back at once.
        self.reset()
        self.reason = 'driver_override'
        return False
    if qualified:
      if self.started_ns is None or now_ns < self.started_ns:
        self.started_ns = now_ns
      self.last_qualified_ns = now_ns
    if self.started_ns is not None and now_ns - self.started_ns > MAX_WINDOW_NS:
      self.reset()
      self.reason = 'window_timeout'
      return False
    if self.last_qualified_ns is None or now_ns < self.last_qualified_ns:
      self.reset()
      self.reason = 'idle'
      return False
    held = now_ns - self.last_qualified_ns <= HOLD_NS
    self.active = bool(qualified or held)
    if not self.active:
      self.reset()
      self.reason = 'idle'
      return False
    self.reason = 'manoeuvre' if qualified else 'hold'
    return True

  def shape(self, target, *, previous, dt, positive_jerk):
    """Upward rate limit while the window is open: release braking at 2 m/s^3, then normal cruise jerk."""
    self.limit = None
    self.limited = False
    if (not self.active or not all(_finite(v) for v in (target, previous, dt, positive_jerk)) or
        dt <= 0 or dt > 0.5 or not 0 < positive_jerk <= 10):
      return target
    ramp = previous
    remaining = dt
    if ramp < 0.:
      release = min(remaining, -ramp / BRAKE_RELEASE_JERK)
      ramp = min(0., ramp + BRAKE_RELEASE_JERK * release)
      remaining -= release
    ramp += positive_jerk * remaining
    self.limit = ramp
    if target > ramp:
      self.limited = True
      return ramp
    return target

  def diagnostic(self):
    return dict(chill=self.active, chill_reason=self.reason, chill_limit=self.limit, chill_limited=self.limited)
