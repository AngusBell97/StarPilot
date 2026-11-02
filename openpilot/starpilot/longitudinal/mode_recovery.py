"""Bounded upward acceleration recovery across fresh driving-mode transitions."""

from dataclasses import dataclass
import math

MAX_AGE_NS = 150_000_000
MAX_GAP_NS = 150_000_000
BRAKE_RELEASE_JERK = 4.0
SOURCES = ('carState', 'carControl', 'controlsState', 'selfdriveState', 'radarState', 'modelV2')


@dataclass(frozen=True)
class RecoveryFrame:
  now_ns: int
  model_ns: int
  drive_id: int
  experimental: bool


def project(sm, cp, now_ns: int, drive_id: int) -> RecoveryFrame | None:
  try:
    if type(now_ns) is not int or type(drive_id) is not int or not 0 < drive_id < now_ns:
      return None
    if not cp.openpilotLongitudinalControl or any(getattr(cp, flag, False) for flag in ('passive', 'dashcamOnly', 'notCar')):
      return None
    for name in SOURCES:
      stamp = sm.logMonoTime[name]
      if (sm.valid[name] is not True or sm.alive[name] is not True or type(stamp) is not int or
          not drive_id <= stamp <= now_ns or now_ns - stamp > MAX_AGE_NS):
        return None
    car = sm['carState']
    if (not sm['carControl'].longActive or not sm['carControl'].enabled or not sm['selfdriveState'].enabled or
        not car.canValid or car.canTimeout or car.gasPressed or car.brakePressed or car.standstill or
        sm['controlsState'].longControlState == 0 or
        not math.isfinite(car.vEgo) or not 2.0 < car.vEgo <= 100.0):
      return None
    return RecoveryFrame(now_ns, sm.logMonoTime['modelV2'], drive_id, bool(sm['selfdriveState'].experimentalMode))
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return None


class ModeRecovery:
  def __init__(self):
    self.reset()

  def reset(self):
    self.previous: RecoveryFrame | None = None
    self.armed = False
    self.applied = False
    self.reason = 'inactive'
    self.limit: float | None = None

  def step(self, frame: RecoveryFrame | None, *, previous_target: float, target: float,
           positive_jerk: float, blocked: bool = False, protective: bool = False) -> float:
    self.applied = False
    self.limit = None
    if (frame is None or blocked or not all(math.isfinite(v) for v in (previous_target, target, positive_jerk)) or
        not 0 < positive_jerk <= 10 or type(frame.experimental) is not bool or
        any(type(v) is not int for v in (frame.now_ns, frame.model_ns, frame.drive_id)) or
        not 0 < frame.drive_id <= frame.model_ns <= frame.now_ns or frame.now_ns - frame.model_ns > MAX_AGE_NS):
      self.reset()
      self.reason = 'blocked'
      return target
    old = self.previous
    if old is None or old.drive_id != frame.drive_id:
      self.reset()
      self.previous = frame
      self.reason = 'initialising'
      return target
    if not (0 < frame.now_ns - old.now_ns <= MAX_GAP_NS and 0 < frame.model_ns - old.model_ns <= MAX_GAP_NS):
      self.reset()
      self.reason = 'discontinuous'
      return target
    self.previous = frame
    if old.experimental != frame.experimental:
      self.armed = True
    if not self.armed:
      self.reason = 'inactive'
      return target
    # Re-entry preserves an outstanding upward recovery. Stronger model
    # braking still passes immediately through the comparison below.
    # Use only time actually advanced by both fresh clocks. A duplicate model
    # cannot spend another frame's recovery budget.
    dt = min(frame.now_ns - old.now_ns, frame.model_ns - old.model_ns) / 1e9
    negative_time = min(dt, max(0.0, -previous_target) / BRAKE_RELEASE_JERK)
    limit = previous_target + BRAKE_RELEASE_JERK * negative_time + positive_jerk * (dt - negative_time)
    self.limit = limit
    if target <= limit:
      self.armed = bool(protective)
      self.reason = 'protective' if protective else 'converged'
      return target
    self.applied = True
    self.reason = 'recovering'
    return limit
