"""Map approach identity and accepted numerical-limit generations."""
import math
import uuid


def same_limit(a, b):
  return abs(float(a) - float(b)) < 0.1


MAP_MAX_AGE = 2.0
MAP_FRESH_AGE = 0.5
MAP_RECOVERY_TIME = 1.0
MAP_RECOVERY_SAMPLES = 3


class MapInputHealth:
  """Bounded receipt lease; repeated publication IDs cannot renew it."""
  def __init__(self):
    self.stamp = None
    self.received_at = None
    self.last_now = None
    self.healthy = False
    self.acquired = False
    self.recovery = None
    self.age = math.inf

  def _lost(self):
    self.healthy = False
    self.recovery = None
    return False

  def update(self, now, stamp, received_at, *, valid, confirmed, key, nonincreasing=False):
    if not all(math.isfinite(v) for v in (now, stamp, received_at)) or stamp <= 0 or received_at <= 0:
      return self._lost()
    backwards = self.last_now is not None and now < self.last_now
    self.last_now = now
    if backwards or received_at > now or (self.stamp is not None and stamp < self.stamp):
      return self._lost()
    new = stamp != self.stamp
    if new:
      if self.received_at is not None and received_at < self.received_at:
        return self._lost()
      self.stamp, self.received_at = stamp, received_at
    self.age = now - self.received_at
    if not valid or not 0 <= self.age <= MAP_MAX_AGE:
      return self._lost()
    if self.healthy:
      return True
    # Startup may use a fresh sample immediately. A previously lost source
    # must prove recovery with distinct confirmed samples, not planner ticks.
    if not new or self.age > MAP_FRESH_AGE:
      return False
    if not self.acquired:
      self.healthy = self.acquired = True
      return True
    if not confirmed:
      self.recovery = None
      return False
    # Do not wait on a newly confirmed limit that cannot raise the current
    # target. The caller includes offsets and any immediate higher preview.
    if nonincreasing:
      self.healthy = True
      self.recovery = None
      return True
    old = self.recovery
    if old is None or key != old[0] or received_at - old[2] > MAP_RECOVERY_TIME:
      self.recovery = (key, received_at, received_at, 1)
    else:
      self.recovery = (key, old[1], received_at, old[3] + 1)
    _, start, end, count = self.recovery
    self.healthy = count >= MAP_RECOVERY_SAMPLES and end - start >= MAP_RECOVERY_TIME
    return self.healthy


def map_distance_jump(previous, distance, speed):
  return previous is not None and distance > previous + max(20.0, 2.0 * speed)


class LowerLimitDistance:
  """Advance a lower preview between receipts without inventing a boundary."""
  def __init__(self):
    self.key = self.stamp = self.raw = self.remaining = self.last_now = None

  def update(self, current, upcoming, distance, road, speed, now, stamp):
    key = (current, upcoming, road)
    new = stamp != self.stamp
    reset = (key != self.key or self.last_now is None or now < self.last_now or
             (new and map_distance_jump(self.raw, distance, speed)))
    if reset:
      self.remaining = distance
    else:
      travel = max(0.0, min(speed, 70.0)) * min(max(0.0, now - self.last_now), MAP_MAX_AGE)
      self.remaining = min(max(0.0, self.remaining - travel), distance)
    # Wheel/along-way speed disagreement must not accumulate over kilometres
    # of preview. Fresh positions may correct upward within this bounded lead.
    self.remaining = max(self.remaining, distance - max(0.0, min(speed, 70.0)) * MAP_MAX_AGE)
    self.key, self.stamp, self.last_now = key, stamp, now
    if new or reset:
      self.raw = distance
    # Only a confirmed current-limit update can commit the zone or end its pin.
    return max(0.001, self.remaining)


class MapPreviewContinuity:
  """Brief same-boundary confidence dips cannot undo an accepted preview."""
  def __init__(self):
    self.confirmed = None
    self.sample = None
    self.crossing = False

  def update(self, current, upcoming, distance, road, speed, *, now, sample_time, confirmed, weak, healthy, active,
             allow_higher_hold=False):
    self.sample = (current, upcoming, distance)
    finite = all(math.isfinite(v) for v in (current, upcoming, distance, speed, now, sample_time))
    old = self.confirmed
    if healthy and finite and old is not None and road == old[3]:
      age, sample_age = now - old[4], sample_time - old[4]
      same_pair = same_limit(current, old[0]) and same_limit(upcoming, old[1])
      near = old[2] <= max(20.0, 2.0 * speed)
      progress = max(20.0, speed * sample_age + 10.0)
      crossing = (same_pair and (distance <= 0 or self.crossing) or
                  weak and same_limit(current, old[1]))
      permitted = active or (allow_higher_hold and old[1] > old[0])
      if (permitted and (confirmed or weak) and near and crossing and
          0 <= age <= 1 and 0 <= sample_age <= age and 0 <= speed <= 70 and
          -progress <= distance <= old[2] + max(20.0, 2.0 * speed)):
        # A tiny negative distance is the handover of an already known pair,
        # not permission to select the old dashboard limit. Do not renew the
        # anchor: repeated or jittering boundary samples have the same expiry.
        self.crossing = True
        self.sample = ((old[0], old[1], max(.001, old[2] - speed * sample_age)) if active
                       else (old[0], 0.0, 0.0))
        return True
      if self.crossing and same_pair and near and distance <= old[2] + max(20.0, 2.0 * speed):
        # Keep the expired anchor until the pair changes or moves away. A
        # positive jitter frame must not restart the same boundary lease.
        return False
    if confirmed and healthy:
      self.confirmed = ((current, upcoming, distance, road, sample_time)
                        if finite and current >= 1 and upcoming >= 1 and distance >= 0 and 0 <= now - sample_time <= 1 else None)
      self.crossing = False
      return True
    if healthy and weak and active and finite and old is not None:
      age = now - old[4]
      sample_age = sample_time - old[4]
      if 0 <= age <= 1 and 0 <= sample_age <= age and 0 <= speed <= 70 and road == old[3]:
        if (distance > 0 and same_limit(current, old[0]) and same_limit(upcoming, old[1]) and
            old[2] - max(20.0, speed * sample_age + 10.0) <= distance <= old[2] + max(20.0, 2.0 * speed)):
          return True
        if (distance >= -max(20.0, speed * sample_age + 10.0) and same_limit(current, old[1]) and
            old[2] <= max(20.0, 2.0 * speed)):
          # A weak boundary reading may retain the previously confirmed preview,
          # but cannot commit its new current/next values or introduce a new zone.
          self.sample = (old[0], old[1], max(.001, old[2] - speed * sample_age))
          return True
    self.confirmed = None
    self.crossing = False
    return False


class LimitZone:
  """Continuity of the selected current limit, independent of lookahead/fallback."""
  def __init__(self):
    self.generation = 0
    self.limit = 0.0
    self.valid = False
    self.source = ''

  def observe(self, limit, source, valid):
    valid = bool(valid and math.isfinite(limit) and limit >= 1)
    if valid != self.valid or (valid and not same_limit(limit, self.limit)):
      self.generation += 1
    self.valid = valid
    self.limit = limit if valid else 0.0
    self.source = source if valid else ''

  def snapshot(self, offset):
    return dict(zoneGeneration=self.generation, zoneLimit=self.limit, zoneValid=self.valid,
                zoneSource=self.source, zoneTarget=self.limit + offset if self.valid else 0.0)


class MapApproach:
  def __init__(self):
    self.key = None
    self.identity = ""
    self.serial = 0
    self.active = False
    self.phase = "stable"
    self.distance = None
    self.rolled_back = False

  def update(self, current, upcoming, distance, road, speed, trigger, valid, *, identity_distance=None, allow_activate=True):
    self.rolled_back = False
    valid = valid and all(math.isfinite(v) for v in (current, upcoming, distance, speed, trigger))
    if not valid or current < 1 or distance < 0:
      self.rolled_back = self.active
      if self.active:
        self.phase = "rollback"
      self.active = False
      return 0.0
    if self.active and self.key and same_limit(current, self.key[1]):
      self.active = False
      self.phase = "commit"
      # Publish the reached boundary before starting another approach. The next
      # planner frame may accept the following limit with a new checkpoint;
      # neither its key nor its preview may erase this commit in the same frame.
      return current
    if upcoming < 1 or same_limit(current, upcoming):
      if self.active:
        self.rolled_back = True
        self.phase = "rollback"
      self.active = False
      return current
    key = (current, upcoming)
    raw_distance = distance if identity_distance is None else identity_distance
    jumped = map_distance_jump(self.distance, raw_distance, speed)
    if key != self.key or jumped:
      self.rolled_back = self.active
      self.active = False
      self.key = key
      self.serial += 1
      self.identity = str(self.serial)
      self.phase = "stable"
    self.distance = raw_distance
    if allow_activate and (distance == 0 or (trigger > 0 and distance <= trigger)):
      self.active = True
    if self.active:
      self.phase = "provisional"
      return upcoming
    return current


class LimitTransition:
  def __init__(self):
    self.epoch = uuid.uuid4().hex
    self.generation = 0
    self.current = 0.0
    self.phase = "stable"
    self.transition = ""
    self.adopt_sequence = 0
    self.snapshot = self.observe(0, False)

  def adopt(self):
    self.adopt_sequence += 1

  def observe(self, limit, valid, *, offset=0.0, provisional=False, transition="",
              current_limit=0.0, current_offset=None, commit=False, rollback=False):
    valid = bool(valid and math.isfinite(limit) and math.isfinite(offset) and limit >= 1)
    if rollback:
      self.phase, self.transition = "rollback", transition
    elif valid and provisional:
      self.phase, self.transition = "provisional", transition
      if self.current < 1 and current_limit >= 1:
        self.current = current_limit
        self.generation += 1
    elif valid:
      self.phase, self.transition = 'stable', transition
      if not same_limit(limit, self.current):
        self.current = limit
        self.generation += 1
        self.phase, self.transition = "stable", transition
      if commit:
        self.phase, self.transition = "commit", transition
    self.snapshot = {
      "epoch": self.epoch, "generation": self.generation, "phase": self.phase,
      "transition": self.transition, "currentLimit": self.current,
      "currentTarget": max(0.0, self.current + (offset if current_offset is None else current_offset)),
      "target": max(0.0, limit + offset) if valid else 0.0, "valid": valid,
      "approachLimit": limit if valid and provisional else 0.0,
      "adoptSeq": self.adopt_sequence, "plannedTarget": 0.0,
      "onsetEta": 0.0, "scheduled": False,
    }
    return self.snapshot
