"""Bounded signal speed hold and early normal acceleration; all leads remain observed."""

import math
from dataclasses import dataclass, replace

from openpilot.starpilot.lateral.adjacent_lane_evidence import adjacent_lane_available
from openpilot.starpilot.lateral.lane_change_preferences import LaneChangePolicy, read_saved

MAX_SOURCE_AGE_NS = 250_000_000
MAX_MODEL_GAP_NS = 150_000_000
SYNC_GRACE_NS = 300_000_000
LEAD_BRAKE_ABORT_MPS2 = 0.8
MIN_PREP_SPEED_MPS = 20 * 0.44704
PREPARE_SECONDS = 4.0
SETTLE_SECONDS = 0.5
PREDICT_SECONDS = 3.0
RELEASE_ACCEL_PER_SECOND = 0.6
COMFORT_BRAKE_LIMIT = -1.0
BRAKE_RELEASE_JERK = 2.0
MOVING_RESPONSE_SECONDS = 6.0


@dataclass(frozen=True)
class GapLead:
  distance_m: float
  relative_speed_mps: float
  acceleration_mps2: float
  lateral_m: float | None = None


@dataclass(frozen=True)
class GapFrame:
  now_ns: int
  model_ns: int
  car_ns: int
  radar_ns: int
  authority_stamps: tuple[int, ...]
  speed_mps: float
  lane_state: str
  direction: str
  standstill: bool
  left_blindspot: bool
  right_blindspot: bool
  lead_present: bool
  lead_accel_mps2: float
  force_decel: bool
  gas_pressed: bool
  brake_pressed: bool
  model_should_stop: bool
  leads: tuple[GapLead, ...] = ()
  signal_direction: str = 'none'
  adjacent_lane_visible: bool = False


def project(sm, now_ns: int, *, pace: bool = False) -> GapFrame | None:
  try:
    if type(now_ns) is not int or now_ns <= 0:
      return None
    authority = ('selfdriveState', 'longitudinalPlan') if pace else ('carControl', 'controlsState', 'selfdriveState')
    services = ('modelV2', 'carState', 'radarState', *authority)
    for name in services:
      stamp = sm.logMonoTime[name]
      if (sm.valid[name] is not True or sm.alive[name] is not True or
          not getattr(sm, 'freq_ok', {}).get(name, True) or type(stamp) is not int or
          stamp <= 0 or stamp > now_ns or now_ns - stamp > MAX_SOURCE_AGE_NS):
        return None
    car = sm['carState']
    radar = sm['radarState']
    if not car.canValid or car.canTimeout or any(radar.radarErrors.to_dict().values()):
      return None
    leads = tuple(GapLead(float(lead.dRel), float(lead.vRel), float(lead.aLeadK), float(lead.yRel))
                  for lead in (radar.leadOne, radar.leadTwo) if lead.present)
    if any(not all(math.isfinite(v) for v in (lead.distance_m, lead.relative_speed_mps, lead.acceleration_mps2)) or
           not 0 < lead.distance_m <= 500 or abs(lead.relative_speed_mps) > 100 or
           abs(lead.acceleration_mps2) > 20 for lead in leads):
      return None
    direction = 'left' if car.leftBlinker and not car.rightBlinker else (
      'right' if car.rightBlinker and not car.leftBlinker else 'none')
    return GapFrame(now_ns, sm.logMonoTime['modelV2'], sm.logMonoTime['carState'], sm.logMonoTime['radarState'],
                    tuple(sm.logMonoTime[name] for name in authority),
                    float(car.vEgo), str(sm['modelV2'].meta.laneChangeState),
                    str(sm['modelV2'].meta.laneChangeDirection), bool(car.standstill),
                    bool(car.leftBlindspot), bool(car.rightBlindspot), bool(leads),
                    min((lead.acceleration_mps2 for lead in leads), default=0.0),
                    bool(sm['longitudinalPlan'].shouldStop) if pace else bool(sm['controlsState'].forceDecel),
                    bool(car.gasPressed), bool(car.brakePressed), bool(sm['modelV2'].action.shouldStop),
                    leads, direction, visible_adjacent_lane(sm['modelV2'], direction))
  except (AttributeError, IndexError, KeyError, TypeError, ValueError, OverflowError):
    return None


def visible_adjacent_lane(model, direction):
  # Lane geometry is not an observation of destination-lane traffic.
  try:
    indices = (0, 1) if direction == 'left' else (2, 3)
    return (direction in ('left', 'right') and
            all(math.isfinite(model.laneLineProbs[i]) and model.laneLineProbs[i] >= .5 and
                math.isfinite(model.laneLineStds[i]) and 0 <= model.laneLineStds[i] <= .5 for i in indices) and
            adjacent_lane_available(model, -1 if direction == 'left' else 1, 2.5))
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


def qualified(policy, frame, long_active):
  if (policy is None or not policy.enabled or not policy.close_gap or long_active is not True or frame is None or
      not all(type(v) in (int, float) and math.isfinite(v)
              for v in (policy.minimum_speed_mps, policy.close_gap_seconds, frame.speed_mps, frame.lead_accel_mps2)) or
      policy.minimum_speed_mps < 0 or not 0.75 <= policy.close_gap_seconds <= 1.0 or
      frame.speed_mps < max(MIN_PREP_SPEED_MPS, policy.minimum_speed_mps) or
      frame.standstill or frame.force_decel or frame.gas_pressed or frame.brake_pressed or frame.model_should_stop or
      frame.direction not in ('left', 'right') or frame.signal_direction != frame.direction or
      (frame.direction == 'left' and frame.left_blindspot) or (frame.direction == 'right' and frame.right_blindspot) or
      type(frame.now_ns) is not int or
      any(type(stamp) is not int or not 0 < stamp <= frame.now_ns or frame.now_ns - stamp > MAX_SOURCE_AGE_NS
          for stamp in (frame.model_ns, frame.car_ns, frame.radar_ns, *frame.authority_stamps)) or
      frame.lead_present != bool(frame.leads)):
    return False
  return all(all(math.isfinite(v) for v in (lead.distance_m, lead.relative_speed_mps, lead.acceleration_mps2)) and
             0 < lead.distance_m <= 500 and abs(lead.relative_speed_mps) <= 100 and abs(lead.acceleration_mps2) <= 20
             for lead in frame.leads)


def approach_has_room(frame, follow_seconds):
  """Conservative constant-ego forecast, including lead braking and closing reserve.

  This is admission for optional behaviour, not a collision-avoidance replacement.
  Evaluate both reported leads; neither is assumed to be the destination lead.
  """
  for lead in frame.leads:
    closing = max(0.0, -lead.relative_speed_mps)
    distance = (lead.distance_m - closing * PREDICT_SECONDS +
                0.5 * min(0.0, lead.acceleration_mps2) * PREDICT_SECONDS ** 2)
    reserve = 6.0 + frame.speed_mps * follow_seconds + closing ** 2 / 4.0
    if lead.acceleration_mps2 <= -LEAD_BRAKE_ABORT_MPS2 or distance < reserve:
      return False
  return True



def response_has_room(frame, follow_seconds, acceleration):
  """Bounded three-second forecast for ALL observed leads; no lane exclusion.

  Includes increasing ego speed/closing reserve under the proposed acceleration.
  This admission model cannot establish physical destination clearance.
  """
  if not frame.leads or not math.isfinite(acceleration) or acceleration < 0:
    return False
  for lead in frame.leads:
    if lead.acceleration_mps2 <= -LEAD_BRAKE_ABORT_MPS2:
      return False
    for t in (0., .5, 1., 1.5, 2., 2.5, 3.):
      lead_a = min(0., lead.acceleration_mps2)
      ego_speed = frame.speed_mps + acceleration * t
      closing = max(0., -lead.relative_speed_mps + (acceleration - lead_a) * t)
      distance = lead.distance_m + lead.relative_speed_mps * t + .5 * (lead_a - acceleration) * t * t
      reserve = 6. + ego_speed * follow_seconds + closing * closing / 4.
      if distance < reserve:
        return False
  return True

def close_approach(frame):
  """Time gap at 1.5 s, with up to two seconds of closing distance."""
  return bool(frame.leads) and any(
    lead.distance_m <= 6.0 + 1.5 * frame.speed_mps + 2.0 * max(0.0, -lead.relative_speed_mps)
    for lead in frame.leads)


def continuous_leads(previous, current, dt):
  # Vision lead slots are hypotheses, not persistent vehicle identifiers. Require
  # mutual coverage; duplicates and slot swaps need not erase good observations.
  def matches(a, b):
    lateral = (a.lateral_m is None and b.lateral_m is None) or (
      a.lateral_m is not None and b.lateral_m is not None and
      math.isfinite(a.lateral_m) and math.isfinite(b.lateral_m) and abs(a.lateral_m - b.lateral_m) <= .5)
    return (lateral and abs(b.distance_m - (a.distance_m + a.relative_speed_mps * dt)) <= 3.0 and
            abs(b.relative_speed_mps - a.relative_speed_mps) <= 2.0 and
            abs(b.acceleration_mps2 - a.acceleration_mps2) <= 1.0)
  return (all(any(matches(a, b) for b in current) for a in previous) and
          all(any(matches(a, b) for a in previous) for b in current))


def observation_qualified(policy, frame, long_active):
  # Cache observations before intent; this grants no control privilege.
  if frame is None:
    return False
  return qualified(policy, replace(frame, direction='left', signal_direction='left',
                                   left_blindspot=False, right_blindspot=False), long_active)


class LeadHistory:
  def __init__(self):
    self.reset()

  def reset(self):
    self.stamp = None
    self.leads = ()
    self.stable_seconds = 0.0
    self.dt = 0.0

  def update(self, frame):
    if frame is None:
      self.reset()
      return False
    previous = self.stamp
    if previous is not None and frame.model_ns == previous:
      self.dt = 0.0
      return False
    delta = frame.model_ns - previous if previous is not None else 0
    self.dt = delta / 1e9 if 0 < delta <= MAX_MODEL_GAP_NS else 0.0
    same = self.dt > 0 and continuous_leads(self.leads, frame.leads, self.dt)
    self.stable_seconds = min(SETTLE_SECONDS, self.stable_seconds + self.dt) if same else 0.0
    self.stamp, self.leads = frame.model_ns, frame.leads
    return previous is None or 0 < delta <= MAX_MODEL_GAP_NS


class LaneChangeGap:
  def __init__(self):
    self.history = LeadHistory()
    self.reset()

  def reset(self):
    self.history.reset()
    self.ramped_follow = None
    self.acceleration_ceiling = None
    self.release_ceiling = 0.0
    self.direction = None
    self.started_ns = None
    self.maneuver_started = False
    self.expired = False
    self.phase = 'idle'
    self.reason = 'idle'
    self.last_policy = None
    self.last_output = 0.0
    self.settled_seconds = 0.0
    self.last_diag_ns = None
    self.last_diag_key = None
    self.frame = None
    self.moving_started_ns = None
    self.response_reason = 'idle'
    self.response_vetoes = ()
    self.response_raised = False
    self.response_limit = None
    self.preparation_ceiling = 0.0
    self.response_aborted = False
    self.response_ever_raised = False

  @property
  def stable_seconds(self):
    return self.history.stable_seconds

  def _release(self):
    # Release from the actual previous output, not an increasing hidden ceiling.
    # Negative native requests always pass through. This is only an upper bound.
    self.release_ceiling = max(0.0, self.last_output) + RELEASE_ACCEL_PER_SECOND * self.history.dt
    self.acceleration_ceiling = self.release_ceiling

  def step(self, policy, frame, *, base_follow, long_active):
    self.ramped_follow = None
    self.frame = frame
    if policy is None or not policy.enabled or not policy.close_gap or not long_active:
      self.reset()
      return None
    if self.last_policy is not None and self.last_policy != policy:
      self.reset()
    self.last_policy = policy
    self.frame = frame
    if not observation_qualified(policy, frame, long_active) or not (
        type(base_follow) in (int, float) and math.isfinite(base_follow) and .5 <= base_follow <= 3.0):
      self.history.reset()
      self.reason = 'invalid_or_inhibited'
      if self.started_ns is not None or self.acceleration_ceiling is not None:
        self.acceleration_ceiling = 0.0
      return None
    fresh = self.history.update(frame)
    if not fresh:
      self.reason = 'duplicate_or_discontinuous'
      if self.started_ns is not None:
        self.acceleration_ceiling = 0.0
      return None
    signal, state = frame.signal_direction, frame.lane_state
    moving = state in ('laneChangeStarting', 'laneChangeFinishing')
    if signal == 'none' and not (moving and self.maneuver_started):
      self.started_ns = None
      self.direction = None
      self.expired = False
      self.maneuver_started = False
      if self.acceleration_ceiling is not None:
        self.phase, self.reason = 'settling', 'signal_released'
        self._release()
      else:
        self.phase, self.reason = 'idle', 'observing'
      return None
    if signal != 'none' and self.direction != signal:
      if self.direction is not None:
        self.history.reset()
        self.history.update(frame)
      self.preparation_ceiling = max(0., self.last_output)
      self.response_aborted = self.response_ever_raised = False
      self.direction, self.started_ns = signal, frame.model_ns
      self.expired = False
      self.maneuver_started = False
    check = frame
    if signal == 'none' and moving and frame.direction == self.direction:
      check = replace(frame, signal_direction=self.direction)
    if not qualified(policy, check, long_active):
      elapsed = frame.model_ns - self.started_ns if self.started_ns is not None else 0
      self.phase, self.reason = 'pending', 'direction_or_admission'
      if elapsed > SYNC_GRACE_NS:
        self.reason = 'waiting_for_valid_intent'
      # No permission while uncertain; do not turn one async frame into expiry.
      if self.acceleration_ceiling is not None:
        self.acceleration_ceiling = 0.0
      if not self.maneuver_started and elapsed >= PREPARE_SECONDS * 1e9:
        self.expired = True
      return None
    if moving:
      if not self.maneuver_started:
        self.moving_started_ns = frame.model_ns
      self.maneuver_started = True
      self.phase, self.reason = 'moving', 'normal_lead_protection'
      # Full native following is restored for BOTH leads. No extra stability or
      # normal-gap forecast wait is needed to permit an already-safe native target.
      # Nothing here discards a lead, raises a target or claims a clear destination.
      self._release()
      return None
    if self.maneuver_started or state != 'preLaneChange':
      self.expired = True
      self.phase, self.reason = 'cancelled', 'manoeuvre_aborted'
      if self.acceleration_ceiling is not None:
        self._release()
      return None
    if self.started_ns is not None and frame.model_ns - self.started_ns >= PREPARE_SECONDS * 1e9:
      self.expired = True
    if self.expired:
      self.phase, self.reason = 'expired', 'preparation_timeout_or_abort'
      if self.acceleration_ceiling is not None:
        self._release()
      return None
    self.settled_seconds = 0.0
    self.phase = 'preparing'
    self.acceleration_ceiling = self.preparation_ceiling
    if not frame.adjacent_lane_visible:
      self.reason = 'lane_geometry'
    elif not frame.leads:
      self.reason = 'no_preparation_lead'
    elif self.stable_seconds + 1e-9 < SETTLE_SECONDS:
      self.reason = 'lead_history'
    elif not approach_has_room(frame, policy.close_gap_seconds):
      self.reason = 'approach_space'
    else:
      # MPC retains its jerk/obstacle constraints. Select the bounded goal now,
      # instead of using up a 0.2s preparation interval ramping a following setting.
      self.reason = 'qualified'
      self.ramped_follow = min(base_follow, policy.close_gap_seconds)
      return self.ramped_follow if self.ramped_follow < base_follow else None
    return None


  def comfort_target(self, native, *, cruise, previous, positive_jerk, blocked, experimental, block_reasons=()):
    """Ease only mild MPC lead-follow braking; model and all later limits remain.

    Indication requests zero acceleration, movement requests normal cruise.
    Strong native braking and uncertain/changed observations withdraw immediately.
    """
    self.response_raised = False
    self.response_limit = None
    self.response_reason = 'inactive'
    self.response_vetoes = tuple(block_reasons)[:16]
    already_aborted = self.response_aborted
    if self.response_ever_raised:
      self.response_aborted = True
    frame, policy = self.frame, self.last_policy
    values = (native, cruise, previous, positive_jerk)
    if (blocked is not False or already_aborted or type(experimental) is not bool or any(type(v) not in (int, float) or not math.isfinite(v) for v in values) or
        not 0 < positive_jerk <= 10 or frame is None or policy is None):
      self.response_reason = 'blocked'
      return native
    check = frame
    if (self.phase == 'moving' and self.maneuver_started and frame.signal_direction == 'none' and
        frame.direction == self.direction):
      check = replace(frame, signal_direction=self.direction)
    if (self.phase not in ('preparing', 'moving') or self.expired or
        not qualified(policy, check, True) or self.history.dt <= 0):
      return native
    if self.phase == 'moving' and (self.moving_started_ns is None or
        frame.model_ns - self.moving_started_ns > MOVING_RESPONSE_SECONDS * 1e9):
      self.response_reason = 'movement_timeout'
      return native
    # Experimental model requests are preserved by the planner minimum below.
    # Do not raise MPC when its model is also braking: the cause is ambiguous.
    if experimental:
      self.response_reason = 'model_braking'
      return native
    if native <= COMFORT_BRAKE_LIMIT:
      self.response_reason = 'strong_braking'
      return native
    if not response_has_room(frame, policy.close_gap_seconds, 0.):
      self.response_reason = 'lead_margin'
      return native
    # A fresh, otherwise admitted observation may change lead hypothesis.
    # Withdraw now, but allow the existing 0.5s stable-history gate to requalify.
    # Actual hazards, invalid authority, cancellation and active owners above
    # still latch withdrawal after an already-raised response.
    self.response_aborted = False
    if self.history.stable_seconds + 1e-9 < SETTLE_SECONDS:
      self.response_reason = 'lead_history'
      return native
    goal = min(cruise, 0.) if self.phase == 'preparing' else cruise
    if goal <= native:
      self.response_reason = 'native_sufficient'
      return native
    # Release negative acceleration at 2m/s³; positive acceleration uses normal
    # selected/native jerk. Split a zero-crossing so positive jerk is never exceeded.
    dt = self.history.dt
    ramp = previous
    if ramp < 0.:
      release_time = min(dt, -ramp / BRAKE_RELEASE_JERK)
      ramp = min(0., ramp + BRAKE_RELEASE_JERK * release_time)
      dt -= release_time
    ramp += positive_jerk * dt
    goal = min(goal, ramp)
    if goal > 0.:
      lo, hi = 0., goal
      for _ in range(16):
        mid = (lo + hi) * .5
        if response_has_room(frame, policy.close_gap_seconds, mid):lo = mid
        else:hi = mid
      goal = lo
    self.response_limit = goal
    result = max(native, goal)
    self.response_raised = result > native + 1e-9
    self.response_ever_raised |= self.response_raised
    self.response_reason = ('signal_hold' if self.phase == 'preparing' else 'normal_acceleration') if self.response_raised else 'native_sufficient'
    return result

  def limit_acceleration(self, target):
    result = target if self.acceleration_ceiling is None else min(target, self.acceleration_ceiling)
    self.last_output = result
    # Once the native request is below the releasing envelope, hand control back.
    # Never clear a preparation hold or an invalid-data hold this way.
    if (self.phase == 'settling' and self.history.dt > 0 and target >= 0 and
        self.acceleration_ceiling is not None and target <= self.acceleration_ceiling):
      self.settled_seconds += self.history.dt
      if self.settled_seconds >= SETTLE_SECONDS:
        self.acceleration_ceiling = None
        self.phase = 'idle'
    else:
      self.settled_seconds = 0.0
    return result

  def diagnostic(self, *, now_ns=None, **targets):
    if self.phase == 'idle' and self.started_ns is None:
      return None
    ns = now_ns if type(now_ns) is int else (self.frame.model_ns if self.frame is not None else None)
    if ns is None:
      return None
    key = (self.phase, self.reason, self.ramped_follow is not None, self.response_reason, self.response_raised, self.response_vetoes, self.last_policy)
    if key == self.last_diag_key and self.last_diag_ns is not None and 0 <= ns - self.last_diag_ns < 250_000_000:
      return None
    self.last_diag_ns, self.last_diag_key = ns, key
    return dict(revision=5, now_ns=ns, model_ns=self.frame.model_ns if self.frame is not None else None,
                car_ns=self.frame.car_ns if self.frame is not None else None,
                radar_ns=self.frame.radar_ns if self.frame is not None else None,
                phase=self.phase, reason=self.reason, response_vetoes=self.response_vetoes,
                policy_enabled=self.last_policy.enabled if self.last_policy else None,
                policy_close_gap=self.last_policy.close_gap if self.last_policy else None,
                policy_gap_s=self.last_policy.close_gap_seconds if self.last_policy else None,
                policy_fast_pace=self.last_policy.close_gap_fast_pace if self.last_policy else None,
                policy_duration_s=self.last_policy.duration_s if self.last_policy else None,
                stable_s=self.stable_seconds, follow_s=self.ramped_follow,
                ceiling=self.acceleration_ceiling, response_reason=self.response_reason,
                response_raised=self.response_raised, response_limit=self.response_limit, response_aborted=self.response_aborted, **targets)


class LaneChangePace:
  """Prequalify observations, select once, retain native steering rate limits."""
  def __init__(self):
    self.history = LeadHistory()
    self.selected = False
    self.in_maneuver = False
    self.last_direction = None
    self.last_policy = None
    self.last_diagnostic = None
    self.last_diag_ns = None
    self.reason = 'idle'

  def duration(self, policy, frame, *, long_active):
    normal = policy.duration_s
    moving = frame is not None and frame.lane_state in ('laneChangeStarting', 'laneChangeFinishing')
    if self.last_policy is not None and policy != self.last_policy:
      self.history.reset()
      self.selected = False
    self.last_policy = policy
    observed = policy.close_gap_fast_pace and observation_qualified(policy, frame, long_active)
    if not observed:
      self.history.reset()
      self.selected = False
      self.reason = 'disabled_or_invalid'
    else:
      duplicate = self.history.stamp == frame.model_ns
      fresh = self.history.update(frame)
      check = frame
      if (moving and self.in_maneuver and frame.signal_direction == 'none' and
          frame.direction == self.last_direction):
        check = replace(frame, signal_direction=self.last_direction)
      eligible = (fresh or duplicate) and qualified(policy, check, long_active) and approach_has_room(frame, policy.close_gap_seconds)
      if not moving:
        self.selected = False
        self.reason = 'observing'
      elif not eligible:
        self.selected = False
        self.reason = 'admission_or_space'
      elif not self.in_maneuver:
        self.selected = bool(fresh and frame.adjacent_lane_visible and
                             self.history.stable_seconds + 1e-9 >= SETTLE_SECONDS and close_approach(frame))
        self.reason = 'selected' if self.selected else 'entry_not_qualified'
      elif frame.direction != self.last_direction:
        self.selected = False
        self.reason = 'direction_changed'
      # After entry, fresh admission and space checks govern withdrawal. A
      # different but non-threatening lead does not abruptly change steering pace.
      # Once withdrawn, never select late in this manoeuvre.
    self.in_maneuver = moving
    if frame is not None and moving:
      self.last_direction = frame.direction
    return 3.0 if self.selected else normal

  def diagnostic(self, frame, duration):
    if frame is None or (not self.in_maneuver and frame.signal_direction == 'none'):
      return None
    key = (self.reason, self.selected)
    if key == self.last_diagnostic and self.last_diag_ns is not None and 0 <= frame.model_ns - self.last_diag_ns < 250_000_000:
      return None
    self.last_diagnostic, self.last_diag_ns = key, frame.model_ns
    return dict(revision=3, model_ns=frame.model_ns, reason=self.reason, speed10=self.selected,
                duration_s=duration, stable_s=self.history.stable_seconds)


class LaneGapPreferences:
  REFRESH_NS = 1_000_000_000
  MAX_AGE_NS = 2_000_000_000

  def __init__(self, params):
    self.params = params
    self.policy: LaneChangePolicy | None = None
    self.last_attempt_ns = -self.REFRESH_NS
    self.last_success_ns = -1

  def sample(self, now_ns: int) -> LaneChangePolicy | None:
    if type(now_ns) is not int or now_ns <= 0:
      self.policy = None
      self.last_success_ns = -1
      self.last_attempt_ns = -self.REFRESH_NS
      return None
    if now_ns < self.last_attempt_ns:
      self.policy = None
      self.last_success_ns = -1
      self.last_attempt_ns = now_ns - self.REFRESH_NS
    if now_ns - self.last_attempt_ns >= self.REFRESH_NS:
      self.last_attempt_ns = now_ns
      saved = read_saved(self.params)
      self.policy = saved.policy if saved.readable and saved.valid and saved.policy.close_gap else None
      self.last_success_ns = now_ns if saved.readable else -1
    return self.policy if self.last_success_ns > 0 and now_ns - self.last_success_ns <= self.MAX_AGE_NS else None
