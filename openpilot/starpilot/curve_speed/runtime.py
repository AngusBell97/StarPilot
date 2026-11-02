"""Session-local curve-speed decisions; no Params, bus, or actuator access."""

from dataclasses import dataclass

from openpilot.starpilot.curve_speed.learning import LearnedCurve, finite_number
from openpilot.starpilot.curve_speed.target import CurveProfile, TargetFilter, evaluate, MAX_STEP_SECONDS

MODEL_MAX_AGE_NS = 150_000_000
INPUT_MAX_AGE_NS = 150_000_000
OVERRIDE_WATCH_NS = 6_000_000_000
TRAINING_QUIET_NS = 5_000_000_000
TRAINING_SETTLE_NS = 2_000_000_000
LEAD_CLEAR_NS = 1_000_000_000
GLOW_RELEASE_NS = 3_000_000_000
PERSIST_INTERVAL_NS = 30_000_000_000
CRUISING_SPEED = 5.0
ACTIVE_ON_DELTA = 0.5
ACTIVE_OFF_DELTA = 0.25
CAP_RELEASE_NS = 350_000_000
RECOVERY_ANCHOR_NS = 1_000_000_000
RECOVERY_RAW_RISE_MPS = 1.0
RECOVERY_DISTANCE_MARGIN_M = 15.0
IN_CURVE_ACCEL = 1.3
NUDGE = 0.15


@dataclass(frozen=True)
class Frame:
  now_ns: int
  model_ns: int
  inputs_fresh: bool
  profile: CurveProfile | None
  system_longitudinal: bool
  long_active: bool
  enabled: bool
  force_decel: bool
  cruise_mps: float
  ego_mps: float
  measured_curvature: float
  tracking_lead: bool | None
  blinker: bool
  gas_pressed: bool
  brake_pressed: bool
  drive_id: int = 0
  following_lead: bool | None = None


@dataclass(frozen=True)
class DriverEvent:
  producer_session: str
  event_id: int
  observed_ns: int
  kind: str
  button: str
  car_state_ns: int


@dataclass(frozen=True)
class Result:
  candidate_mps: float | None
  ceiling_mps: float | None
  binding_distance_m: float
  reason: str
  training: bool
  progress: float
  dirty_revision: int | None
  learning_reason: str = ''
  apex_speed_mps: float | None = None


class Runtime:
  """The candidate is diagnostic; only ceiling_mps may later reach a planner."""

  def __init__(self, document: object = None, *, enabled: bool = False, no_lead: bool = False):
    loaded = LearnedCurve.load(document)
    self.curve: LearnedCurve = loaded.curve
    self.steering_load_enabled = False
    self.document_valid = loaded.valid
    self.document_reason = loaded.reason
    self.enabled = enabled
    self.no_lead = no_lead
    self.filter = TargetFilter()
    self.cap_latched = False
    self.cap_release_since_ns = 0
    self.eligibility = 'uninitialised'
    self.anchor_pending = False
    self.last_envelope = None
    self.last_envelope_ego = 0.0
    self.continuity_counts = {}
    self.following_lead = None
    self.last_ns = 0
    self.last_model_ns = 0
    self.last_valid_ns = 0
    self.drive_id = 0
    self.was_controlling = False
    self.was_long_active = False
    self.was_gas = False
    self.was_brake = False
    self.last_training_ns = 0
    self.quiet_until_ns = 0
    self.last_persist_ns = 0
    self.last_lead_ns = 0
    self.watch_until_ns = 0
    self.watch_curvature = 0.0
    self.watch_peak = 0.0
    self.nudged = False
    self.override = False
    self.confirmed_ns = 0
    self.pending_ns = 0
    self.pending_target: float | None = None
    self.pending_cap = False
    self.pending_hold = False
    self.pending_cruise = 0.0
    self.pending_ego = 0.0
    self.glow = False
    self.glow_release_since_ns = 0
    self.event_session = ''
    self.last_event_id = -1
    self.last_event_ns = 0
    self.event_floor_ns = 0
    self.retired_sessions: set[str] = set()
    self.event_exhausted = False

  def _reset_filter(self) -> None:
    self.filter.reset()
    self.cap_latched = False
    self.cap_release_since_ns = 0
    self.anchor_pending = False
    self.last_envelope = None

  def _count(self, name):
    self.continuity_counts[name] = min(self.continuity_counts.get(name, 0) + 1, 1_000_000)

  def _ceiling(self, target: float | None, cruise: float, now_ns: int) -> float | None:
    if target is None:
      self.cap_latched = False
      self.cap_release_since_ns = 0
    elif not self.cap_latched:
      self.cap_latched = target < cruise - ACTIVE_ON_DELTA
      self.cap_release_since_ns = 0
    elif target >= cruise - ACTIVE_OFF_DELTA:
      if not self.cap_release_since_ns:
        self.cap_release_since_ns = now_ns
      elif now_ns - self.cap_release_since_ns >= CAP_RELEASE_NS:
        self.cap_latched = False
        self.cap_release_since_ns = 0
    else:
      self.cap_release_since_ns = 0
    return target if self.cap_latched else None

  def reset(self, *, drive_id: int = 0) -> None:
    self.event_floor_ns = max(self.event_floor_ns, self.last_ns, self.last_event_ns)
    self._reset_filter()
    self.last_ns = 0
    self.last_model_ns = 0
    self.last_valid_ns = 0
    self.drive_id = drive_id
    self.was_controlling = False
    self.was_long_active = False
    self.was_gas = False
    self.was_brake = False
    self.last_training_ns = 0
    self.last_lead_ns = 0
    self.quiet_until_ns = 0
    self.watch_until_ns = 0
    self.watch_curvature = 0.0
    self.watch_peak = 0.0
    self.nudged = False
    self.override = False
    self.confirmed_ns = 0
    self.pending_ns = 0
    self.pending_target = None
    self.pending_cap = False
    self.pending_hold = False
    self.pending_cruise = 0.0
    self.pending_ego = 0.0
    self.glow = False
    self.glow_release_since_ns = 0
    self.event_session = ''
    self.last_event_id = -1
    self.last_event_ns = 0
    self.retired_sessions.clear()
    self.event_exhausted = False

  def acknowledge_saved(self, revision: int) -> None:
    self.curve.acknowledge_saved(revision)
    self.last_persist_ns = self.last_ns

  def confirm_applied(self, frame_ns: int, *, applied: bool) -> bool:
    """Call only after planner composition identifies the selected curve owner."""
    if (type(frame_ns) is not int or type(applied) is not bool or frame_ns != self.pending_ns or
        self.confirmed_ns == frame_ns):
      self.was_controlling = False
      self.glow = False
      return False
    self.confirmed_ns = frame_ns
    if self.pending_hold:
      return True
    target = self.pending_target
    selected = applied and self.pending_cap and target is not None
    self.was_controlling = bool(selected and target < self.pending_cruise - 1.0 and
                                self.pending_ego >= target - ACTIVE_OFF_DELTA)
    if not selected and (self.pending_cap or target is None):
      self.glow = False
      self.glow_release_since_ns = 0
    elif self.was_controlling:
      self.glow = True
      self.glow_release_since_ns = 0
      self.quiet_until_ns = frame_ns + TRAINING_QUIET_NS
    elif self.glow and target is not None and target > self.pending_cruise - ACTIVE_OFF_DELTA:
      if not self.glow_release_since_ns:
        self.glow_release_since_ns = frame_ns
      elif frame_ns - self.glow_release_since_ns >= GLOW_RELEASE_NS:
        self.glow = False
        self.glow_release_since_ns = 0
    return True

  def _event(self, event: DriverEvent | None, frame: Frame) -> bool:
    if event is None or self.event_exhausted:
      return False
    if (not event.producer_session or type(event.event_id) is not int or event.event_id < 0 or
        type(event.observed_ns) is not int or not self.event_floor_ns < event.observed_ns <= frame.now_ns or
        frame.now_ns - event.observed_ns > MODEL_MAX_AGE_NS or
        type(event.car_state_ns) is not int or not 0 < event.car_state_ns <= frame.now_ns or
        not event.observed_ns <= event.car_state_ns or
        event.car_state_ns - event.observed_ns > MODEL_MAX_AGE_NS):
      return False
    if event.producer_session in self.retired_sessions:
      return False
    if event.producer_session != self.event_session:
      if event.observed_ns <= self.last_event_ns:
        return False
      if self.event_session:
        self.retired_sessions.add(self.event_session)
        if len(self.retired_sessions) > 64:
          self.event_exhausted = True
          return False
      self.event_session = event.producer_session
      self.last_event_id = -1
    if event.event_id <= self.last_event_id:
      return False
    self.last_event_id = event.event_id
    self.last_event_ns = event.observed_ns
    # SLC confirmation, command receipt and ordinary cruise changes are not
    # evidence of an unconsumed physical press. Card emits the separate kind
    # only after excluding actions owned by SLC.
    return event.kind == 'curveAccelPress' and event.button == 'accel'

  def _finish_watch(self, frame: Frame, measured_accel: float) -> None:
    if not self.watch_until_ns:
      return
    if finite_number(measured_accel) and measured_accel > self.watch_peak:
      self.watch_peak = measured_accel
      self.watch_curvature = abs(frame.measured_curvature)
    if frame.now_ns < self.watch_until_ns and (frame.gas_pressed or frame.brake_pressed or
                                               measured_accel >= IN_CURVE_ACCEL):
      return
    if self.document_valid and self.watch_curvature > 0:
      self.curve.nudge(self.watch_curvature, max(self.watch_peak, self.curve.comfort(self.watch_curvature) + NUDGE))
    self.watch_until_ns = 0

  def step(self, frame: Frame, *, event: DriverEvent | None = None, before_learning=None) -> Result:
    now = frame.now_ns
    self.following_lead = frame.following_lead
    self.eligibility = 'invalid_evidence'
    previous_target = self.pending_target
    if type(now) is not int or now <= 0:
      self.reset()
      return self._result(None, None, 0.0, 'invalid_clock', False, 0)
    if (self.last_ns and (now <= self.last_ns or now - self.last_ns > int(MAX_STEP_SECONDS * 1e9)) or
        self.drive_id and frame.drive_id and frame.drive_id != self.drive_id):
      self.reset(drive_id=frame.drive_id)
    elif self.last_ns and self.confirmed_ns != self.last_ns:
      self.was_controlling = False
    if frame.drive_id:
      self.drive_id = frame.drive_id
    dt = (now - self.last_ns) / 1e9 if self.last_ns else 0.05
    self.last_ns = now
    self.pending_ns = now
    self.pending_target = None
    self.pending_cap = False
    self.pending_hold = False
    valid_profile = (frame.inputs_fresh and frame.profile is not None and
                     type(frame.model_ns) is int and 0 < frame.model_ns <= now and
                     now - frame.model_ns <= MODEL_MAX_AGE_NS and
                     frame.profile.observed_ns == frame.model_ns and
                     frame.model_ns > self.last_model_ns)
    if not valid_profile:
      self.filter.clear_opening()
      # Authenticated driver events may inhibit control even if geometry is
      # unusable. They must never teach a comfort preference in this branch.
      if self._event(event, frame) and (self.was_controlling or self.anchor_pending and self.cap_latched):
        self.override = True
      self.watch_until_ns = 0
      self.last_training_ns = 0
      self.pending_hold = (self.filter.target is not None and self.enabled and self.document_valid and
                           frame.system_longitudinal and frame.enabled and frame.long_active and
                           not frame.gas_pressed and not frame.brake_pressed and not frame.force_decel and
                           not self.override and now - self.last_valid_ns <= RECOVERY_ANCHOR_NS)
      if self.pending_hold:
        if not self.anchor_pending:
          self._count('anchor_started')
        self.anchor_pending = True
        self.pending_target = self.filter.target
        if now - self.last_valid_ns > int(MAX_STEP_SECONDS * 1e9):
          self.was_controlling = False
        # Invalid time is not evidence that the curve has eased. Freeze the
        # release timer rather than restarting it or counting the missing time.
        if self.cap_release_since_ns:
          self.cap_release_since_ns += int(dt * 1e9)
      else:
        if self.anchor_pending:
          self._count('anchor_expired')
        # A gas takeover must not be forgotten just because another input is
        # stale. This is only an inhibition, never learning from stale evidence.
        self.override = self.override or bool((self.was_controlling or self.anchor_pending and self.cap_latched) and frame.gas_pressed and
          previous_target is not None and finite_number(frame.ego_mps) and previous_target < frame.ego_mps - 0.5)
        self._reset_filter()
        self.nudged = False
        self.was_controlling = False
        self.glow = False
        self.was_long_active = frame.long_active
        self.was_gas = frame.gas_pressed
        self.was_brake = frame.brake_pressed
      return self._result(None, None, 0.0, 'stale_or_invalid_evidence', False, now)
    self.last_model_ns = frame.model_ns
    previous_valid_ns = self.last_valid_ns
    recovery_age = (now - previous_valid_ns) / 1e9
    self.last_valid_ns = now
    if (not finite_number(frame.cruise_mps) or frame.cruise_mps <= 0 or
        not finite_number(frame.ego_mps) or frame.ego_mps < 0 or
        frame.ego_mps > 100.0 or not finite_number(frame.measured_curvature) or
        abs(frame.measured_curvature) > 0.1):
      self._reset_filter()
      self.watch_until_ns = 0
      self.last_training_ns = 0
      self.override = False
      self.was_controlling = False
      self.glow = False
      self.was_long_active = frame.long_active
      return self._result(None, None, 0.0, 'invalid_numeric_input', False, now)
    try:
      envelope = evaluate(frame.profile, self.curve, frame.cruise_mps)
    except (ValueError, OverflowError):
      self._reset_filter()
      self.watch_until_ns = 0
      self.last_training_ns = 0
      self.override = False
      self.was_controlling = False
      self.glow = False
      self.was_long_active = frame.long_active
      return self._result(None, None, 0.0, 'invalid_profile', False, now)
    candidate = envelope.speed_mps
    measured_accel = abs(frame.measured_curvature) * frame.ego_mps**2
    in_curve = measured_accel >= IN_CURVE_ACCEL
    accel_press = self._event(event, frame)
    gas_edge = frame.gas_pressed and not self.was_gas
    gas_override = gas_edge and previous_target is not None and previous_target < frame.ego_mps - 0.5
    brake_edge = frame.brake_pressed and not self.was_brake
    long_drop = self.was_long_active and not frame.long_active
    feedback_allowed = self.enabled and self.document_valid and frame.system_longitudinal
    if not feedback_allowed:
      self.watch_until_ns = 0
      self.override = False
      self.nudged = False
    if (self.was_controlling or self.anchor_pending and self.cap_latched) and feedback_allowed and (accel_press or gas_override):
      self.override = True
    if self.override and not in_curve and candidate >= frame.cruise_mps - ACTIVE_OFF_DELTA:
      self.override = False
    gates = ((self.enabled, 'disabled'), (self.document_valid, 'invalid_document'),
             (frame.system_longitudinal, 'stock_longitudinal'), (frame.enabled, 'disengaged'),
             (frame.long_active, 'long_inactive'), (not frame.force_decel, 'force_decel'),
             (frame.ego_mps > CRUISING_SPEED, 'low_speed'), (not frame.gas_pressed, 'gas'),
             (not frame.brake_pressed, 'brake'), (not self.override, 'driver_override'),
             (not self.no_lead or frame.following_lead is False,
              'following_unknown' if frame.following_lead is None else 'following_lead'),
             (not (frame.blinker and not in_curve), 'indicator'))
    self.eligibility = next((reason for passed, reason in gates if not passed), 'eligible')
    allowed = self.eligibility == 'eligible'
    if (self.anchor_pending and self.filter.target is not None and
        self.eligibility == 'following_unknown' and
        all(passed or reason == 'following_unknown' for passed, reason in gates) and
        0 < now - previous_valid_ns <= RECOVERY_ANCHOR_NS):
      # A fresh lead tracker can need one requalification frame after an input
      # interruption. Keep only the bounded target memory through that frame;
      # unknown following never grants a cap or renews the recovery deadline.
      self.last_valid_ns = previous_valid_ns
      self.pending_hold = True
      self.pending_target = self.filter.target
      self.was_controlling = False
      self.watch_until_ns = 0
      self.last_training_ns = 0
      self.last_lead_ns = now
      if self.cap_release_since_ns:
        self.cap_release_since_ns += int(dt * 1e9)
      self.filter.clear_opening()
      self._count('anchor_following_unknown')
      # This is fresh geometry: preserve the map owner's existing per-frame
      # bookkeeping/exclusion callback, still with no visual cap or learning.
      excluded = bool(before_learning(frame, None)) if before_learning is not None else False
      return self._result(candidate, None, envelope.binding_distance_m,
                          'following_unknown', False, now,
                          'map_corner_exclusion' if excluded else 'learning_lead_unknown')
    if not allowed:
      self._reset_filter()
      if not self.enabled or not self.document_valid or not frame.system_longitudinal:
        self.watch_until_ns = 0
    if allowed and self.anchor_pending:
      old = self.last_envelope
      held = self.filter.target
      # Resume only on newly qualified evidence of the same approaching bend.
      # A straightened path or a different, farther binding point starts fresh.
      continuous = (recovery_age <= MAX_STEP_SECONDS or
                    old is not None and held is not None and candidate < held and
                    candidate <= old.speed_mps + RECOVERY_RAW_RISE_MPS and
                    envelope.binding_distance_m <= old.binding_distance_m + RECOVERY_DISTANCE_MARGIN_M +
                    max(frame.ego_mps, self.last_envelope_ego) * recovery_age)
      if continuous:
        self.filter.target = min(held, frame.cruise_mps)
        self._count('anchor_resumed')
      else:
        self._reset_filter()
        self._count('reseed_fresh')
      self.anchor_pending = False
    # Without an existing curve cap or a binding envelope, there is no curve
    # trajectory to release. Tracking a higher selected cruise through the
    # filter would invent a temporary curve cap from the old selected speed.
    # A genuinely latched curve retains its filtered release and hysteresis.
    if allowed and not self.cap_latched and candidate >= frame.cruise_mps:
      self.filter.reset()
    try:
      target = self.filter.step(envelope, ego_mps=frame.ego_mps, cruise_mps=frame.cruise_mps, dt=dt) if allowed else None
    except ValueError:
      self._reset_filter()
      self.eligibility = 'invalid_target'
      target = None
    if allowed and target is not None:
      self.last_envelope = envelope
      self.last_envelope_ego = frame.ego_mps
    was_latched = self.cap_latched
    ceiling = self._ceiling(target, frame.cruise_mps, now)
    if self.cap_latched != was_latched:
      self._count('latch_on' if self.cap_latched else 'latch_off')
    controlling = ceiling is not None
    self.pending_target = target
    self.pending_cap = ceiling is not None
    self.pending_cruise = frame.cruise_mps
    self.pending_ego = frame.ego_mps
    map_exclusion = bool(before_learning(frame, ceiling)) if before_learning is not None else False
    if map_exclusion:
      feedback_allowed = False
      self.watch_until_ns = 0
    if feedback_allowed:
      self._finish_watch(frame, measured_accel)
    if self.was_controlling and not self.nudged and feedback_allowed:
      if (accel_press or gas_override) and finite_number(measured_accel):
        self.watch_until_ns = now + OVERRIDE_WATCH_NS
        self.watch_curvature = abs(frame.measured_curvature)
        self.watch_peak = measured_accel
        self.nudged = True
      elif (brake_edge or long_drop) and in_curve:
        curvature = abs(frame.measured_curvature)
        self.curve.nudge(curvature, self.curve.comfort(curvature) - NUDGE)
        self.nudged = True
    if not controlling and not self.override:
      self.nudged = False
    if frame.tracking_lead is not False:
      self.last_lead_ns = now
      self.last_training_ns = 0
    elif not self.last_lead_ns:
      self.last_lead_ns = now
    manual = not frame.long_active or frame.gas_pressed or frame.brake_pressed
    trainable = (feedback_allowed and manual and
                 frame.ego_mps > CRUISING_SPEED and frame.tracking_lead is False and not frame.blinker and
                 now - self.last_lead_ns >= LEAD_CLEAR_NS and
                 now >= self.quiet_until_ns and finite_number(measured_accel) and
                 not controlling)
    learned = False
    if trainable:
      if not self.last_training_ns:
        self.last_training_ns = now
      if now - self.last_training_ns >= TRAINING_SETTLE_NS and in_curve:
        self.curve.observe(abs(frame.measured_curvature), measured_accel)
        learned = True
    else:
      self.last_training_ns = 0
    learning_reason = ('map_corner_exclusion' if map_exclusion else
                       'learning_disabled' if not self.enabled else
                       'invalid_learning_document' if not self.document_valid else
                       'learning_requires_system_longitudinal' if not frame.system_longitudinal else
                       'learning_requires_driver_speed' if not manual else
                       'learning_requires_cruising_speed' if frame.ego_mps <= CRUISING_SPEED else
                       'learning_lead_unknown' if frame.tracking_lead is None else
                       'learning_following_lead' if frame.tracking_lead else
                       'learning_lead_clear_wait' if now - self.last_lead_ns < LEAD_CLEAR_NS else
                       'learning_indicator_active' if frame.blinker else
                       'learning_feedback_quiet_wait' if now < self.quiet_until_ns else
                       'learning_controller_active' if controlling else
                       'learning_settling' if now - self.last_training_ns < TRAINING_SETTLE_NS else
                       'learning' if learned else 'learning_waiting_for_curve')
    self.was_long_active = frame.long_active
    self.was_gas = frame.gas_pressed
    self.was_brake = frame.brake_pressed
    reason = ('available' if controlling else 'not_binding' if allowed else
              'invalid_learning_document' if not self.document_valid else 'not_eligible')
    return self._result(candidate, ceiling, envelope.binding_distance_m, reason, learned, now, learning_reason, envelope.apex_speed_mps)

  def _result(self, candidate: float | None, ceiling: float | None, distance: float,
              reason: str, training: bool, now_ns: int, learning_reason: str | None = None, apex_speed_mps: float | None = None) -> Result:
    revision = (self.curve.revision if self.document_valid and self.curve.dirty and
                (not training or now_ns - self.last_persist_ns >= PERSIST_INTERVAL_NS) else None)
    return Result(candidate, ceiling, distance, reason, training, self.curve.progress, revision, learning_reason or reason, apex_speed_mps)
