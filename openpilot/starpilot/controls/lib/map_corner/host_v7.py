"""SP-034 adapter to 7.0's validated curve frame and actual planner ownership.

The worker retains its local store/geometry identities. No weather observation is
invented: weather is deliberately no longer a learning gate (SP-031 decision 5).
"""
from dataclasses import replace
import math

from openpilot.cereal.services import SERVICE_LIST
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.curve_speed.learning import MAX_CURVATURE
from openpilot.starpilot.curve_speed.runtime import CRUISING_SPEED, IN_CURVE_ACCEL
from openpilot.starpilot.curve_speed.target import MIN_SPEED, MAX_LATERAL_ACCEL
from .integration import MapFeature
from .types import Fix, Frame


def visual_evidence(profile, model, comfort):
  rates = tuple(model.orientationRate.z)
  if len(rates) != len(profile.curvatures):
    return ()
  evidence = []
  for k, d, rate in zip(profile.curvatures, profile.distances, rates, strict=True):
    k = min(k * 1.23 if k >= .004 and d >= 30 else k, MAX_CURVATURE)
    denominator = max(k, 1e-4)
    raw = math.sqrt(comfort.comfort(k) / denominator)
    unclamped = MIN_SPEED + .1 < raw < math.sqrt(MAX_LATERAL_ACCEL / denominator) - .1
    evidence.append((d, math.copysign(k, rate), unclamped))
  return tuple(evidence)


def fresh(sm, service, now_ns):
  try:
    stamp = sm.logMonoTime[service]
    return (type(stamp) is int and 0 < stamp <= now_ns and
            now_ns - stamp <= int(2e9 / SERVICE_LIST[service].frequency) and
            bool(sm.valid[service]) and bool(sm.alive[service]))
  except (KeyError, AttributeError, TypeError, ZeroDivisionError):
    return False


class MapHost(MapFeature):
  def __init__(self, params):
    super().__init__()
    self.params = params
    self.enabled = self.learn = False
    self.settings_ns = -1_000_000_000
    self.drive_id = 0
    self.pending = None
    self.decision = None
    self.result_reason = 'off'
    self.winner = 'none'
    self.model_ns = 0

  def refresh(self, now_ns):
    # Called before acquiring planner inputs, never from the control callback.
    if now_ns < self.settings_ns or now_ns - self.settings_ns >= 1_000_000_000:
      self.settings_ns = now_ns
      self.enabled = (read_saved(self.params, 'MapCornerSpeed', 8) == (b'1', True) and
                      read_saved(self.params, 'SafeMode', 8) in ((None, True), (b'0', True)))
      self.learn = self.enabled and read_saved(self.params, 'LearnCornerSpeeds', 8) == (b'1', True)

  def begin(self, drive_id):
    self.pending = self.decision = None
    self.winner = 'none'
    self.model_ns = 0
    if drive_id != self.drive_id:
      self.close()
      from .controller import Coordinator
      self.coordinator = Coordinator()
      self.fix = self.fix_id = None
      self.drive_id = drive_id

  def sample(self, sm, curve_host, planner, frame, visual, *, event=None, other=None):
    self.model_ns = frame.model_ns
    now = frame.now_ns / 1e9
    parent = bool(self.enabled and curve_host.runtime.enabled and curve_host.runtime.document_valid)
    auto = bool(frame.long_active and not frame.gas_pressed and not frame.brake_pressed)
    car = sm['carState']
    events_current = fresh(sm, 'onroadEvents', frame.now_ns)
    events = sm['onroadEvents'] if events_current else ()
    fault = any(e.immediateDisable or e.softDisable for e in events)
    cancel = any(e.userDisable for e in events)
    cancel |= any(str(e.type) == 'cancel' and e.pressed for e in car.buttonEvents)
    if self.coordinator.previous_auto and not auto and not (frame.gas_pressed or frame.brake_pressed or cancel):
      fault = True
    lead = frame.tracking_lead is not False
    stop = bool(frame.force_decel or planner.force_stop_plan.forcing or sm['modelV2'].action.shouldStop)
    from openpilot.starpilot.navigation.intent import cruise_ceiling
    nav = cruise_ceiling(sm, planner.CP, frame.now_ns, frame.cruise_mps)
    other = frame.cruise_mps if other is None else min(frame.cruise_mps, other)
    if nav is not None:
      other = min(other, nav)
    load_exclusion = bool(getattr(getattr(planner, "steering_load", None), "exclude_learning", False) or
                          getattr(getattr(planner, "mode_recovery", None), "applied", False))
    learn = self.learn and not load_exclusion
    confounded = bool(lead or stop or frame.blinker or nav is not None or not events_current or load_exclusion)
    evidence = ()
    observable = bool(parent and frame.inputs_fresh and frame.profile is not None and
                      not curve_host.runtime.override and not lead and not frame.blinker)
    if observable:
      evidence = visual_evidence(frame.profile, sm['modelV2'], curve_host.runtime.curve)
      observable = bool(evidence)
    result = self.latest_result
    result_fresh = bool(result and result.control_epoch == self.coordinator.control_epoch and
                        0 <= now - result.now <= .5 and 0 <= self.coordinator.odometer - result.odometer <= 30)
    in_curve = abs(frame.measured_curvature) * frame.ego_mps ** 2 >= IN_CURVE_ACCEL
    available = bool(frame.inputs_fresh and frame.system_longitudinal and frame.enabled and
                     frame.ego_mps > CRUISING_SPEED and not stop and not fault and
                     (not curve_host.runtime.no_lead or frame.following_lead is False) and
                     not (frame.blinker and not in_curve))
    button = bool(event is not None and event.kind == 'curveAccelPress' and event.button == 'accel' and
                  0 <= frame.now_ns - event.observed_ns <= 100_000_000)
    decision = self.coordinator.step(now=now, speed=frame.ego_mps, acceleration=float(car.aEgo),
      cruise=frame.cruise_mps, visual=visual or 0.0,
      visual_valid=bool(available and observable and result_fresh and result.visual_confirmed),
      other=other, automated=auto, parent=parent, learn=learn and parent, available=available,
      button=button, gas=frame.gas_pressed, brake=frame.brake_pressed, cancel=cancel,
      confounded=confounded, fault=fault, result=result,
      visual_override=bool(curve_host.runtime.override and button))
    self.decision, self.status = decision, decision.status
    if not parent:
      self.close()
      return decision.suppress_training
    if self.runtime is None:
      self.runtime = self.runtime_factory()
    self.runtime.fence(self.coordinator.observation_fence, learn)
    service = next((s for s in ('gpsLocationExternal', 'gpsLocation') if fresh(sm, s, frame.now_ns)), None)
    if service is None:
      self.fix = self.fix_id = None
    else:
      identity = (service, int(sm.logMonoTime[service]))
      if identity != self.fix_id:
        g = sm[service]
        self.fix = Fix(identity[1] / 1e9, identity[1], float(g.latitude), float(g.longitude),
                       float(g.bearingDeg), float(g.horizontalAccuracy), float(g.speed), bool(g.hasFix))
        self.fix_id = identity
    self.sequence += 1
    self.pending = Frame(now, self.sequence, self.coordinator.control_epoch, self.coordinator.observation_fence,
      self.fix, frame.ego_mps, float(car.aEgo), self.coordinator.odometer, auto, learn, decision.owner,
      decision.intervention, decision.observation_id, confounded, fault, observable, False, evidence,
      decision.binding_key, 0.0, other,
      confound_flags=int(lead) | int(stop) << 1 | int(frame.blinker) << 2 | int(nav is not None) << 3)
    self.result_reason = ('result_missing' if result is None else
                         'result_epoch' if result.control_epoch != self.coordinator.control_epoch else
                         'result_expired' if not 0 <= now - result.now <= .5 else
                         'result_travel' if not 0 <= self.coordinator.odometer - result.odometer <= 30 else 'ready')
    return decision.suppress_training

  def select(self, vision_ceiling):
    from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
    cap = self.decision.cap if self.decision is not None else 0.0
    if self.pending is None or not self.pending.automated or self.pending.fault:
      cap = 0.0
    if cap > 0 and (vision_ceiling is None or cap < vision_ceiling.speed_mps):
      self.winner = 'map'
      return CurveCeiling(cap, self.model_ns)
    self.winner = 'visual' if vision_ceiling is not None else 'none'
    return vision_ceiling

  def confirm(self, planner):
    # Called only after the planner has arbitrated MPC/lead/stop/navigation/coast.
    if self.pending is None:
      # Clock/input failures cannot leave a worker learning from an old owner.
      if self.runtime is not None:
        self.coordinator.invalidate()
        self.runtime.fence(self.coordinator.observation_fence, False)
        self.latest_result = None
      return
    applied = bool(planner.last_curve_ceiling_applied and self.winner == 'map' and self.result_reason == 'ready' and
                   self.decision is not None and not self.decision.skip_key)
    frame = self.pending
    if frame.automated and not applied:
      frame = replace(frame, owner='other')
    if frame.automated and applied and self.decision is not None:
      frame = replace(frame, applied_map_cap=self.decision.cap)
    self.coordinator.previous_owner = frame.owner
    newest, overflow = self.runtime.exchange(frame)
    reason = 'queue_full' if overflow else 'worker_failed' if self.runtime.failed else self.result_reason
    self.runtime.record_diagnostics(self.diagnostics.update(frame, self.latest_result, reason))
    if overflow or self.runtime.failed:
      self.coordinator.invalidate()
      self.runtime.fence(self.coordinator.observation_fence, False)
      self.latest_result = None
    elif newest is not None and (self.latest_result is None or newest.sequence > self.latest_result.sequence):
      self.latest_result = newest
    self.pending = None
