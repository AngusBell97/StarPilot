"""Bounded control diagnostics; learning/UI reasons remain separate."""

from openpilot.common.swaglog import cloudlog


class SmoothnessDiagnostics:
  def __init__(self):
    self.last_ns = 0
    self.previous = None
    self.transitions = 0
    self.samples = 0
    self.active = False
    self.was_sample_active = False
    self.counters = {}
    self.maxima = {}

  def count(self, name):
    self.counters[name] = min(self.counters.get(name, 0) + 1, 1_000_000)

  def maximum(self, name, value):
    self.maxima[name] = max(self.maxima.get(name, 0), min(value, 10_000_000_000))

  def update(self, now_ns, runtime, planner):
    recovery = planner.mode_recovery
    active = runtime.enabled or recovery.armed or recovery.applied
    if not active and not self.active:
      self.counters = {}
      self.maxima = {}
      runtime.continuity_counts = {}
      return
    if active and not self.was_sample_active:
      self.counters = {}
      self.maxima = {}
      runtime.continuity_counts = {}
      self.last_ns = 0
      self.previous = None
      self.transitions = self.samples = 0
    self.was_sample_active = active
    if now_ns == self.last_ns:
      return
    if now_ns < self.last_ns:
      self.last_ns = 0
      self.previous = None
      self.transitions = self.samples = 0
    state = (runtime.eligibility, runtime.cap_latched, runtime.following_lead,
             planner.last_curve_ceiling_applied, recovery.reason, recovery.applied)
    if self.previous is not None and state != self.previous:
      self.transitions += 1
    self.previous = state
    self.samples += 1
    if runtime.cap_latched and not runtime.pending_cap:
      self.count('latched_without_ceiling')
    if self.last_ns and now_ns - self.last_ns < 1_000_000_000:
      return
    cloudlog.event('curve.smoothness', model_ns=runtime.last_model_ns, now_ns=now_ns,
                   eligibility=runtime.eligibility, following=runtime.following_lead,
                   latched=runtime.cap_latched, release_since_ns=runtime.cap_release_since_ns,
                   target=runtime.filter.target, applied=planner.last_curve_ceiling_applied,
                   recovery=recovery.reason, recovery_applied=recovery.applied, recovery_limit=recovery.limit,
                   acceleration=float(planner.output_a_target), samples=self.samples,
                   transitions=self.transitions, input_counts=self.counters, input_maxima_ns=self.maxima,
                   continuity_counts=runtime.continuity_counts)
    self.counters = {}
    self.maxima = {}
    runtime.continuity_counts = {}
    self.last_ns = now_ns
    self.active = active
    self.samples = self.transitions = 0
