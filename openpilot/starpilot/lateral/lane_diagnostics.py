"""Best-effort SP-016 evidence on the existing logMessage stream; no control writes."""
from dataclasses import asdict
import hashlib
import json
import math
import uuid
from openpilot.starpilot.lateral.lane_centering import LaneCenteringSettings
from openpilot.starpilot.lateral.lane_v5 import LaneCenteringV5Settings


def _clean(value):
  if isinstance(value, float):
    return value if math.isfinite(value) else None
  if isinstance(value, dict):
    return {key: _clean(item) for key, item in value.items()}
  if isinstance(value, (list, tuple)):
    return [_clean(item) for item in value]
  return value


class LaneDiagnostics:
  def __init__(self):
    self.session = uuid.uuid4().hex
    self.settings = None
    self.revision = None
    self.last_settings_ns = None
    self.last_model_ns = None
    self.last_emit_ns = None
    self.sequence = 0
    self.failures = 0

  def emit(self, event, controller, request, result, *, model_ns, now_ns, **host):
    # Never let unavailable/malformed diagnostic data or a log sink affect steering.
    try:
      self._emit(event, controller, request, result, model_ns, now_ns, host)
    except Exception:
      self.failures += 1

  def _emit(self, event, controller, request, result, model_ns, now_ns, host):
    if now_ns <= 0 or model_ns <= 0:
      return
    if self.last_emit_ns is not None and now_ns < self.last_emit_ns:
      self.last_emit_ns = self.last_model_ns = self.last_settings_ns = None
    if request is not None and ((not (type(request.settings) in (LaneCenteringSettings, LaneCenteringV5Settings) and request.settings is self.settings) and request.settings != self.settings) or self.last_settings_ns is None or
                                now_ns - self.last_settings_ns >= 60_000_000_000):
      values = asdict(request.settings)
      revision = hashlib.sha256(json.dumps(values, sort_keys=True, allow_nan=False).encode()).hexdigest()[:16]
      event('lane_centering_settings', version=1, session=self.session, revision=revision,
            control_ns=now_ns, model_ns=model_ns, values=values)
      self.settings, self.revision, self.last_settings_ns = request.settings, revision, now_ns
    if result is None or model_ns == self.last_model_ns or (self.last_emit_ns is not None and now_ns - self.last_emit_ns < 40_000_000):
      return
    # At most25Hz, normally once per20Hz model frame. Existing100Hz feedback remains unchanged.
    self.last_model_ns, self.last_emit_ns = model_ns, now_ns
    self.sequence += 1
    if getattr(controller, "version", None) == 8:
      event("lane_centering_diagnostic", version=1, session=self.session, sequence=self.sequence,
        revision=self.revision, control_ns=now_ns, model_ns=model_ns, algorithm=8, reason=result.reason,
        geometry=_clean(controller.diagnostic), requested=result.correction, limit=result.correction_limit, host=_clean(host))
      return
    if getattr(controller, "version", None) in (7, 90):
      event("lane_centering_diagnostic", version=1, session=self.session, sequence=self.sequence,
        revision=self.revision, control_ns=now_ns, model_ns=model_ns, algorithm=controller.version, reason=result.reason, geometry=_clean(controller.diagnostic),
        requested=result.correction, limit=result.correction_limit, host=_clean(host))
      return
    raw = controller._raw
    strength = request.settings.strength if request is not None else None
    pre_cap = raw * .30 if raw is not None else None
    first_cap = max(-result.correction_limit, min(result.correction_limit, pre_cap)) if pre_cap is not None else None
    target = max(-result.correction_limit, min(result.correction_limit, first_cap * strength)) if first_cap is not None and strength is not None else None
    geometry = controller.diagnostic if controller.diagnostic.get('frame') == model_ns else {}
    if geometry.get('algorithm') == 6:
      geometry = {k: v for k, v in geometry.items() if k not in ('v6_modes', 'v6_eligible', 'v6_regions', 'v6_sources')}
    payload = dict(version=1, session=self.session, sequence=self.sequence, failures=self.failures,
      revision=self.revision, control_ns=now_ns, model_ns=model_ns, reason=result.reason,
      mode=controller.selected_mode, geometry=geometry, response_s=controller._response_tau,
      preference=controller._e2e_authority, release_s=request.settings.release_tau if request is not None else None,
      raw_curvature=raw, pre_cap=pre_cap, pre_strength=first_cap, target=target,
      requested=result.correction, limit=result.correction_limit, host=host)
    event('lane_centering_diagnostic', **_clean(payload))
