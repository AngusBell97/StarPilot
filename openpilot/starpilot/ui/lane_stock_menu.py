"""Only stock Dom controls, independent stored tuning and stale-edit guards."""
from dataclasses import replace
import math
from openpilot.starpilot.lateral.stock_dom_adapter import STOCK_DEFAULTS
from openpilot.starpilot.ui.feature_settings_state import FeatureRow
from openpilot.starpilot.saved_document import commit_exact
KEYS = frozenset(STOCK_DEFAULTS)
VERSION = "LaneCenteringVersion"
SPECS = {
  "LaneCenteringDomOffset": ("Lane offset", -.3, .3, .01, "m"),
  "LaneCenteringDomE2EAuthority": ("Model path preference", 0., 1., .05, "%"),
  "LaneCenteringDomStrength": ("Lane Centering Strength", .5, 3., .015, "%"),
}
def rows(owner, page, allowed):
  if page != "lane": return []
  result = [owner._bool_row("LaneCentering", "Enable Lane Centering", allowed),
            owner._bool_row("LaneCenteringDomPauseOnSignal", "Pause on signal", allowed)]
  if owner._raw("LaneCentering") is None:
    result[0] = replace(result[0], value="Off", default_value="Off")  # Stock startup default.
  for key, (label, low, high, step, unit) in SPECS.items():
    value, source, valid = owner._value(key)
    scale = 100 / 1.5 if key.endswith("Strength") else 100 if unit == "%" else 1
    try:
      number = float(value)
      valid = valid and math.isfinite(number) and low <= number <= high
    except (TypeError, ValueError): number, valid = 0., False
    result.append(FeatureRow(key, label, str(round(number * scale, 4)) if valid else "Invalid saved value", source,
      available=allowed and valid, minimum=math.ceil(low * scale) if key.endswith("Strength") else low * scale,
      maximum=high * scale, step=step * scale if valid else 0.,
      unit=unit, display_unit=unit, default_value=str(STOCK_DEFAULTS[key] * scale),
      reason={"LaneCenteringDomOffset": "Stock Dom lane offset in metres; saved separately from your custom versions.",
              "LaneCenteringDomE2EAuthority": "Higher values let a confident model path reduce stock lane-centering correction.",
              "LaneCenteringDomStrength": "Stock Dom strength: 100% is normal, with stock correction and activation limits."}[key]))
  return [replace(row, dependencies=((VERSION, b"7"),)) for row in result]

def apply(owner, request):
  from openpilot.starpilot.lateral.lane_runtime import runtime_supported
  def authorized():
    return (owner._raw(VERSION) == b"7" and dict(request.dependencies).get(VERSION) == b"7" and
      runtime_supported(owner.vehicle_params()) and bool(request.vehicle_fingerprint) and
      owner.vehicle_fingerprint() == request.vehicle_fingerprint and
      (owner.authority("lane") or owner.authority("lane_live")) and
      all(owner._readable(k) and owner._raw(k) == v for k, v in request.dependencies))
  if request.key not in KEYS or not authorized(): return False
  try:
    if request.key == "LaneCenteringDomPauseOnSignal":
      if request.value not in ("On", "Off"): return False
      encoded = b"1" if request.value == "On" else b"0"
    else:
      _, low, high, _, unit = SPECS[request.key]
      scale = 100 / 1.5 if request.key.endswith("Strength") else 100 if unit == "%" else 1
      value = float(request.value) / scale
      if not math.isfinite(value) or not low <= value <= high: return False
      encoded = str(round(value, 8)).encode()
  except (TypeError, ValueError, OverflowError): return False
  return commit_exact(owner.params, key=request.key, max_bytes=128, raw=encoded, expected=request.expected,
    authorized=authorized, temp_prefix=".lane-dom-").verified
