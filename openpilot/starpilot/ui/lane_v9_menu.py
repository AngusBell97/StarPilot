"""Independent v9 controls with live, source-bound edits."""
from dataclasses import replace
import math

from openpilot.starpilot.lateral.lane_v9_adapter import V9_DEFAULTS, V9_VERSION
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

KEYS = frozenset(V9_DEFAULTS)
VERSION = "LaneCenteringVersion"
SELECTED = str(V9_VERSION).encode()
SMOOTHING_PAGE = "lane/smoothing"
PAGES = (SMOOTHING_PAGE,)
PAUSE = "LaneCenteringV9PauseOnSignal"
SPECS = {
  "LaneCenteringV9Offset": ("Lane offset", -.3, .3, .01, "m",
    "Move the lane-centre target left or right, in metres. Saved separately for v9."),
  "LaneCenteringV9E2EAuthority": ("Model path preference", 0., 1., .05, "%",
    "Higher values let a confident model path reduce v9 lane-centering correction."),
  "LaneCenteringV9Strength": ("Lane Centering Strength", .5, 3., .015, "%",
    "5 is normal; lower values reduce centering and higher values strengthen it. Correction limits still apply."),
  "LaneCenteringV9BuildupTime": ("Correction buildup time", 0., 1., .01, "s",
    "Smooths increases in correction. On a reversal, the old correction unwinds to zero before the opposite correction builds. Higher values respond more gradually; zero bypasses buildup smoothing."),
  "LaneCenteringV9UnwindTime": ("Correction unwind time", 0., 1., .01, "s",
    "Smooths decreases in correction while lane guidance remains usable. On a reversal, this brings the old correction to zero before opposite buildup begins. Zero bypasses unwind smoothing."),
  "LaneCenteringV9ReleaseTime": ("Return-to-model time", 0., 1., .01, "s",
    "Smooths the return to the model when lane guidance becomes unusable. Higher values retain correction longer; zero bypasses fallback smoothing. Safety and disengagement gates can still remove correction immediately."),
}
TIMERS = frozenset(key for key, spec in SPECS.items() if spec[4] == "s")


def _scale(key):
  return 100 / 1.5 if key.endswith("Strength") else 100 if SPECS[key][4] == "%" else 1


def rows(owner, page, allowed):
  if page not in ("lane", SMOOTHING_PAGE):
    return []
  result = []
  if page == "lane":
    result = [owner._bool_row("LaneCentering", "Enable Lane Centering", allowed),
              owner._bool_row(PAUSE, "Pause on signal", allowed)]
    if owner._raw("LaneCentering") is None:
      result[0] = replace(result[0], value="Off", default_value="Off")
  for key, (label, low, high, step, unit, help_text) in SPECS.items():
    if (key in TIMERS) != (page == SMOOTHING_PAGE):
      continue
    value, source, valid = owner._value(key)
    scale = _scale(key)
    try:
      number = float(value)
      valid = valid and math.isfinite(number) and low <= number <= high
    except (TypeError, ValueError):
      number, valid = 0., False
    strength = key.endswith("Strength")
    result.append(FeatureRow(key, label,
      str(round(number * scale) if strength else round(number * scale, 4)) if valid else "Invalid saved value", source,
      available=allowed and valid, minimum=33 if strength else low * scale, maximum=high * scale,
      step=(1 if strength else step * scale) if valid else 0., unit=unit, display_unit=unit,
      default_value=str(round(V9_DEFAULTS[key] * scale) if strength else V9_DEFAULTS[key] * scale),
      reason=help_text if valid else "Invalid saved value"))
  result = [replace(row, dependencies=((VERSION, SELECTED),)) for row in result]
  if page == "lane":
    result.append(FeatureRow("", "Smoothing", "Manage independent buildup, unwind and fallback release times. Zero bypasses only the corresponding filter.",
                             page=SMOOTHING_PAGE, available=True))
  return result


def error(owner, request):
  if request.key not in KEYS:
    return "Unknown v9 setting"
  if owner._raw(VERSION) != SELECTED or request.dependencies != ((VERSION, SELECTED),):
    return "Lane-centering version changed; refresh settings"
  current, source, valid = owner._value(request.key)
  if not valid or source != request.expected:
    return "Saved v9 setting changed; refresh settings"
  if request.key == PAUSE:
    return "" if current in ("0", "1") and request.value in ("On", "Off") and not request.display_unit else "Invalid pause setting"
  _, low, high, _, unit, _ = SPECS[request.key]
  if request.display_unit != unit:
    return "Slider units changed; refresh settings"
  try:
    value = float(request.value)
    stored = max(.5, value / _scale(request.key)) if request.key.endswith("Strength") else value / _scale(request.key)
    if not math.isfinite(float(current)) or not low <= float(current) <= high:
      return "Invalid saved v9 value"
    if not math.isfinite(value) or not low <= stored <= high or (request.key.endswith("Strength") and not 33 <= value <= 200):
      return "Value outside setting range"
    if request.key in TIMERS and not math.isclose(value / .01, round(value / .01), rel_tol=0, abs_tol=1e-7):
      return "Smoothing time must be 0-1 seconds in 0.01-second steps"
    if request.key.endswith("Strength") and not value.is_integer():
      return "Value does not match setting step"
  except (TypeError, ValueError, OverflowError):
    return "Invalid numeric value"
  return ""


def apply(owner, request):
  from openpilot.starpilot.lateral.lane_runtime import runtime_supported

  def authorized():
    return (not error(owner, request) and runtime_supported(owner.vehicle_params()) and
            bool(request.vehicle_fingerprint) and owner.vehicle_fingerprint() == request.vehicle_fingerprint and
            (owner.authority("lane") or owner.authority("lane_live")) and
            all(owner._readable(key) and owner._raw(key) == value for key, value in request.dependencies))

  if not authorized():
    return False
  if request.key == PAUSE:
    encoded = b"1" if request.value == "On" else b"0"
  else:
    value = float(request.value) / _scale(request.key)
    if request.key.endswith("Strength"):
      value = max(.5, value)
    encoded = str(round(value, 8)).encode()
  return commit_exact(owner.params, key=request.key, max_bytes=128, raw=encoded, expected=request.expected,
                      authorized=authorized, temp_prefix=".lane-v9-").verified
