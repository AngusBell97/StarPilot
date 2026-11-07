"""Galaxy-only slider presentation. Native owners retain their storage and guards."""
from dataclasses import replace
import math
import re
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

SPEED = "GalaxySpeedUnits"
# One Galaxy distance choice covers every distance slider. It keeps the old small-measurement param.
DISTANCE = SMALL = "GalaxySmallDistanceUnits"
# The comma's own distance units (driving-screen distances). It reuses the old Galaxy road-distance param so no new
# params key (and params library rebuild) is needed: 0 = same as the comma's speed units (IsMetric), 1 metric, 2 imperial.
COMMA_DISTANCE = ROAD = "GalaxyRoadDistanceUnits"
CHOICES = ("Same as comma", "Metric", "Imperial")
SPECS = {
  COMMA_DISTANCE: ("Comma distance units", ("Same as speed", "Metric", "Imperial"), 0,
                   "Distances on the comma's driving screen: navigation, the lead-distance widget and lead distance info. "
                   "Same as speed follows Comma speed units."),
  SPEED: ("Galaxy speed units", CHOICES, 0,
          "How Galaxy shows speeds on its sliders. Same as comma follows Comma speed units. Galaxy only."),
  DISTANCE: ("Galaxy distance units", CHOICES, 0,
             "How Galaxy shows distances on its sliders (lane widths, offsets, clearances, stopping distances). Same as comma "
             "follows Comma distance units. Galaxy only; changing units does not change your saved tune."),
}
SI = {"mph": .44704, "km/h": 1 / 3.6, "m": 1., "cm": .01, "in": .0254, "ft": .3048, "km": 1000., "mi": 1609.344}
ROAD_KEYS = frozenset(("SLCHigherLimitDistance", "ForceStopDistanceOffset"))


def selection(params, key):
  raw, readable = read_saved(params, key, 8)
  valid = readable and raw in (None, b"0", b"1", b"2")
  return (SPECS[key][2] if raw is None else int(raw)) if valid else None, raw


def rows(params, authorized):
  result = []
  for key, (label, choices, default, help_text) in SPECS.items():
    index, raw = selection(params, key)
    result.append(FeatureRow(key, label, choices[index] if index is not None else "Invalid saved value", raw,
      choices if index is not None else ("Invalid saved value", *choices), available=authorized, reason=help_text, default_value=choices[default]))
  return result


def apply(params, request, authorized):
  if request.key not in SPECS or not authorized(): return False
  choices = SPECS[request.key][1]
  if request.value not in choices or request.dependencies or request.display_unit: return False
  return commit_exact(params, key=request.key, max_bytes=8, expected=request.expected,
    raw=str(choices.index(request.value)).encode(), authorized=authorized, temp_prefix=".galaxy-units-").verified


def category(row):
  if not row.key or row.unit not in SI or not row.step or row.repair_value: return None
  return SPEED if row.unit in ("mph", "km/h") else DISTANCE


def comma_distance(params):
  """The comma's distance units: (metric, dependencies). Same as speed follows IsMetric."""
  speed_raw, speed_readable = read_saved(params, "IsMetric", 8)
  index, raw = selection(params, COMMA_DISTANCE)
  metric = (speed_readable and speed_raw == b"1") if not index else index == 1
  return metric, ((COMMA_DISTANCE, raw), ("IsMetric", speed_raw))


def target(row, index, metric_comma=False):
  # Same as comma: speed rows already follow the comma's units; distances follow them too.
  if index == 0 and category(row) == SPEED: return row.unit
  metric = metric_comma if index == 0 else index == 1
  if category(row) == SPEED: return "km/h" if metric else "mph"
  if row.unit in ("km", "mi"): return "km" if metric else "mi"
  if row.key in ROAD_KEYS: return "m" if metric else "ft"
  if row.unit in ("cm", "in") and not row.key.endswith("CarWidth"): return "cm" if metric else "in"
  return "m" if metric else "ft"


def number(value):
  return format(value, ".12g")


def convert_text(text, source, dest):
  if source == dest: return text
  factor = SI[source] / SI[dest]
  # SLC bands carry speed units in the label as well as the value suffix.
  pattern = r"(-?\d+(?:\.\d+)?)(?:–<(-?\d+(?:\.\d+)?))? " + re.escape(source)
  def converted(match):
    start = number(float(match[1]) * factor)
    return start + ("–<" + number(float(match[2]) * factor) if match[2] else "") + " " + dest
  return re.sub(pattern, converted, text)


def help_text(text, unit):
  names = {"m": "metres", "cm": "centimetres", "ft": "feet", "in": "inches"}
  if unit not in names: return text
  return re.sub(r"\b(?:centimetres|metres|inches|feet)\b", names[unit], text)


def project(params, row, owner=None):
  key = category(row)
  if key is None: return row
  index, raw = selection(params, key)
  deps = row.dependencies + ((key, raw),)
  if index is None:
    return replace(row, available=False, dependencies=deps, reason="Invalid Galaxy slider units; repair them in Localization.")
  metric = False
  if index == 0 and key == DISTANCE:
    metric, comma_deps = comma_distance(params)
    deps += comma_deps
  dest = target(row, index, metric)
  factor = SI[row.unit] / SI[dest]
  if dest == row.unit:
    return replace(row, unit=dest, display_unit=dest, dependencies=deps)
  # Converted: show clean steps in the new units (e.g. 0.05 m rather than 0.03048 m). The display step is never finer
  # than the setting's own step, so each shown value saves to a distinct value on the setting's own grid (restore).
  step = display_step(row.step * factor)
  shown = lambda value: number(round(round(float(value) * factor / step) * step, 6)) if _numeric(value) else value
  low = math.ceil(row.minimum * factor / step - 1e-9) * step
  high = math.floor(row.maximum * factor / step + 1e-9) * step
  converted = replace(row, unit=dest, display_unit=dest, dependencies=deps,
    label=convert_text(row.label, row.unit, dest),
    value=shown(row.value), minimum=round(low, 6), maximum=round(high, 6), step=step,
    default_value=shown(row.default_value),
    presets=tuple((shown(v), convert_text(label, row.unit, dest)) if _numeric(v) else (v, label) for v, label in row.presets))
  return replace(converted, reason=help_text(converted.reason, converted.unit))


NICE_STEPS = (0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 5, 10, 20, 25, 50, 100, 200, 250, 500, 1000)


def display_step(converted_step):
  """The smallest clean step that is at least the setting's own step in the shown units."""
  return next((nice for nice in NICE_STEPS if nice >= converted_step * (1 - 1e-9)), converted_step)


def _numeric(value):
  try: return value is not None and math.isfinite(float(value))
  except (ValueError, TypeError, OverflowError): return False


def restore(params, row, request, owner=None):
  """Return the request in the owner's original units; never change IsMetric."""
  if category(row) is None: return request
  shown = project(params, row, owner)
  # Restore defaults may reset a greyed row (its master switch is off) to its exact native default.
  inactive_reset = (not row.available and row.default_reset_available and not request.direction and
                    selection(params, category(row))[0] is not None and request.value == shown.default_value)
  if (not (shown.available or inactive_reset) or request.dependencies != shown.dependencies or request.expected != row.source or
      request.display_unit != shown.display_unit):
    raise ValueError("Slider units or saved settings changed; refresh settings")
  if request.direction:
    # Offset +/- uses a direction-only request; resolve it in the displayed units.
    value = max(shown.minimum, min(shown.maximum, float(shown.value) + request.direction * shown.step))
  else:
    value = float(request.value)
  if not math.isfinite(value): raise ValueError("Invalid slider value")
  if not request.direction and request.value == shown.default_value:
    restored = row.default_value  # Keep the exact native reset, including off-grid SI defaults.
  else:
    restored_value = value * SI[shown.unit] / SI[row.unit]
    # Save on the setting's own grid: a clean shown value (1.5 m) becomes the nearest value the setting takes (4.9 ft).
    restored_value = row.minimum + round((restored_value - row.minimum) / row.step) * row.step
    restored_value = min(max(restored_value, row.minimum), row.maximum)
    if math.isclose(restored_value, round(restored_value), abs_tol=1e-7, rel_tol=0):
      restored_value = round(restored_value)
    restored = number(restored_value)
  return replace(request, value=restored, dependencies=row.dependencies, display_unit=row.display_unit, direction=0)


def unchanged(params, dependencies):
  return all(read_saved(params, key, 8) == (raw, True) for key, raw in dependencies if key in SPECS)
