"""Galaxy-only slider presentation. Native owners retain their storage and guards."""
from dataclasses import replace
import math
import re
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

SPEED = "GalaxySpeedUnits"
ROAD = "GalaxyRoadDistanceUnits"
SMALL = "GalaxySmallDistanceUnits"
SPECS = {
  SPEED: ("Galaxy speed sliders", ("Same as comma", "km/h", "mph"), 0,
          "Display speed sliders independently of the comma's speedometer and wheel units."),
  ROAD: ("Galaxy road-distance sliders", ("Original units", "m / km", "ft / mi"), 0,
         "Units for distances along the road, including SLC higher-limit start distance and stopping-distance adjustment."),
  SMALL: ("Galaxy small-measurement sliders", ("Original units", "m / cm", "ft / in"), 1,
          "Units for lane widths, clearances, offsets and bend radii. Changing units does not change your saved tune."),
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
  if row.unit in ("mph", "km/h"): return SPEED
  return ROAD if row.key in ROAD_KEYS or row.unit in ("km", "mi") else SMALL


def target(row, index):
  if index == 0: return row.unit
  metric = index == 1
  if category(row) == SPEED: return "km/h" if metric else "mph"
  if row.unit in ("km", "mi"): return "km" if metric else "mi"
  if category(row) == ROAD: return "m" if metric else "ft"
  if row.key.startswith("LaneCenter") and (row.key.endswith(("Offset", "Clearance", "SmoothingDistance", "SingleDeadband", "OverrideStart", "OverrideFull"))):
    return "cm" if metric else "in"
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
  dest = target(row, index)
  factor = SI[row.unit] / SI[dest]
  converted = replace(row, unit=dest, display_unit=dest, dependencies=deps,
    label=convert_text(row.label, row.unit, dest),
    value=number(float(row.value) * factor) if _numeric(row.value) else row.value,
    minimum=row.minimum * factor, maximum=row.maximum * factor, step=row.step * factor,
    default_value=number(float(row.default_value) * factor) if _numeric(row.default_value) else row.default_value,
    presets=tuple((number(float(v) * factor), convert_text(label, row.unit, dest)) if _numeric(v) else (v, label)
                  for v, label in row.presets))
  if index and owner is not None and row.key.startswith("LaneCenter"):
    from openpilot.starpilot.ui import lane_v4_menu as lane, lane_versions_menu as versions
    logical = versions.REVERSE.get(row.key, row.key)
    if logical in lane.DISTANCE_KEYS:
      view = versions.View(owner) if row.key in versions.REVERSE else owner
      # Reuse the existing physical bounds/peer guards and clean unit-specific grids.
      shown = lane.present(view, replace(row, key=logical), index == 1)
      converted = replace(converted, value=shown.value, minimum=shown.minimum, maximum=shown.maximum,
        step=shown.step, unit=shown.unit, display_unit=shown.unit, default_value=shown.default_value,
        available=shown.available, reason=shown.reason)
    elif row.key == "LaneCenteringDomOffset":
      converted = replace(converted, minimum=-30 if index == 1 else -11.5, maximum=30 if index == 1 else 11.5,
                          step=1 if index == 1 else .25)
  return replace(converted, reason=help_text(converted.reason, converted.unit))


def _numeric(value):
  try: return value is not None and math.isfinite(float(value))
  except (ValueError, TypeError, OverflowError): return False


def restore(params, row, request, owner=None):
  """Return the request in the owner's original units; never change IsMetric."""
  if category(row) is None: return request
  shown = project(params, row, owner)
  if (not shown.available or request.dependencies != shown.dependencies or request.expected != row.source or
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
    grid = (restored_value - row.minimum) / row.step
    if math.isclose(grid, round(grid), abs_tol=1e-6, rel_tol=0):
      restored_value = row.minimum + round(grid) * row.step
    if math.isclose(restored_value, round(restored_value), abs_tol=1e-7, rel_tol=0):
      restored_value = round(restored_value)
    restored = number(restored_value)
  return replace(request, value=restored, dependencies=row.dependencies, display_unit=row.display_unit, direction=0)


def unchanged(params, dependencies):
  return all(read_saved(params, key, 8) == (raw, True) for key, raw in dependencies if key in SPECS)
