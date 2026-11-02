"""Versioned Galaxy controls, reusing v4 units, ordered bounds and descriptions."""
from dataclasses import replace
from pathlib import Path
import math
import os
import tempfile
from openpilot.starpilot.ui import lane_v4_menu as v4, lane_stock_menu, lane_v9_menu
from openpilot.starpilot.ui.feature_settings_state import FeatureRow
from openpilot.starpilot.lateral.lane_versions import (VERSION, INITIALIZED, PREVIEW, KEY_MAP, REVERSE,
  DEFAULTS, seed_values, raw, version_keys, V6_PREVIEW, V51_PREVIEW, V51_RECOVERY)
from openpilot.starpilot.saved_document import commit_exact, acquire_native_lock


RECOVERY_HELP = ("Set how gradually the planned path returns to the lane centre. Longer values make a gentler approach; "
                 "shorter values aim to reach the centre sooner. With Curve adaptation on, Max applies on straight roads "
                 "and Min applies to the tightest bends. This changes the approach, not the final centre target. "
                 "A shortened preview can end before recovery completes. Measured in seconds.")
RECOVERY_ADAPTATION = "LaneCenteringV51CurveAdaptation"


def recovery_dependencies(owner, key):
  peer = next(name for name in V51_RECOVERY if name != key)
  return ((peer, owner._raw(peer)), (RECOVERY_ADAPTATION, owner._raw(RECOVERY_ADAPTATION)),
          (VERSION, owner._raw(VERSION)))


def recovery_rows(owner, allowed):
  adaptive = owner._value(RECOVERY_ADAPTATION)[0] == "1"
  rows = []
  for key, field in V51_RECOVERY.items():
    value, source, valid = owner._value(key)
    try: valid = valid and math.isfinite(float(value)) and 1 <= float(value) <= 4
    except (ValueError, TypeError): valid = False
    bend = field == "recovery_min"
    reason = RECOVERY_HELP + (" Turn on Curve adaptation to edit bend settings." if bend and not adaptive else "")
    rows.append(FeatureRow(key, "Minimum centre recovery time" if bend else "Maximum centre recovery time",
      value, source, step=.05 if valid else 0., minimum=1., maximum=4., unit="s", display_unit="s",
      available=allowed and valid and (adaptive or not bend), default_value=str(DEFAULTS[key]),
      dependencies=recovery_dependencies(owner, key), reason=reason if valid else "Invalid saved recovery time"))
  return rows


VERSION_CHOICES = {"v4": 4, "v5": 5, "v5.1": 51, "v6": 6, "Stock Dom": 7, "Chubbs / Blendful": 9, "v9": 90}
VERSION_DESCRIPTIONS = {
  b"4": "v4: Adds a steering correction from one lookahead point, using lane lines or road edges, with single-boundary clearance support.",
  b"5": "v5: Shapes a smooth correction path ahead using two lane lines. Strength controls how much correction it requests.",
  b"51": "v5.1: Uses the full lane-centre target from two lane lines. Centre recovery time shapes the approach; correction limits still apply.",
  b"6": "v6: Checks geometry along the preview and shapes a correction path using lane lines or edges, with single-boundary clearance support.",
  b"7": "Stock Dom: Dom's lane-centering controller adds a bounded steering correction at one lookahead point, with single-line tracking support.",
  b"9": "Chubbs / Blendful: Blends lane guidance into the vision model's steering request, using its own confidence weighting and smoothing. Switching into or out requires parking and applies next drive.",
  b"90": "v9: A separately tuned Dom-based controller with custom lane-line acceptance, no correction deadband, and independent buildup, unwind and fallback smoothing.",
}


def selected_version(raw_version):
  return int(raw_version) if raw_version in (b"5", b"6", b"51") else 5


class View:
  def __init__(self, owner):
    self.owner = owner
    self.mapping = version_keys(selected_version(owner._raw(VERSION)))[0]
  def __getattr__(self, name): return getattr(self.owner, name)
  def _raw(self, key): return self.owner._raw(self.mapping.get(key, key))
  def _readable(self, key): return self.owner._readable(self.mapping.get(key, key))
  def _default(self, key):
    actual = self.mapping.get(key, key)
    if actual in DEFAULTS:
      value = DEFAULTS[actual]
      return str(int(value)) if isinstance(value, bool) else str(value)
    return self.owner._default(key)
  def _value(self, key):
    value = self._raw(key)
    try: return (self._default(key) if value is None else value.decode(), value, self._readable(key))
    except (UnicodeDecodeError, ValueError, OSError): return ("invalid saved value", value, False)
  def _number_row(self, *args, **kwargs): return type(self.owner)._number_row(self, *args, **kwargs)
  def _bool_row(self, *args, **kwargs): return type(self.owner)._bool_row(self, *args, **kwargs)


def logical_request(request):
  return replace(request, key=REVERSE.get(request.key, request.key), dependencies=tuple(
    (REVERSE.get(key, key), value) for key, value in request.dependencies if key != VERSION))


def range_descriptor(row):
  if row.key in V51_RECOVERY:
    return dict(group="centre_recovery", end="min" if V51_RECOVERY[row.key] == "recovery_min" else "max",
                label="Centre recovery time", help=RECOVERY_HELP,
                enabled=dict(row.dependencies).get(RECOVERY_ADAPTATION) in (None, b"1"))
  return v4.range_descriptor(replace(row, key=REVERSE.get(row.key, row.key), dependencies=tuple(
    (REVERSE.get(key, key), value) for key, value in row.dependencies if key != VERSION)))


def error(owner, request):
  if request.key == VERSION:
    if request.value not in VERSION_CHOICES: return "Choose v4, v5, v5.1, v6, Stock Dom, Chubbs / Blendful or v9"
    if (request.value == "Chubbs / Blendful" or owner._raw(VERSION) == b"9") and not owner.authority("parked_preferences"):
      return "Switching into or out of Chubbs / Blendful requires parking and applies next drive"
    if request.value == "v5.1" and owner._raw(version_keys(51)[1]) != b"1":
      try: seed_values(owner.params, 51)
      except (OSError, ValueError, TypeError): return "Select a valid v5 tune before initializing v5.1"
    return ""
  version = owner._raw(VERSION)
  mapping, initialized, preview = version_keys(selected_version(version))
  if request.key not in REVERSE and request.key not in V51_RECOVERY and request.key != preview: return "Unknown lane setting"
  if version not in (b"5", b"6", b"51") or request.key not in set(mapping.values()) | {preview} | (set(V51_RECOVERY) if version == b"51" else set()) or owner._raw(initialized) != b"1": return "Lane-centering version changed; refresh settings"
  if dict(request.dependencies).get(VERSION) != version: return "Version dependency missing"
  if request.key in V51_RECOVERY:
    try:
      value = float(request.value)
      if not math.isfinite(value) or not 1 <= value <= 4 or not math.isclose((value-1)/.05, round((value-1)/.05), abs_tol=1e-7):
        return "Centre recovery time must be 1-4 seconds in 0.05-second steps"
      if request.display_unit != "s": return "Recovery time units changed; refresh settings"
      if request.dependencies != recovery_dependencies(owner, request.key): return "Related settings changed; refresh settings"
      bend = V51_RECOVERY[request.key] == "recovery_min"
      if bend and owner._value(RECOVERY_ADAPTATION)[0] != "1": return "Turn on Curve adaptation to edit bend settings"
      peer = next(key for key in V51_RECOVERY if key != request.key)
      peer_value = float(owner._value(peer)[0])
      if not math.isfinite(peer_value) or not 1 <= peer_value <= 4: return "Invalid related recovery time"
      lower, upper = (value, peer_value) if bend else (peer_value, value)
      if lower > upper: return "Minimum centre recovery time cannot exceed Maximum centre recovery time"
      return ""
    except (ValueError, TypeError, OSError): return "Invalid centre recovery time"
  if request.key == preview:
    try:
      value = float(request.value)
      return "" if math.isfinite(value) and 1 <= value <= 4 else "Preview time must be 1-4 seconds"
    except (ValueError, TypeError): return "Invalid preview time"
  logical = logical_request(request)
  if version == b"51" and logical.key == "LaneCenteringStrength": return "Strength is fixed in v5.1"
  if logical.key in v4.CHECK_KEYS: return v4.edit_error(View(owner), logical)
  return ""


def decorate(owner, page, rows, allowed, metric, galaxy):
  version = owner._raw(VERSION)
  if version == b"7":
    rows = lane_stock_menu.rows(owner, page, allowed)
  elif version == b"90":
    rows = lane_v9_menu.rows(owner, page, allowed)
    if not galaxy:
      rows = [row for row in rows if not row.page]
  elif version == b"9":
    rows = [replace(owner._bool_row("LaneCentering", "Enable Lane Centering", allowed),
                    reason="Enable Chubbs lane guidance. Its algorithm choice applies next drive; this master switch remains live."),
            FeatureRow("", "Chubbs / Blendful", "Experimental lane-line guidance in the vision-model process. Uses its own fixed tuning; the normal vehicle steering limits remain. Switching into or out applies next drive. Your other lane-centering tunes are retained.", None)] if page == "lane" else []
  if version in (b"5", b"6", b"51"):
    mapping, initialized, preview = version_keys(int(version))
    proxy = View(owner)
    rows = [proxy._bool_row("LaneCentering", "Enable Lane Centering", allowed),
            proxy._bool_row("LaneCenteringPauseOnSignal", "Pause on signal", allowed),
            v4.present(proxy, proxy._number_row("LaneCenterOffset", "Lane offset", allowed), metric),
            v4.present(proxy, proxy._number_row("LaneCenteringE2EAuthority", "Model path preference", allowed, scale=100, unit="%"), metric)]
    if version != b"51":
      # Keep the existing v5/v6 strength grid used by Galaxy's 1-10 display.
      strength = proxy._number_row("LaneCenteringStrength", "Lane Centering Strength", allowed, scale=100/1.5, unit="%")
      rows.append(replace(strength,
        value=str(round(float(strength.value))) if strength.step > 0 else strength.value,
        default_value=str(round(float(strength.default_value))) if strength.default_value is not None else None,
        minimum=33, maximum=200, step=1 if strength.step > 0 else 0))
    if galaxy:
      _, rows = v4.menu(proxy, page, rows, allowed, metric)
    mapped = []
    for row in rows:
      if version in (b"5", b"51"):
        if row.key in (v4.PREFIX + "EdgeStd", v4.PREFIX + "SingleClearance") or (version == b"51" and row.key == v4.PREFIX + "SingleDeadband"):
          continue
        row = replace(row, label=row.label.replace("Line and edge acceptance", "Lane-line acceptance"),
                      value=row.value.replace("lane lines and road edges", "lane lines"),
                      reason=row.reason.replace("lane lines and road edges", "lane lines").replace("line or edge", "lane line").replace(" Keeping away from a single boundary ignores this offset.", ""))
      if version in (b"5", b"51") and row.key == "LaneCentering":
        row = replace(row, reason="Centre using two accepted lane lines. If either line or the lane geometry becomes unusable, fade correction back to the model.")
      if version == b"5" and row.key == v4.PREFIX + "SingleDeadband":
        row = replace(row, reason="Ignore this much difference from the lane-centre target before requesting correction. Lower values correct smaller differences but may increase steering activity.")
      if row.key == v4.PREFIX + "LookaheadTime":
        value, source, valid = owner._value(preview)
        try: valid = valid and math.isfinite(float(value)) and 1 <= float(value) <= 4
        except ValueError: valid = False
        mapped.append(FeatureRow(preview, "Maximum preview time", value, source, step=.25, minimum=1., maximum=4.,
          unit="s", display_unit="s", available=allowed and valid, default_value="3.0",
          dependencies=((VERSION, version),), reason="How far ahead lane centering shapes its target path. Longer values help prepare for bends, but farther detections may be less reliable. Available geometry and a 60 m distance limit can shorten the usable preview. Measured in seconds."))
        continue
      if not row.key:
        if version == b"51" and row.page == "lane/preferences":
          row = replace(row, value="Adjust correction limits, smoothing, model preference and curve adaptation.")
        mapped.append(row);continue
      reason = row.reason
      if version == b"6":
        reason = {
          v4.PREFIX + "SmoothingTime": "Smooths detected boundary positions before the path is fitted. Higher values reduce small movements but delay response.",
          v4.PREFIX + "SmoothingDistance": "Limits how far smoothed boundaries can lag behind the latest detections. It does not limit movement per update.",
          v4.PREFIX + "ResponseTime": "Controls how quickly the applied correction follows a new request. Lower values react faster; strength controls the amount requested.",
          v4.PREFIX + "MaxWidth": "Largest gap used for centring between two boundaries. V6 checks this throughout its preview; single-boundary clearance may still apply in wider sections.",
        }.get(row.key, reason)
      if version == b"51":
        if row.key == v4.ADAPTATION:
          reason += " Centre recovery time also decreases toward its minimum in tighter bends."
        if row.key in (v4.PREFIX + "ResponseTime", v4.PREFIX + "ResponseMin"):
          reason = "Controls how quickly the applied correction follows its request. Lower values respond faster; higher values smooth changes more gradually. Existing correction limits still apply."
      if row.key == v4.PREFIX + "Clearance":
        reason = "Extra space to aim for on each side when centring between two boundaries. Their minimum usable gap is car body width plus twice this clearance. Prediction, model preference and correction limits can prevent reaching it."
      mapped.append(replace(row, key=mapping.get(row.key, row.key), reason=reason,
           dependencies=tuple((mapping.get(k, k), v) for k, v in row.dependencies) + ((VERSION, version),)))
    rows = mapped
    if version == b"51" and galaxy and page == "lane/preferences/smoothing":
      rows[0:0] = recovery_rows(owner, allowed)
  if page == "lane" and galaxy:
    rows.insert(0, FeatureRow(VERSION, "Lane centering version", "v9" if version == b"90" else "Chubbs / Blendful" if version == b"9" else "Stock Dom" if version == b"7" else "v4" if version in (None, b"4") else "v5.1" if version == b"51" else "v" + version.decode() if version in (b"5", b"6", b"51") else "Invalid saved version",
      version, tuple(VERSION_CHOICES) if owner.authority("parked_preferences") else
      (("Chubbs / Blendful",) if version == b"9" else tuple(k for k in VERSION_CHOICES if k != "Chubbs / Blendful")),
      available=allowed and (version != b"9" or owner.authority("parked_preferences")), default_value="v4",
      reason=VERSION_DESCRIPTIONS.get(version or b"4", "Choose a supported lane-centering version.")))
  return rows


def apply(owner, request):
  from openpilot.starpilot.lateral.lane_runtime import runtime_supported
  key = request.key
  def authorized():
    return ((key != VERSION or (request.value != "Chubbs / Blendful" and owner._raw(VERSION) != b"9") or
             owner.authority("parked_preferences")) and runtime_supported(owner.vehicle_params()) and
            bool(request.vehicle_fingerprint) and owner.vehicle_fingerprint() == request.vehicle_fingerprint and
            (owner.authority("lane") or owner.authority("lane_live")) and
            all(owner._readable(k) and owner._raw(k) == v for k, v in request.dependencies))
  if not authorized() or error(owner, request): return False
  if key == VERSION:
    if request.value in ("v5", "v6", "v5.1") and owner._raw(version_keys(VERSION_CHOICES[request.value])[1]) != b"1":
      return seed_and_select(owner, request, authorized)
    return commit_exact(owner.params, key=VERSION, max_bytes=8, raw=str(VERSION_CHOICES[request.value]).encode(),
      expected=request.expected, authorized=lambda: authorized() and not error(owner, request), temp_prefix=".lane-version-").verified
  logical = logical_request(request)
  try:
    if key in (PREVIEW, V6_PREVIEW, V51_PREVIEW) or key in V51_RECOVERY: value = float(request.value)
    elif logical.key in v4.CHECK_KEYS: value = v4.stored_number(logical)
    elif logical.key == "LaneCenteringStrength":
      percent = float(request.value)
      if not 33 <= percent <= 200: return False
      value = max(.5, percent * 1.5 / 100)
    elif request.value in ("On", "Off"): value = request.value == "On"
    else: return False
    encoded = (b"1" if value else b"0") if isinstance(value, bool) else str(round(value, 8)).encode()
  except (ValueError, TypeError, OverflowError): return False
  return commit_exact(owner.params, key=key, max_bytes=128, raw=encoded, expected=request.expected,
    authorized=lambda: authorized() and not error(owner, request), temp_prefix=".lane-v5-").verified


def seed_and_select(owner, request, authorized):
  """Only marker+selection make a complete first-use snapshot visible to runtime.

  Stage first; use the native lock; retain v4 unchanged. A partial seed is reused
  only if its exact bytes still match this capture, never overwritten silently.
  """
  staged=[];lock=None
  try:
    version = VERSION_CHOICES[request.value]
    _, initialized, _ = version_keys(version)
    captured, seed = seed_values(owner.params, version)
    seed = {key: value for key, value in seed.items() if key != "LaneCentering"}
    destination = Path(owner.params.get_param_path(VERSION)); root=destination.parent.parent
    for key, value in seed.items():
      with tempfile.NamedTemporaryFile(dir=root, prefix=".lane-v5-seed-", delete=False) as f:
        f.write(value);f.flush();os.fsync(f.fileno());staged.append((key, value, Path(f.name)))
    for key, value in ((initialized,b"1"),(VERSION,str(version).encode())):
      with tempfile.NamedTemporaryFile(dir=root,prefix=".lane-v5-seed-",delete=False) as f:
        f.write(value);f.flush();os.fsync(f.fileno());staged.append((key,value,Path(f.name)))
    lock=os.open(root/".lock",os.O_CREAT|os.O_RDONLY,0o775);acquire_native_lock(lock)
    if not authorized() or raw(owner.params,VERSION)!=request.expected: return False
    if any(raw(owner.params,k)!=v for k,v in captured.items()): return False
    if any(raw(owner.params,k) not in (None,v) for k,v in seed.items()): return False
    if raw(owner.params,initialized) not in (None,b"0"): return False
    for key,value,path in staged:
      if not authorized(): return False
      os.replace(path,owner.params.get_param_path(key))
    fd=os.open(destination.parent,os.O_RDONLY)
    try: os.fsync(fd)
    finally: os.close(fd)
    return raw(owner.params,VERSION)==str(version).encode() and all(raw(owner.params,k)==v for k,v in seed.items())
  except (OSError,ValueError,TypeError): return False
  finally:
    if lock is not None: os.close(lock)
    for _,_,path in staged:
      try: path.unlink(missing_ok=True)
      except OSError: pass
