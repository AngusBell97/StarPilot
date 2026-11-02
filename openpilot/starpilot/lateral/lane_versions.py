"""Versioned lane preferences and an explicit live handover; v4 keys stay intact."""
from dataclasses import fields, replace
import math
from pathlib import Path
from openpilot.common.params import UnknownKeyName
from openpilot.starpilot.lateral.lane_centering import LaneCenteringController, LaneCenteringSettings, LaneCenteringInput
from openpilot.starpilot.lateral.lane_tuning import NUMBERS, BOOLS, PREFIX, valid_tuning
from openpilot.starpilot.lateral.lane_v5 import LaneCenteringV5Controller, LaneCenteringV51Controller, LaneCenteringV5Settings, LaneCenteringV5Input

from openpilot.starpilot.lateral.lane_v6 import LaneCenteringV6Controller

from openpilot.starpilot.lateral.stock_dom_adapter import STOCK_DEFAULTS, StockController, settings as stock_settings
from openpilot.starpilot.lateral.lane_v8_adapter import V8_DEFAULTS, V8Controller, settings as v8_settings

VERSION = "LaneCenteringVersion"
INITIALIZED = "LaneCenteringV5Initialized"
PREVIEW = "LaneCenteringV5PreviewTime"
ROOT_FIELDS = {"LaneCentering": "enabled", "LaneCenteringPauseOnSignal": "pause_on_signal",
               "LaneCenterOffset": "offset_m", "LaneCenteringE2EAuthority": "e2e_authority",
               "LaneCenteringStrength": "strength"}
FIELDS = ROOT_FIELDS | {PREFIX + spec[0]: field for field, spec in NUMBERS.items()} | {key: field for field, key in BOOLS.items()}
KEY_MAP = {key: key.replace(PREFIX, "LaneCenteringV5") if key.startswith(PREFIX) else
           "LaneCenteringV5" + {"LaneCentering": "Enabled", "LaneCenteringPauseOnSignal": "PauseOnSignal",
           "LaneCenterOffset": "Offset", "LaneCenteringE2EAuthority": "E2EAuthority", "LaneCenteringStrength": "Strength"}[key] for key in FIELDS}
KEY_MAP["LaneCentering"] = "LaneCentering"  # Common master switch, including the native controls.
REVERSE = {value: key for key, value in KEY_MAP.items() if value != key}
DEFAULTS = {KEY_MAP[key]: getattr(LaneCenteringV5Settings(enabled=True, clearance=.20), name) for key, name in FIELDS.items() if KEY_MAP[key] != key}
DEFAULTS.update({PREVIEW: 3., INITIALIZED: False, VERSION: 4})

V6_INITIALIZED = "LaneCenteringV6Initialized"
V6_PREVIEW = "LaneCenteringV6PreviewTime"
V6_MAP = {k: v.replace("LaneCenteringV5", "LaneCenteringV6") for k, v in KEY_MAP.items()}
REVERSE.update({v: k for k, v in V6_MAP.items() if k != v})
DEFAULTS.update({k.replace("LaneCenteringV5", "LaneCenteringV6"): v for k, v in list(DEFAULTS.items()) if k.startswith("LaneCenteringV5")})

V51_INITIALIZED = "LaneCenteringV51Initialized"
V51_PREVIEW = "LaneCenteringV51PreviewTime"
V51_RECOVERY = {"LaneCenteringV51RecoveryTime": "recovery_time", "LaneCenteringV51RecoveryMin": "recovery_min"}
V51_MAP = {k: v.replace("LaneCenteringV5", "LaneCenteringV51") for k, v in KEY_MAP.items()}
REVERSE.update({v: k for k, v in V51_MAP.items() if k != v})
DEFAULTS.update({k.replace("LaneCenteringV5", "LaneCenteringV51"): v for k, v in list(DEFAULTS.items()) if k.startswith("LaneCenteringV5")})
DEFAULTS[V51_MAP["LaneCenteringStrength"]] = 1.5
DEFAULTS["LaneCenteringV51MaxCorrection"] = .0024  # Start at the existing 200% ceiling.
DEFAULTS.update({"LaneCenteringV51RecoveryTime": 3., "LaneCenteringV51RecoveryMin": 2.})
DEFAULTS.update(STOCK_DEFAULTS)
DEFAULTS.update(V8_DEFAULTS)

def version_keys(version):
  if version == 51: return V51_MAP, V51_INITIALIZED, V51_PREVIEW
  return (V6_MAP, V6_INITIALIZED, V6_PREVIEW) if version == 6 else (KEY_MAP, INITIALIZED, PREVIEW)


def raw(params, key):
  try:
    return Path(params.get_param_path(key)).read_bytes()
  except FileNotFoundError:
    return None


class MappedParams:
  def __init__(self, params, mapping=KEY_MAP): self.params, self.mapping = params, mapping
  def get_param_path(self, key):
    # V5.1 retains the schema key for compatibility but never reads its value.
    # The reader's supported missing-key fallback is normalized below.
    if self.mapping is V51_MAP and key == "LaneCenteringStrength": raise UnknownKeyName(key)
    return self.params.get_param_path(self.mapping.get(key, key))


def read_selected(params):
  from openpilot.starpilot.lateral.lane_runtime import read_settings
  try:
    source = raw(params, VERSION)
    if source in (None, b"4"):
      return 4, read_settings(params)
    if source == b"9":
      return 9, LaneCenteringSettings()  # Chubbs runs only inside modeld.
    if source == b"7":
      settings = stock_settings(params)
      return (7, settings) if raw(params, VERSION) == source else (0, LaneCenteringSettings())
    if source == b"8":
      settings = v8_settings(params)
      return (8, settings) if raw(params, VERSION) == source else (0, LaneCenteringSettings())
    if source not in (b"5", b"6", b"51"):
      return 0, LaneCenteringSettings()
    version = int(source)
    mapping, initialized, preview_key = version_keys(version)
    if raw(params, initialized) != b"1":
      return 0, LaneCenteringSettings()
    settings = read_settings(MappedParams(params, mapping), defaults={old: DEFAULTS[new] for old, new in mapping.items() if new in DEFAULTS})
    recovery = {}
    if version == 51:
      settings = replace(settings, strength=1.5)
      for key, field in V51_RECOVERY.items():
        value = raw(params, key)
        recovery[field] = float(DEFAULTS[key] if value is None else value)
      if not all(math.isfinite(value) and 1 <= value <= 4 for value in recovery.values()) or recovery["recovery_min"] > recovery["recovery_time"]:
        return 0, LaneCenteringSettings()
    preview = float(raw(params, preview_key) or b"3.0")
    if not math.isfinite(preview) or not 1 <= preview <= 4 or raw(params, VERSION) != source:
      return 0, LaneCenteringSettings()
    return version, LaneCenteringV5Settings(**{f.name: getattr(settings, f.name) for f in fields(settings)}, preview_time=preview, **recovery)
  except (OSError, ValueError, TypeError, UnknownKeyName):
    return 0, LaneCenteringSettings()


def seed_values(params, version=5):
  if version in (6, 51):
    selected, settings = read_selected(params)
    if version == 51 and selected != 5: raise ValueError("Select a valid v5 tune before initializing v5.1")
    if selected not in (4, 5): raise ValueError("Select a valid v4 or v5 tune before initializing v6")
    source = FIELDS if selected == 4 else KEY_MAP.values()
    captured = {key: raw(params, key) for key in source}
    captured[VERSION] = raw(params, VERSION)
    if selected == 5:
      captured[INITIALIZED] = raw(params, INITIALIZED)
      captured[PREVIEW] = raw(params, PREVIEW)
      if version == 51 and captured[INITIALIZED] != b"1": raise ValueError("Source initialization changed")
    if captured[VERSION] not in ((None, b"4") if selected == 4 else (b"5",)):
      raise ValueError("Source version changed")
    if selected == 5 and float(captured[PREVIEW] or b"3.0") != settings.preview_time:
      raise ValueError("Source preview changed")
    if not valid_tuning(settings): raise ValueError("Invalid source tune")
    # A disabled fallback from malformed storage is not a valid tune to copy.
    mapping = {key: key for key in FIELDS} if selected == 4 else KEY_MAP
    for key, field in FIELDS.items():
      value = captured[mapping[key]]
      if value is None: continue
      expected = getattr(settings, field)
      if isinstance(expected, bool):
        if value not in (b"0", b"1") or (value == b"1") != expected: raise ValueError("Invalid source boolean")
      elif not math.isfinite(float(value)) or float(value) != expected: raise ValueError("Invalid source number")
    target_map, _, target_preview = version_keys(version)
    seed = {target_map[key]: (b"1" if getattr(settings, field) else b"0") if isinstance(getattr(settings, field), bool)
            else str(getattr(settings, field)).encode() for key, field in FIELDS.items()}
    if version == 51:
      seed[target_map["LaneCenteringStrength"]] = b"1.5"
      seed["LaneCenteringV51MaxCorrection"] = b"0.0024"
      seed.update({key: str(DEFAULTS[key]).encode() for key in V51_RECOVERY})
    seed[target_preview] = str(getattr(settings, "preview_time", 3.)).encode()
    return captured, seed
  """Capture a valid complete v4 tune. Caller stages and commits marker last."""
  from openpilot.starpilot.lateral.lane_runtime import read_settings
  captured = {key: raw(params, key) for key in FIELDS}
  settings = read_settings(params)
  if not valid_tuning(settings): raise ValueError("Invalid v4 settings")
  for key, value in captured.items():
    if value is None: continue
    expected = getattr(settings, FIELDS[key])
    if isinstance(expected, bool):
      if value not in (b"0", b"1") or (value == b"1") != expected: raise ValueError("Invalid v4 boolean")
    elif not math.isfinite(float(value)) or float(value) != expected: raise ValueError("Invalid v4 number")
  seed = {KEY_MAP[key]: (b"1" if getattr(settings, field) else b"0") if isinstance(getattr(settings, field), bool)
          else str(getattr(settings, field)).encode() for key, field in FIELDS.items()}
  seed.update({KEY_MAP[PREFIX + "Clearance"]: b"0.2", KEY_MAP[PREFIX + "SingleClearance"]: b"0.45", PREVIEW: b"3.0"})
  # Seeding may change width eligibility; reject contradictory copied limits.
  if settings.car_width + .4 >= settings.max_width: raise ValueError("V5 clearance conflicts with maximum lane width")
  return captured, seed


_INPUT_FIELDS = tuple(f.name for f in fields(LaneCenteringInput))


class LaneCenteringVersions:
  def __init__(self):
    self.v4 = LaneCenteringController()
    self.v5 = LaneCenteringV5Controller()
    self.v51 = LaneCenteringV51Controller()
    self.v6 = LaneCenteringV6Controller()
    self.stock = StockController()
    self.v8 = V8Controller()
    self.version = 4
    self.active = self.v4

  def __getattr__(self, name):
    return getattr(self.active, name)

  def reset(self):
    self.v4.reset(); self.v5.reset(); self.v51.reset(); self.v6.reset(); self.stock.reset(); self.v8.reset()

  def update(self, observation, *, version=4, motion=None, measured_curvature=None, lateral_delay_s=None):
    if version not in (4, 5, 6, 7, 8, 51):
      self.reset()
      return self.active._result(observation.base_curvature, "invalid_version")
    switched = version != self.version
    if switched:
      inherited = 0. if version in (7, 8) or self.version in (7, 8) else self.active._correction
      self.v4.reset(); self.v5.reset(); self.v51.reset(); self.v6.reset(); self.stock.reset(); self.v8.reset()
      self.version = version
      self.active = {4: self.v4, 5: self.v5, 51: self.v51, 6: self.v6, 7: self.stock, 8: self.v8}[version]
      # Carry the current scalar only. Targets, adaptation and histories are new.
      self.active._correction = inherited
    if version == 7:
      result = self.stock.update(observation, measured_curvature=measured_curvature, lateral_delay_s=lateral_delay_s)
    elif version == 8:
      result = self.v8.update(observation, motion=motion, measured_curvature=measured_curvature)
    elif version == 4:
      result = self.v4.update(observation)
    else:
      names = _INPUT_FIELDS if type(observation) is LaneCenteringInput else (f.name for f in fields(observation))
      values = {name: getattr(observation, name) for name in names}
      values["motion"] = motion
      result = self.active.update(LaneCenteringV5Input(**values))
    if switched:
      self.active.diagnostic["version_handover"] = version
    return result
