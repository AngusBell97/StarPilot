from __future__ import annotations

import json
import math
import numbers
import sys
from copy import deepcopy

PERSONALITY_PROFILES_PARAM = "LongitudinalPersonalityProfiles"
PERSONALITY_IDS = ("traffic", "aggressive", "standard", "relaxed")

ACCELERATION_SPEEDS_MPH = (0, 11, 22, 34, 45, 56, 89)
_ACCELERATION_SPEEDS_MS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 40.0)
BRAKING_SPEEDS_MPH = ACCELERATION_SPEEDS_MPH
FOLLOWING_SPEEDS_MPH = tuple(range(0, 91, 10))

FOLLOWING_PRESET_CURVES = {
  "close": (0.90, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30),
  "medium": (1.20, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55, 1.60),
  "far": (1.55, 1.55, 1.60, 1.70, 1.80, 1.90, 2.00, 2.10, 2.20, 2.30),
}

ACCELERATION_PRESETS = ("eco", "standard", "sport", "sport_plus", "custom")
BRAKING_PRESETS = ("eco", "standard", "sport", "custom")
FOLLOWING_PRESETS = ("close", "medium", "far", "custom")

CURVE_BOUNDS = {
  "acceleration": (0.0, 6.0),
  "braking": (0.5, 2.0),
  "following": (0.75, 3.0),
}

_CATEGORY_SPECS = {
  "acceleration": (ACCELERATION_PRESETS, len(ACCELERATION_SPEEDS_MPH)),
  "braking": (BRAKING_PRESETS, len(BRAKING_SPEEDS_MPH)),
  "following": (FOLLOWING_PRESETS, len(FOLLOWING_SPEEDS_MPH)),
}

_DEFAULT_SELECTIONS = {
  "traffic": ("eco", "standard", "close"),
  "aggressive": ("sport_plus", "sport", "close"),
  "standard": ("standard", "standard", "medium"),
  "relaxed": ("eco", "eco", "far"),
}

_ACCELERATION_PRESET_CURVES_GAS = {
  "eco": (1.50, 1.30, 1.10, 0.90, 0.75, 0.55, 0.35),
  "standard": (2.00, 1.80, 1.55, 1.30, 1.05, 0.85, 0.55),
  "sport": (2.50, 2.25, 1.95, 1.60, 1.30, 1.05, 0.75),
  "sport_plus": (3.50, 3.20, 2.80, 2.35, 1.90, 1.55, 1.15),
}
_ACCELERATION_PRESET_CURVES_EV = {
  "eco": (1.50, 1.34, 1.18, 1.02, 0.90, 0.74, 0.58),
  "standard": (2.00, 1.84, 1.64, 1.44, 1.24, 1.08, 0.84),
  "sport": (2.50, 2.30, 2.06, 1.78, 1.54, 1.34, 1.10),
  "sport_plus": (3.50, 3.26, 2.94, 2.58, 2.22, 1.94, 1.62),
}


def _acceleration_preset_curve(preset: str, ev_tuning: bool) -> list[float]:
  curves = _ACCELERATION_PRESET_CURVES_EV if ev_tuning else _ACCELERATION_PRESET_CURVES_GAS
  return list(curves[preset])

_BRAKING_PRESET_CURVES = {
  "eco": (0.5,) * len(BRAKING_SPEEDS_MPH),
  "standard": (1.0,) * len(BRAKING_SPEEDS_MPH),
  "sport": (2.0,) * len(BRAKING_SPEEDS_MPH),
}


def default_personality_profiles(ev_tuning: bool) -> dict[str, dict]:
  profiles = {}
  for personality in PERSONALITY_IDS:
    acceleration, braking, following = _DEFAULT_SELECTIONS[personality]
    profiles[personality] = {
      "acceleration": {
        "preset": acceleration,
        "curve": _acceleration_preset_curve(acceleration, ev_tuning),
      },
      "braking": {
        "preset": braking,
        "curve": list(_BRAKING_PRESET_CURVES[braking]),
      },
      "following": {
        "preset": following,
        "curve": list(FOLLOWING_PRESET_CURVES[following]),
      },
    }
  return deepcopy(profiles)


def _decode_profiles(raw_profiles) -> dict:
  if isinstance(raw_profiles, bytes):
    raw_profiles = raw_profiles.decode("utf-8", errors="replace")
  if isinstance(raw_profiles, str):
    try:
      raw_profiles = json.loads(raw_profiles)
    except json.JSONDecodeError:
      return {}
  return raw_profiles if isinstance(raw_profiles, dict) else {}


def _validated_category(category: str, raw_category) -> dict | None:
  if not isinstance(raw_category, dict):
    return None

  presets, expected_length = _CATEGORY_SPECS[category]
  preset = raw_category.get("preset")
  curve = raw_category.get("curve")
  if preset not in presets or not isinstance(curve, (list, tuple)) or len(curve) != expected_length:
    return None

  minimum, maximum = CURVE_BOUNDS[category]
  values = []
  for raw_value in curve:
    if isinstance(raw_value, bool) or not isinstance(raw_value, numbers.Real):
      return None
    value = float(raw_value)
    if not math.isfinite(value) or not minimum <= value <= maximum:
      return None
    values.append(round(value, 4))
  return {"preset": preset, "curve": values}


def load_personality_profiles(raw_profiles, ev_tuning: bool) -> dict[str, dict]:
  profiles = default_personality_profiles(ev_tuning)
  decoded = _decode_profiles(raw_profiles)
  for personality in PERSONALITY_IDS:
    raw_profile = decoded.get(personality)
    if not isinstance(raw_profile, dict):
      continue
    for category in _CATEGORY_SPECS:
      validated = _validated_category(category, raw_profile.get(category))
      if validated is not None:
        profiles[personality][category] = validated
  return profiles


def strict_personality_profiles(raw_profiles) -> dict[str, dict] | None:
  if isinstance(raw_profiles, bytes):
    try:
      raw_profiles = raw_profiles.decode("utf-8")
    except UnicodeDecodeError:
      return None
  if isinstance(raw_profiles, str):
    try:
      raw_profiles = json.loads(raw_profiles)
    except json.JSONDecodeError:
      return None
  if not isinstance(raw_profiles, dict) or set(raw_profiles) != set(PERSONALITY_IDS):
    return None

  profiles = {}
  for personality in PERSONALITY_IDS:
    raw_profile = raw_profiles[personality]
    if not isinstance(raw_profile, dict) or set(raw_profile) != set(_CATEGORY_SPECS):
      return None
    profile = {}
    for category in _CATEGORY_SPECS:
      raw_category = raw_profile.get(category)
      if not isinstance(raw_category, dict) or set(raw_category) != {"preset", "curve"}:
        return None
      validated = _validated_category(category, raw_category)
      if validated is None:
        return None
      profile[category] = validated
    profiles[personality] = profile
  return profiles


def serialize_personality_profiles(profiles, ev_tuning: bool) -> str:
  canonical = strict_personality_profiles(profiles)
  if canonical is None:
    raise ValueError("Longitudinal personality profiles must be complete and valid.")
  return json.dumps(canonical, separators=(",", ":"), sort_keys=True)


def update_personality_profile(profiles, personality: str, category: str, preset: str, curve, ev_tuning: bool) -> dict[str, dict]:
  if not isinstance(personality, str) or personality not in PERSONALITY_IDS:
    raise ValueError(f"Unknown personality: {personality}")
  if not isinstance(category, str) or category not in _CATEGORY_SPECS:
    raise ValueError(f"Unknown curve category: {category}")
  if not isinstance(preset, str):
    raise TypeError(f"Unknown {category} preset: {preset}")

  validated = _validated_category(category, {"preset": preset, "curve": curve})
  if validated is None:
    minimum, maximum = CURVE_BOUNDS[category]
    presets, expected_length = _CATEGORY_SPECS[category]
    message = f"Invalid {category} profile: preset must be one of {', '.join(presets)} and curve must contain "
    raise ValueError(message + f"{expected_length} finite values between {minimum} and {maximum}.")
  if preset != "custom":
    validated["curve"] = category_curve(category, validated, ev_tuning)

  updated = load_personality_profiles(profiles, ev_tuning)
  updated[personality][category] = validated
  return updated


def active_personality_id(traffic_mode: bool, personality) -> str | None:
  if traffic_mode:
    return "traffic"
  if isinstance(personality, bool):
    return None
  if isinstance(personality, numbers.Integral):
    personality_value = int(personality)
  else:
    capnp_module = sys.modules.get("capnp.lib.capnp")
    capnp_enum_type = getattr(capnp_module, "_DynamicEnum", None)
    if capnp_enum_type is None or type(personality) is not capnp_enum_type:
      return None
    raw_value = getattr(personality, "raw", None)
    if isinstance(raw_value, bool) or not isinstance(raw_value, numbers.Integral):
      return None
    personality_value = int(raw_value)
  return {0: "aggressive", 1: "standard", 2: "relaxed"}.get(personality_value)


def resolve_personality_profile(raw_profiles, traffic_mode: bool, personality) -> dict | None:
  profiles = strict_personality_profiles(raw_profiles)
  personality_id = active_personality_id(traffic_mode, personality)
  if profiles is None or personality_id is None:
    return None
  return deepcopy(profiles[personality_id])


def category_curve(category: str, config: dict, ev_tuning: bool) -> list[float]:
  if category not in _CATEGORY_SPECS:
    raise ValueError(f"Unknown profile category: {category}")
  validated = _validated_category(category, config)
  if validated is None:
    raise ValueError(f"Invalid {category} profile.")
  preset = validated["preset"]
  if preset == "custom":
    return list(validated["curve"])
  if category == "acceleration":
    return _acceleration_preset_curve(preset, ev_tuning)
  if category == "braking":
    return list(_BRAKING_PRESET_CURVES[preset])
  if category == "following":
    return list(FOLLOWING_PRESET_CURVES[preset])
  raise AssertionError(f"Unhandled profile category: {category}")


def _smooth_interp(value: float, breakpoints: tuple[float, ...], values: list[float]) -> float:
  if value <= breakpoints[0]:
    return float(values[0])
  if value >= breakpoints[-1]:
    return float(values[-1])

  index = next(idx for idx, point in enumerate(breakpoints[1:], start=1) if point >= value) - 1
  t = (value - breakpoints[index]) / float(breakpoints[index + 1] - breakpoints[index])
  t2 = t * t
  t3 = t2 * t
  t4 = t2 * t2
  return float(
    values[index] * (1 - 10 * t3 + 15 * t4 - 6 * t3 * t2)
    + values[index + 1] * (10 * t3 - 15 * t4 + 6 * t3 * t2)
  )


def _linear_interp(x: float, breakpoints: tuple[float, ...], values: list[float]) -> float:
  if x <= breakpoints[0]:
    return float(values[0])
  if x >= breakpoints[-1]:
    return float(values[-1])
  for index in range(1, len(breakpoints)):
    if x <= breakpoints[index]:
      start_x = breakpoints[index - 1]
      fraction = (x - start_x) / (breakpoints[index] - start_x)
      return float(values[index - 1] + fraction * (values[index] - values[index - 1]))
  return float(values[-1])


def interpolate_category_curve(category: str, v_ego: float, config: dict, ev_tuning: bool) -> float:
  values = category_curve(category, config, ev_tuning)
  if category in ("acceleration", "braking"):
    return _smooth_interp(v_ego, _ACCELERATION_SPEEDS_MS, values)
  if category == "following":
    speed_mph = float(v_ego) / 0.44704
    return _linear_interp(speed_mph, FOLLOWING_SPEEDS_MPH, values)
  raise ValueError(f"Unknown profile category: {category}")
