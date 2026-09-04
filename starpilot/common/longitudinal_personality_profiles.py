#!/usr/bin/env python3
"""Versioned, fail-closed longitudinal acceleration/braking profiles."""
from __future__ import annotations

from copy import deepcopy
import json
import math
import numbers

PERSONALITY_PROFILES_PARAM = "LongitudinalPersonalityProfiles"
PROFILE_SCHEMA_VERSION = 1
PERSONALITY_IDS = ("traffic", "aggressive", "standard", "relaxed")
TRUCK_FINGERPRINT_TOKENS = (
  " RAM 1500 ",
  " RAM HD ",
  " F 150 ",
  " MAVERICK ",
  " RANGER ",
  " SILVERADO ",
  " RIDGELINE ",
  " SANTA CRUZ ",
)

ACCELERATION_SPEEDS_MPH = (0.0, 11.184681, 22.369363, 33.554044, 44.738726, 55.923407, 89.477452)
BRAKING_SPEEDS_MPH = ACCELERATION_SPEEDS_MPH
FOLLOWING_SPEEDS_MPH = tuple(range(0, 91, 10))
_SPEEDS_MS = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 40.0)


def is_truck_fingerprint(fingerprint: object) -> bool:
  if not isinstance(fingerprint, str) or not fingerprint.strip():
    return False
  normalized = f" {fingerprint.strip().upper().replace('_', ' ').replace('-', ' ')} "
  return any(token in normalized for token in TRUCK_FINGERPRINT_TOKENS)

ACCELERATION_PRESETS = ("dom_default", "standard", "eco", "sport", "sport_plus", "custom")
BRAKING_PRESETS = ("dom_default", "standard", "eco", "sport", "custom")
FOLLOWING_PRESETS = ("dom_default", "close", "medium", "far", "custom")
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

_BRAKING_PRESET_CURVES = {
  "eco": (0.5,) * len(BRAKING_SPEEDS_MPH),
  "standard": (1.0,) * len(BRAKING_SPEEDS_MPH),
  "sport": (2.0,) * len(BRAKING_SPEEDS_MPH),
}
FOLLOWING_PRESET_CURVES = {
  "close": (0.90, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30),
  "medium": (1.20, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55, 1.60),
  "far": (1.55, 1.55, 1.60, 1.70, 1.80, 1.90, 2.00, 2.10, 2.20, 2.30),
}
PROFILE_AXES = {
  "acceleration": {
    "speed": {"unit": "mph", "values": list(ACCELERATION_SPEEDS_MPH)},
    "value": {"unit": "m/s^2", "meaning": "maximum_requested_acceleration"},
  },
  "braking": {
    "speed": {"unit": "mph", "values": list(BRAKING_SPEEDS_MPH)},
    "value": {"unit": "m/s^2", "meaning": "cruise_slc_deceleration_magnitude"},
  },
  "following": {
    "speed": {"unit": "mph", "values": list(FOLLOWING_SPEEDS_MPH)},
    "value": {"unit": "s", "meaning": "base_time_headway"},
  },
}

_ACCELERATION_PROFILE_IDS = {
  "standard": 0,
  "eco": 1,
  "sport": 2,
  "sport_plus": 3,
}


def _acceleration_preset_curve(preset: str, ev_tuning: bool, truck_tuning: bool) -> list[float]:
  # Import lazily so persisted-schema parsing remains independent of controls/runtime modules.
  from openpilot.starpilot.common.accel_profile import get_accel_profile_curve_values

  # Profile customisation resolves an impossible dual flag deterministically in favour of EV.
  return get_accel_profile_curve_values(
    _ACCELERATION_PROFILE_IDS[preset], bool(ev_tuning), bool(truck_tuning) and not bool(ev_tuning)
  )


def default_personality_profiles(ev_tuning: bool, truck_tuning: bool = False) -> dict[str, dict]:
  del ev_tuning, truck_tuning
  return {
    personality: {
      "acceleration": {"preset": "dom_default", "curve": []},
      "braking": {"preset": "dom_default", "curve": []},
      "following": {"preset": "dom_default", "curve": []},
    }
    for personality in PERSONALITY_IDS
  }


def profile_document(profiles: dict[str, dict], *, enabled: bool) -> dict:
  if type(enabled) is not bool:
    raise ValueError("enabled must be a JSON boolean")
  return {
    "schemaVersion": PROFILE_SCHEMA_VERSION,
    "enabled": enabled,
    "axes": deepcopy(PROFILE_AXES),
    "profiles": deepcopy(profiles),
  }


def _decode_json(raw):
  if isinstance(raw, bytes):
    try:
      raw = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError:
      return None
  if isinstance(raw, str):
    try:
      raw = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
      return None
  return raw


def _validated_category(category: str, raw_category) -> dict | None:
  if category not in _CATEGORY_SPECS or not isinstance(raw_category, dict) or set(raw_category) != {"preset", "curve"}:
    return None
  presets, expected_length = _CATEGORY_SPECS[category]
  preset = raw_category.get("preset")
  curve = raw_category.get("curve")
  if not isinstance(preset, str) or preset not in presets or not isinstance(curve, list):
    return None
  if preset == "dom_default":
    return {"preset": preset, "curve": []} if not curve else None
  if preset != "custom":
    return {"preset": preset, "curve": []} if not curve else None
  if len(curve) != expected_length:
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


def strict_profile_document(raw_document) -> dict | None:
  decoded = _decode_json(raw_document)
  if not isinstance(decoded, dict) or set(decoded) != {"schemaVersion", "enabled", "axes", "profiles"}:
    return None
  if type(decoded["schemaVersion"]) is not int or decoded["schemaVersion"] != PROFILE_SCHEMA_VERSION:
    return None
  if type(decoded["enabled"]) is not bool or decoded["axes"] != PROFILE_AXES:
    return None

  raw_profiles = decoded["profiles"]
  if not isinstance(raw_profiles, dict) or set(raw_profiles) != set(PERSONALITY_IDS):
    return None
  profiles = {}
  for personality in PERSONALITY_IDS:
    raw_profile = raw_profiles.get(personality)
    if not isinstance(raw_profile, dict) or set(raw_profile) != set(_CATEGORY_SPECS):
      return None
    profile = {}
    for category in _CATEGORY_SPECS:
      validated = _validated_category(category, raw_profile.get(category))
      if validated is None:
        return None
      profile[category] = validated
    profiles[personality] = profile
  return profile_document(profiles, enabled=decoded["enabled"])


def strict_personality_profiles(raw_document) -> dict[str, dict] | None:
  document = strict_profile_document(raw_document)
  if document is None or not document["enabled"]:
    return None
  return deepcopy(document["profiles"])


def load_personality_profiles(raw_document, ev_tuning: bool, truck_tuning: bool = False) -> dict[str, dict]:
  document = strict_profile_document(raw_document)
  return deepcopy(document["profiles"]) if document is not None else default_personality_profiles(ev_tuning, truck_tuning)


def serialize_personality_profiles(profiles, ev_tuning: bool, truck_tuning: bool = False, *, enabled: bool) -> str:
  del ev_tuning, truck_tuning
  document = profile_document(profiles, enabled=enabled)
  canonical = strict_profile_document(document)
  if canonical is None:
    raise ValueError("Longitudinal personality profiles must be complete and valid.")
  return json.dumps(canonical, separators=(",", ":"), sort_keys=True, allow_nan=False)


def update_personality_profile(
  profiles, personality: str, category: str, preset: str, curve, ev_tuning: bool, truck_tuning: bool = False,
) -> dict[str, dict]:
  if personality not in PERSONALITY_IDS:
    raise ValueError(f"Unknown personality: {personality}")
  if category not in _CATEGORY_SPECS:
    raise ValueError(f"Unknown profile category: {category}")
  validated = _validated_category(category, {"preset": preset, "curve": curve})
  if validated is None:
    minimum, maximum = CURVE_BOUNDS[category]
    presets, expected_length = _CATEGORY_SPECS[category]
    message = f"Invalid {category} profile: preset must be one of {', '.join(presets)} and curve must contain "
    message += f"{expected_length} finite numeric values between {minimum} and {maximum}."
    raise ValueError(message)

  base_document = profile_document(profiles, enabled=True)
  canonical = strict_profile_document(base_document)
  if canonical is None:
    base = default_personality_profiles(ev_tuning, truck_tuning)
  else:
    base = canonical["profiles"]
  updated = deepcopy(base)
  updated[personality][category] = validated
  return updated


def active_personality_id(traffic_mode: bool, personality) -> str | None:
  if type(traffic_mode) is not bool:
    return None
  if traffic_mode:
    return "traffic"
  if isinstance(personality, bool):
    return None
  raw = getattr(personality, "raw", personality)
  if isinstance(raw, bool) or not isinstance(raw, numbers.Integral):
    return None
  return {0: "aggressive", 1: "standard", 2: "relaxed"}.get(int(raw))


def resolve_personality_profile(raw_document, traffic_mode: bool, personality) -> dict | None:
  profiles = strict_personality_profiles(raw_document)
  personality_id = active_personality_id(traffic_mode, personality)
  if profiles is None or personality_id is None:
    return None
  return deepcopy(profiles[personality_id])


def resolve_personality_category(raw_document, traffic_mode: bool, personality, category: str) -> dict | None:
  profile = resolve_personality_profile(raw_document, traffic_mode, personality)
  if profile is None or category not in _CATEGORY_SPECS:
    return None
  config = profile[category]
  return None if config["preset"] == "dom_default" else deepcopy(config)


def category_curve(category: str, config: dict, ev_tuning: bool, truck_tuning: bool = False) -> list[float]:
  validated = _validated_category(category, config)
  if validated is None:
    raise ValueError(f"Invalid {category} profile configuration.")
  preset = validated["preset"]
  if preset == "dom_default":
    raise ValueError("Dom default resolves through the legacy controller path")
  if preset == "custom":
    return list(validated["curve"])
  if category == "acceleration":
    return _acceleration_preset_curve(preset, ev_tuning, truck_tuning)
  if category == "braking":
    return list(_BRAKING_PRESET_CURVES[preset])
  return list(FOLLOWING_PRESET_CURVES[preset])


def initial_custom_curve(
  category: str,
  current_config: dict,
  ev_tuning: bool,
  truck_tuning: bool,
  *,
  legacy_curve: list[float] | None = None,
) -> list[float]:
  if category not in _CATEGORY_SPECS or not isinstance(current_config, dict):
    raise ValueError("Unknown or malformed profile category")
  preset = current_config.get("preset")
  if preset == "dom_default":
    candidate = legacy_curve
  elif preset == "custom":
    candidate = current_config.get("curve")
  elif category == "acceleration" and preset in _ACCELERATION_PROFILE_IDS:
    candidate = _acceleration_preset_curve(preset, ev_tuning, truck_tuning)
  elif category == "braking" and preset in _BRAKING_PRESET_CURVES:
    candidate = list(_BRAKING_PRESET_CURVES[preset])
  elif category == "following" and preset in FOLLOWING_PRESET_CURVES:
    candidate = list(FOLLOWING_PRESET_CURVES[preset])
  else:
    candidate = None
  validated = _validated_category(category, {"preset": "custom", "curve": candidate})
  if validated is None:
    raise ValueError(f"Cannot initialize Custom {category} from the current selection")
  return validated["curve"]


def _linear_interp(value: float, breakpoints: tuple[float, ...], values: list[float]) -> float:
  if value <= breakpoints[0]:
    return float(values[0])
  if value >= breakpoints[-1]:
    return float(values[-1])
  index = next(index for index, point in enumerate(breakpoints[1:], start=1) if point >= value) - 1
  t = (value - breakpoints[index]) / float(breakpoints[index + 1] - breakpoints[index])
  return float(values[index] + t * (values[index + 1] - values[index]))


def interpolate_category_curve(
  category: str, v_ego: float, config: dict, ev_tuning: bool, truck_tuning: bool = False,
) -> float:
  if not isinstance(v_ego, numbers.Real) or isinstance(v_ego, bool) or not math.isfinite(float(v_ego)):
    raise ValueError("Vehicle speed must be finite")
  values = category_curve(category, config, ev_tuning, truck_tuning)
  if category == "following":
    return _linear_interp(float(v_ego) / 0.44704, FOLLOWING_SPEEDS_MPH, values)
  return _linear_interp(float(v_ego), _SPEEDS_MS, values)
