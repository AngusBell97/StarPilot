import ast
import subprocess
import sys
from pathlib import Path

from openpilot.starpilot.common.accel_profile import (
  ACCELERATION_PROFILES,
  get_accel_profile_curve_values,
)
from openpilot.starpilot.common.longitudinal_personality_profiles import (
  PERSONALITY_IDS,
  PERSONALITY_PROFILES_PARAM,
  active_personality_id,
  category_curve,
  default_personality_profiles,
  interpolate_category_curve,
  load_personality_profiles,
  resolve_personality_profile,
  serialize_personality_profiles,
  strict_personality_profiles,
  update_personality_profile,
)


def test_profile_model_import_does_not_depend_on_controls_accel_module():
  root = Path(__file__).resolve().parents[3]
  script = """
import sys
from types import ModuleType
sys.modules['openpilot.starpilot.common.accel_profile'] = ModuleType('accel_profile')
import openpilot.starpilot.common.longitudinal_personality_profiles
"""
  result = subprocess.run([sys.executable, "-c", script], cwd=root, text=True, capture_output=True, check=False)

  assert result.returncode == 0, result.stderr


def test_personality_acceleration_presets_match_dom_curves():
  profile_ids = {
    "standard": ACCELERATION_PROFILES["STANDARD"],
    "eco": ACCELERATION_PROFILES["ECO"],
    "sport": ACCELERATION_PROFILES["SPORT"],
    "sport_plus": ACCELERATION_PROFILES["SPORT_PLUS"],
  }
  for ev_tuning in (False, True):
    for preset, profile_id in profile_ids.items():
      config = {"preset": preset, "curve": [0.0] * 7}
      assert category_curve("acceleration", config, ev_tuning) == get_accel_profile_curve_values(profile_id, ev_tuning, False)


def test_default_personality_profiles_match_approved_ui():
  profiles = default_personality_profiles(ev_tuning=True)

  assert tuple(profiles) == PERSONALITY_IDS
  assert profiles["traffic"]["acceleration"]["preset"] == "eco"
  assert profiles["traffic"]["braking"]["preset"] == "standard"
  assert profiles["traffic"]["following"]["preset"] == "close"
  assert profiles["aggressive"]["acceleration"]["preset"] == "sport_plus"
  assert profiles["aggressive"]["braking"]["preset"] == "sport"
  assert profiles["aggressive"]["following"]["preset"] == "close"
  assert profiles["standard"]["acceleration"]["preset"] == "standard"
  assert profiles["standard"]["braking"]["preset"] == "standard"
  assert profiles["standard"]["following"]["preset"] == "medium"
  assert profiles["relaxed"]["acceleration"]["preset"] == "eco"
  assert profiles["relaxed"]["braking"]["preset"] == "eco"
  assert profiles["relaxed"]["following"]["preset"] == "far"

  assert len(profiles["traffic"]["acceleration"]["curve"]) == 7
  assert len(profiles["traffic"]["braking"]["curve"]) == 7
  assert len(profiles["traffic"]["following"]["curve"]) == 10


def test_load_personality_profiles_keeps_valid_fields_and_falls_back_per_category():
  raw = {
    "traffic": {
      "acceleration": {"preset": "custom", "curve": [1.0] * 7},
      "braking": {"preset": "custom", "curve": [0.25] * 6},
      "following": {"preset": "custom", "curve": [1.1] * 10},
    },
    "unknown": {"acceleration": {"preset": "sport"}},
  }

  profiles = load_personality_profiles(raw, ev_tuning=False)

  assert profiles["traffic"]["acceleration"] == {"preset": "custom", "curve": [1.0] * 7}
  assert profiles["traffic"]["following"] == {"preset": "custom", "curve": [1.1] * 10}
  assert profiles["traffic"]["braking"] == default_personality_profiles(False)["traffic"]["braking"]
  assert "unknown" not in profiles


def test_load_personality_profiles_rejects_non_finite_and_out_of_range_values():
  raw = {
    "standard": {
      "acceleration": {"preset": "custom", "curve": [float("nan")] * 7},
      "braking": {"preset": "custom", "curve": [2.1] * 7},
      "following": {"preset": "custom", "curve": [0.74] * 10},
    },
  }

  profiles = load_personality_profiles(raw, ev_tuning=True)
  defaults = default_personality_profiles(True)["standard"]

  assert profiles["standard"] == defaults


def test_update_personality_profile_is_atomic_and_strict():
  profiles = default_personality_profiles(ev_tuning=True)
  curve = [1.35 + 0.05 * index for index in range(10)]

  updated = update_personality_profile(profiles, "standard", "following", "custom", curve, ev_tuning=True)

  assert profiles["standard"]["following"]["preset"] == "medium"
  assert updated["standard"]["following"] == {"preset": "custom", "curve": [round(value, 4) for value in curve]}

  preset_updated = update_personality_profile(
    profiles,
    "standard",
    "following",
    "close",
    [2.5] * 10,
    ev_tuning=True,
  )
  assert preset_updated["standard"]["following"] == {
    "preset": "close",
    "curve": [0.9, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3],
  }

  for args in (
    ("unknown", "following", "custom", curve),
    ("standard", "unknown", "custom", curve),
    ("standard", "following", "invalid", curve),
    ("standard", "following", "custom", curve[:-1]),
    ("standard", "following", "custom", [3.1] * 10),
  ):
    try:
      update_personality_profile(profiles, *args, ev_tuning=True)
    except ValueError:
      pass
    else:
      raise AssertionError(f"Invalid update accepted: {args}")


def test_braking_curve_cannot_reduce_authority_below_stock_eco_floor():
  profiles = default_personality_profiles(False)

  try:
    update_personality_profile(profiles, "relaxed", "braking", "custom", [0.49] * 7, False)
  except ValueError:
    pass
  else:
    raise AssertionError("Braking curve accepted a value below the stock Eco floor")


def test_serialized_profiles_round_trip_to_canonical_data():
  profiles = default_personality_profiles(ev_tuning=False)
  encoded = serialize_personality_profiles(profiles, ev_tuning=False)

  assert load_personality_profiles(encoded, ev_tuning=False) == profiles
  assert " " not in encoded


def test_profile_curve_values_are_serialized_without_binary_float_noise():
  profiles = default_personality_profiles(False)
  noisy_curve = [0.9, 0.9 + 0.05, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3]
  updated = update_personality_profile(profiles, "traffic", "following", "custom", noisy_curve, False)

  assert updated["traffic"]["following"]["curve"][1] == 0.95
  assert "0.9500000000000001" not in serialize_personality_profiles(updated, False)


def test_active_personality_id_handles_traffic_and_rejects_invalid_values():
  assert active_personality_id(True, 2) == "traffic"
  assert active_personality_id(False, 0) == "aggressive"
  assert active_personality_id(False, 1) == "standard"
  assert active_personality_id(False, 2) == "relaxed"
  capnp_enum_type = type(
    "_DynamicEnum",
    (),
    {
      "__module__": "capnp.lib.capnp",
      "__init__": lambda self, raw: setattr(self, "raw", raw),
    },
  )
  assert active_personality_id(False, capnp_enum_type(0)) is None
  assert active_personality_id(False, capnp_enum_type(1)) is None
  assert active_personality_id(False, capnp_enum_type(2)) is None
  for invalid in (99, -1, 0.9, True, False, float("nan"), float("inf"), "1", object()):
    assert active_personality_id(False, invalid) is None


def test_active_personality_id_accepts_real_capnp_dynamic_enum():
  from cereal import log

  state = log.SelfdriveState.new_message()
  for name in ("aggressive", "standard", "relaxed"):
    state.personality = name
    assert active_personality_id(False, state.personality) == name


def test_strict_personality_profiles_rejects_missing_malformed_and_partial_data():
  valid = default_personality_profiles(False)
  assert strict_personality_profiles(valid) == valid

  malformed_values = (
    None,
    "",
    "{}",
    "  { }  ",
    "not-json",
    {},
    {"standard": valid["standard"]},
  )
  for malformed in malformed_values:
    assert strict_personality_profiles(malformed) is None

  partial = default_personality_profiles(False)
  del partial["standard"]["following"]
  assert strict_personality_profiles(partial) is None

  invalid = default_personality_profiles(False)
  invalid["aggressive"]["following"]["curve"][0] = 0.5
  assert strict_personality_profiles(invalid) is None

  string_encoded_number = default_personality_profiles(False)
  string_encoded_number["standard"]["acceleration"]["curve"][0] = "1.0"
  assert strict_personality_profiles(string_encoded_number) is None


def test_resolve_personality_profile_revalidates_control_boundary():
  valid = default_personality_profiles(False)
  assert resolve_personality_profile(valid, False, 0) == valid["aggressive"]
  assert resolve_personality_profile(valid, True, 2) == valid["traffic"]

  malformed = default_personality_profiles(False)
  malformed["standard"]["acceleration"]["curve"] = [float("nan")] * 7
  assert resolve_personality_profile(malformed, False, 1) is None
  assert resolve_personality_profile(valid, False, 0.9) is None


def test_profiles_are_excluded_from_generic_remote_toggle_sync():
  root = Path(__file__).resolve().parents[3]
  tree = ast.parse((root / "starpilot/common/starpilot_variables.py").read_text(encoding="utf-8"))
  assignment = next(
    node for node in tree.body
    if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "EXCLUDED_KEYS" for target in node.targets)
  )

  assert PERSONALITY_PROFILES_PARAM in ast.literal_eval(assignment.value)


def test_category_curve_uses_preset_or_custom_values():
  profiles = default_personality_profiles(ev_tuning=False)
  following = profiles["standard"]["following"]
  following["curve"] = [2.5] * 10

  assert category_curve("following", following, ev_tuning=False)[0] == 1.20
  following["preset"] = "custom"
  assert category_curve("following", following, ev_tuning=False) == [2.5] * 10


def test_following_curve_interpolates_at_ten_mph_breakpoints():
  config = {"preset": "custom", "curve": [1.0 + 0.1 * index for index in range(10)]}

  assert interpolate_category_curve("following", 0.0, config, ev_tuning=False) == 1.0
  assert interpolate_category_curve("following", 5.0 * 0.44704, config, ev_tuning=False) == 1.05
  assert interpolate_category_curve("following", 90.0 * 0.44704, config, ev_tuning=False) == 1.9
