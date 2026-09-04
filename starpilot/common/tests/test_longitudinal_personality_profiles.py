import json
import math
from pathlib import Path

import pytest

from openpilot.starpilot.common.accel_profile import (
  ACCELERATION_PROFILES,
  get_accel_profile_curve_values,
)
from openpilot.starpilot.common.longitudinal_personality_profiles import (
  ACCELERATION_SPEEDS_MPH,
  BRAKING_SPEEDS_MPH,
  FOLLOWING_PRESET_CURVES,
  FOLLOWING_SPEEDS_MPH,
  PERSONALITY_IDS,
  PROFILE_SCHEMA_VERSION,
  active_personality_id,
  category_curve,
  default_personality_profiles,
  initial_custom_curve,
  is_truck_fingerprint,
  interpolate_category_curve,
  load_personality_profiles,
  profile_document,
  resolve_personality_profile,
  serialize_personality_profiles,
  strict_personality_profiles,
  update_personality_profile,
)


def test_document_is_versioned_disabled_and_declares_exact_axes_and_units():
  document = profile_document(default_personality_profiles(False), enabled=False)

  assert document["schemaVersion"] == PROFILE_SCHEMA_VERSION == 1
  assert document["enabled"] is False
  assert document["axes"] == {
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
  assert set(document["profiles"]) == set(PERSONALITY_IDS)
  for profile in document["profiles"].values():
    assert set(profile) == {"acceleration", "braking", "following"}


@pytest.mark.parametrize("fingerprint", [
  "RAM 1500 5TH GEN",
  "RAM HD 5TH GEN",
  "FORD F-150 14TH GEN",
  "FORD MAVERICK 1ST GEN",
  "FORD RANGER 2ND GEN",
  "CHEVROLET SILVERADO 1500 2020",
  "HONDA RIDGELINE 2017",
  "HYUNDAI SANTA CRUZ 2025",
])
def test_supported_truck_fingerprints_select_the_truck_curve(fingerprint):
  assert is_truck_fingerprint(fingerprint) is True


@pytest.mark.parametrize("fingerprint", [None, "", "HONDA CIVIC 2022", "FORD EXPLORER 6TH GEN"])
def test_non_truck_fingerprints_do_not_select_the_truck_curve(fingerprint):
  assert is_truck_fingerprint(fingerprint) is False


def test_runtime_loader_uses_detected_truck_curve_without_changing_legacy_truck_flag():
  source = (Path(__file__).parents[1] / "starpilot_variables.py").read_text(encoding="utf-8")
  assert "is_truck_fingerprint(CP.carFingerprint) or truck_tuning_param" in source
  assert ") and not toggle.personality_ev_tuning" in source
  assert "toggle.truck_tuning = truck_tuning_param" in source


def test_acceleration_presets_select_truck_automatically_and_ev_wins_if_both_are_true():
  config = {"preset": "sport", "curve": []}
  assert category_curve("acceleration", config, False, True) == get_accel_profile_curve_values(2, False, True)
  assert category_curve("acceleration", config, True, True) == get_accel_profile_curve_values(2, True, False)


def test_declared_mph_axis_matches_the_runtime_metre_per_second_breakpoints():
  expected_mph = [speed_mps * 2.2369362920544 for speed_mps in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 40.0)]
  assert list(ACCELERATION_SPEEDS_MPH) == pytest.approx(expected_mph)
  assert BRAKING_SPEEDS_MPH == ACCELERATION_SPEEDS_MPH


def test_strict_document_rejects_unversioned_partial_extra_or_axis_changes():
  valid = profile_document(default_personality_profiles(False), enabled=True)
  assert strict_personality_profiles(valid) == valid["profiles"]
  assert strict_personality_profiles(json.dumps(valid)) == valid["profiles"]

  invalid_documents = [
    valid["profiles"],
    {**valid, "schemaVersion": 2},
    {**valid, "enabled": 1},
    {**valid, "extra": True},
    {key: value for key, value in valid.items() if key != "axes"},
  ]
  wrong_axis = json.loads(json.dumps(valid))
  wrong_axis["axes"]["acceleration"]["speed"]["values"][0] = 1
  invalid_documents.append(wrong_axis)
  partial = json.loads(json.dumps(valid))
  del partial["profiles"]["standard"]["braking"]
  invalid_documents.append(partial)

  for invalid in invalid_documents:
    assert strict_personality_profiles(invalid) is None


def test_strict_document_rejects_boolean_non_finite_fractional_and_out_of_range_values():
  for value in (True, False, math.nan, math.inf, -math.inf, "1.0", 6.1):
    invalid = profile_document(default_personality_profiles(False), enabled=True)
    invalid["profiles"]["standard"]["acceleration"] = {"preset": "custom", "curve": [1.0] * 7}
    invalid["profiles"]["standard"]["acceleration"]["curve"][0] = value
    assert strict_personality_profiles(invalid) is None


def test_disabled_document_never_resolves_an_override():
  disabled = profile_document(default_personality_profiles(False), enabled=False)
  for traffic, personality in ((True, 0), (False, 0), (False, 1), (False, 2)):
    assert resolve_personality_profile(disabled, traffic, personality) is None


def test_context_mapping_is_traffic_first_then_cereal_zero_one_two():
  assert active_personality_id(True, 99) == "traffic"
  assert active_personality_id(False, 0) == "aggressive"
  assert active_personality_id(False, 1) == "standard"
  assert active_personality_id(False, 2) == "relaxed"
  for invalid in (-1, 3, 0.5, True, False, math.nan, math.inf, "1", None):
    assert active_personality_id(False, invalid) is None
  for malformed_traffic in (1, 0, "1", "0", "true", "false", None):
    assert active_personality_id(malformed_traffic, 0) is None


def test_enabled_document_resolves_each_profile_and_revalidates_runtime_boundary():
  document = profile_document(default_personality_profiles(False), enabled=True)
  assert resolve_personality_profile(document, True, 2) == document["profiles"]["traffic"]
  assert resolve_personality_profile(document, False, 0) == document["profiles"]["aggressive"]
  assert resolve_personality_profile(document, False, 1) == document["profiles"]["standard"]
  assert resolve_personality_profile(document, False, 2) == document["profiles"]["relaxed"]

  malformed = json.loads(json.dumps(document))
  malformed["profiles"]["standard"]["acceleration"]["curve"] = [math.nan] * 7
  assert resolve_personality_profile(malformed, False, 1) is None
  assert resolve_personality_profile(document, False, 1.0) is None


def test_acceleration_presets_match_dom_curves_for_gas_ev_and_truck():
  profile_ids = {
    "standard": ACCELERATION_PROFILES["STANDARD"],
    "eco": ACCELERATION_PROFILES["ECO"],
    "sport": ACCELERATION_PROFILES["SPORT"],
    "sport_plus": ACCELERATION_PROFILES["SPORT_PLUS"],
  }
  for ev_tuning, truck_tuning in ((False, False), (True, False), (False, True)):
    for preset, profile_id in profile_ids.items():
      config = {"preset": preset, "curve": []}
      assert category_curve("acceleration", config, ev_tuning, truck_tuning) == get_accel_profile_curve_values(
        profile_id, ev_tuning, truck_tuning
      )


def test_custom_initialisation_seeds_from_selected_acceleration_preset():
  current = {"preset": "sport", "curve": [0.0] * 7}
  assert initial_custom_curve("acceleration", current, ev_tuning=False, truck_tuning=False) == \
    get_accel_profile_curve_values(ACCELERATION_PROFILES["SPORT"], False, False)
  assert initial_custom_curve("acceleration", current, ev_tuning=False, truck_tuning=True) == \
    get_accel_profile_curve_values(ACCELERATION_PROFILES["SPORT"], False, True)


def test_custom_initialisation_uses_ev_over_truck_when_both_flags_are_set():
  current = {"preset": "standard", "curve": [0.0] * 7}
  assert initial_custom_curve("acceleration", current, ev_tuning=True, truck_tuning=True) == \
    get_accel_profile_curve_values(ACCELERATION_PROFILES["STANDARD"], True, False)


def test_truck_detection_accepts_live_canonical_fingerprint_identifiers():
  for fingerprint in (
    "RAM_1500_5TH_GEN",
    "RAM_HD_5TH_GEN",
    "FORD_F_150_MK14",
    "FORD_MAVERICK_MK1",
    "FORD_RANGER_MK2",
    "CHEVROLET_SILVERADO",
    "HONDA_RIDGELINE",
    "HYUNDAI_SANTA_CRUZ_2025",
  ):
    assert is_truck_fingerprint(fingerprint), fingerprint

  assert not is_truck_fingerprint("HYUNDAI_SANTA_FE_2022")
  assert not is_truck_fingerprint(None)


def test_custom_initialisation_seeds_braking_from_selected_preset():
  assert initial_custom_curve(
    "braking", {"preset": "eco", "curve": [2.0] * 7}, ev_tuning=True, truck_tuning=True
  ) == [0.5] * 7
  assert initial_custom_curve(
    "braking", {"preset": "sport", "curve": [0.5] * 7}, ev_tuning=False, truck_tuning=False
  ) == [2.0] * 7


def test_dom_default_custom_initialisation_uses_effective_legacy_curve():
  legacy_curve = [1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5]
  current = {"preset": "dom_default", "curve": [0.0] * 7}
  assert initial_custom_curve("acceleration", current, True, True, legacy_curve=legacy_curve) == legacy_curve


def test_existing_custom_curve_is_never_reseeded():
  curve = [1.0 + index * 0.1 for index in range(7)]
  current = {"preset": "custom", "curve": curve}
  assert initial_custom_curve("acceleration", current, True, True) == curve


def test_profile_update_is_atomic_and_accepts_bounded_following_category():
  profiles = default_personality_profiles(True)
  curve = [1.0 + index * 0.1 for index in range(7)]
  updated = update_personality_profile(profiles, "standard", "acceleration", "custom", curve, True, False)
  assert profiles["standard"]["acceleration"]["preset"] == "dom_default"
  assert updated["standard"]["acceleration"] == {"preset": "custom", "curve": curve}

  following = [0.75 + index * 0.1 for index in range(10)]
  updated = update_personality_profile(updated, "standard", "following", "custom", following, True, False)
  assert profiles["standard"]["following"]["preset"] == "dom_default"
  assert updated["standard"]["following"] == {"preset": "custom", "curve": [round(value, 4) for value in following]}
  for invalid in ([0.74] * 10, [3.01] * 10, [math.nan] * 10, [True] * 10, [1.0] * 9):
    with pytest.raises(ValueError):
      update_personality_profile(updated, "standard", "following", "custom", invalid, True, False)


def test_serialization_requires_explicit_enabled_state_and_preserves_it():
  profiles = default_personality_profiles(False)
  with pytest.raises(TypeError):
    serialize_personality_profiles(profiles, False)
  encoded = serialize_personality_profiles(profiles, False, enabled=False)
  document = json.loads(encoded)
  assert document == profile_document(profiles, enabled=False)
  assert strict_personality_profiles(encoded) is None
  assert " " not in encoded


def test_loader_is_ui_only_fallback_and_does_not_partially_repair_persisted_document():
  defaults = default_personality_profiles(False)
  assert load_personality_profiles(None, False) == defaults
  malformed = profile_document(defaults, enabled=True)
  malformed["profiles"]["standard"]["acceleration"] = {"preset": "custom", "curve": [1.0] * 7}
  malformed["profiles"]["standard"]["acceleration"]["curve"][0] = math.nan
  assert load_personality_profiles(malformed, False) == defaults
  assert strict_personality_profiles(malformed) is None


def test_custom_interpolation_matches_legacy_linear_segments_and_clamps_endpoints():
  config = {"preset": "custom", "curve": [1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]}
  assert interpolate_category_curve("acceleration", -1.0, config, False, False) == 1.0
  assert interpolate_category_curve("acceleration", 1.25, config, False, False) == pytest.approx(1.25)
  assert interpolate_category_curve("acceleration", 100.0, config, False, False) == pytest.approx(2.0)


def test_named_presets_are_canonical_without_unused_curve_points():
  profiles = default_personality_profiles(False)
  updated = update_personality_profile(profiles, "traffic", "acceleration", "eco", [], False, False)
  assert updated["traffic"]["acceleration"] == {"preset": "eco", "curve": []}
  document = profile_document(updated, enabled=True)
  assert strict_personality_profiles(document) == updated


def test_following_presets_and_custom_curve_use_exact_ten_mph_linear_axis():
  for preset, curve in FOLLOWING_PRESET_CURVES.items():
    assert category_curve("following", {"preset": preset, "curve": []}, False, False) == list(curve)

  assert FOLLOWING_SPEEDS_MPH == tuple(range(0, 91, 10))
  config = {"preset": "custom", "curve": [0.75 + 0.1 * index for index in range(10)]}
  assert interpolate_category_curve("following", 0.0, config, False, False) == pytest.approx(0.75)
  assert interpolate_category_curve("following", 5.0 * 0.44704, config, False, False) == pytest.approx(0.80)
  assert interpolate_category_curve("following", 90.0 * 0.44704, config, False, False) == pytest.approx(1.65)


def test_following_custom_initialisation_uses_effective_legacy_curve():
  legacy_curve = [1.0 + index * 0.05 for index in range(10)]
  current = {"preset": "dom_default", "curve": []}
  assert initial_custom_curve("following", current, False, False, legacy_curve=legacy_curve) == legacy_curve
