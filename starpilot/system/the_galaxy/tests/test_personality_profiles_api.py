import json

import numpy as np

from openpilot.starpilot.common.accel_profile import A_CRUISE_MAX_BP_CUSTOM, ACCELERATION_PROFILES, get_accel_profile_curve_values
from openpilot.starpilot.common.longitudinal_personality_profiles import (
  FOLLOWING_SPEEDS_MPH,
  PERSONALITY_PROFILES_PARAM,
  PROFILE_SCHEMA_VERSION,
  default_personality_profiles,
  profile_document,
  strict_profile_document,
)
from test_navigation_params import _params_client, the_galaxy


def _client(monkeypatch, values=None, *, ev_tuning=False, truck_tuning=False):
  client, params = _params_client(monkeypatch, values or {"IsOnroad": False}, "tici")
  monkeypatch.setattr(the_galaxy, "_get_detected_ev_tuning", lambda: ev_tuning)
  monkeypatch.setattr(the_galaxy, "_get_detected_truck_tuning", lambda: truck_tuning, raising=False)
  monkeypatch.setattr(the_galaxy, "_safe_params_get_live_raw", lambda key, default=None, block=False: params.values.get(key, default))
  return client, params


def test_get_returns_disabled_dom_defaults_and_explicit_graph_metadata(monkeypatch):
  client, _ = _client(monkeypatch)
  response = client.get("/api/personality_profiles")
  assert response.status_code == 200
  body = response.get_json()
  assert body["schema_version"] == PROFILE_SCHEMA_VERSION
  assert body["configured"] is False
  assert body["profiles"] == default_personality_profiles(False)
  assert set(body["options"]) == {"acceleration", "braking", "following"}
  assert set(body["speed_breakpoints_mph"]) == {"acceleration", "braking", "following"}
  assert body["speed_breakpoints_mph"]["following"] == list(FOLLOWING_SPEEDS_MPH)


def test_first_save_persists_one_atomic_versioned_document_with_other_categories_dom_default(monkeypatch):
  client, params = _client(monkeypatch)
  response = client.put("/api/personality_profiles", json={
    "profile": "standard", "category": "braking", "preset": "sport", "curve": [2.0] * 7,
  })
  assert response.status_code == 200
  stored = params.values[PERSONALITY_PROFILES_PARAM]
  document = strict_profile_document(stored)
  assert document is not None and document["enabled"] is False
  assert document["profiles"]["standard"]["braking"]["preset"] == "sport"
  for profile_id, profile in document["profiles"].items():
    for category, config in profile.items():
      if (profile_id, category) != ("standard", "braking"):
        assert config == {"preset": "dom_default", "curve": []}
  assert len([write for write in params.writes if write[0] == PERSONALITY_PROFILES_PARAM]) == 1


def test_profile_read_modify_write_endpoint_is_serialized():
  source = (the_galaxy.Path(the_galaxy.__file__)).read_text(encoding="utf-8")
  endpoint = source.split('@app.route("/api/personality_profiles"', 1)[1].split('@app.route(', 1)[0]
  assert "@_serialize_personality_profile_writes" in endpoint


def test_selecting_custom_is_seeded_server_side_from_current_ev_preset_with_ev_over_truck(monkeypatch):
  profiles = default_personality_profiles(False)
  profiles["aggressive"]["acceleration"] = {"preset": "sport", "curve": []}
  stored = profile_document(profiles, enabled=True)
  client, params = _client(monkeypatch, {
    "IsOnroad": False, "TruckTuning": True, PERSONALITY_PROFILES_PARAM: stored,
  }, ev_tuning=True)

  response = client.put("/api/personality_profiles", json={
    "profile": "aggressive", "category": "acceleration", "preset": "custom", "curve": [0.0] * 7,
  })

  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["aggressive"]["acceleration"] == {
    "preset": "custom",
    "curve": get_accel_profile_curve_values(ACCELERATION_PROFILES["SPORT"], True, False),
  }


def test_selecting_custom_uses_the_automatically_detected_truck_curve(monkeypatch):
  profiles = default_personality_profiles(False)
  profiles["aggressive"]["acceleration"] = {"preset": "sport", "curve": []}
  client, params = _client(monkeypatch, {
    "IsOnroad": False,
    PERSONALITY_PROFILES_PARAM: profile_document(profiles, enabled=True),
  }, truck_tuning=True)
  assert the_galaxy._get_detected_truck_tuning() is True
  original_initializer = the_galaxy.initial_custom_curve
  observed = {}

  def capture_initializer(category, current_config, ev_tuning, truck_tuning, *, legacy_curve=None):
    curve = original_initializer(category, current_config, ev_tuning, truck_tuning, legacy_curve=legacy_curve)
    observed.update(ev_tuning=ev_tuning, truck_tuning=truck_tuning, curve=curve)
    return curve

  monkeypatch.setattr(the_galaxy, "initial_custom_curve", capture_initializer)
  response = client.put("/api/personality_profiles", json={
    "profile": "aggressive", "category": "acceleration", "preset": "custom", "curve": [],
  })

  assert response.status_code == 200
  assert observed["ev_tuning"] is False
  assert observed["truck_tuning"] is True
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document is not None
  assert document["profiles"]["aggressive"]["acceleration"]["curve"] == observed["curve"]


def test_custom_braking_is_seeded_from_selected_deceleration_preset(monkeypatch):
  profiles = default_personality_profiles(False)
  profiles["relaxed"]["braking"] = {"preset": "eco", "curve": []}
  client, params = _client(monkeypatch, {
    "IsOnroad": False, PERSONALITY_PROFILES_PARAM: profile_document(profiles, enabled=True),
  })
  response = client.put("/api/personality_profiles", json={
    "profile": "relaxed", "category": "braking", "preset": "custom", "curve": [2.0] * 7,
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["relaxed"]["braking"] == {"preset": "custom", "curve": [0.5] * 7}


def test_dom_default_custom_acceleration_seeds_from_effective_legacy_custom_curve(monkeypatch):
  values = {
    "IsOnroad": False,
    "CustomAccelProfile": True,
    "CustomAccelProfileInitialized": True,
    **{
      f"CustomAccelProfile{mph}MPH": value
      for mph, value in zip((0, 11, 22, 34, 45, 56, 89), (1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5), strict=True)
    },
  }
  client, params = _client(monkeypatch, values)
  response = client.put("/api/personality_profiles", json={
    "profile": "traffic", "category": "acceleration", "preset": "custom", "curve": [6.0] * 7,
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["traffic"]["acceleration"]["curve"] == [1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5]


def test_existing_custom_category_persists_subsequent_graph_edits_exactly(monkeypatch):
  profiles = default_personality_profiles(False)
  profiles["standard"]["acceleration"] = {"preset": "custom", "curve": [1.0] * 7}
  client, params = _client(monkeypatch, {
    "IsOnroad": False, PERSONALITY_PROFILES_PARAM: profile_document(profiles, enabled=True),
  })
  edited = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]
  response = client.put("/api/personality_profiles", json={
    "profile": "standard", "category": "acceleration", "preset": "custom", "curve": edited,
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["standard"]["acceleration"]["curve"] == edited


def test_dom_default_custom_seed_resamples_valid_dynamic_curve_and_malformed_dynamic_falls_back(monkeypatch):
  dynamic = {
    "IsOnroad": False,
    "CustomAccelProfile": True,
    "CustomAccelProfileBreakpointsInitialized": True,
    "CustomAccelProfilePointCount": 3,
    "CustomAccelProfileBreakpoint1MPH": 0,
    "CustomAccelProfileBreakpoint2MPH": 40,
    "CustomAccelProfileBreakpoint3MPH": 90,
    "CustomAccelProfilePoint1Accel": 1.0,
    "CustomAccelProfilePoint2Accel": 2.0,
    "CustomAccelProfilePoint3Accel": 3.0,
  }
  client, params = _client(monkeypatch, dynamic)
  response = client.put("/api/personality_profiles", json={
    "profile": "standard", "category": "acceleration", "preset": "custom", "curve": [0.0] * 7,
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  dynamic_axis_ms = np.array([0.0, 40.0, 90.0]) * 0.44704
  expected = [round(float(value), 4) for value in np.interp(A_CRUISE_MAX_BP_CUSTOM, dynamic_axis_ms, [1.0, 2.0, 3.0])]
  assert document["profiles"]["standard"]["acceleration"]["curve"] == expected

  malformed_client, malformed_params = _client(monkeypatch, {
    **dynamic, "CustomAccelProfilePointCount": 3.5, "AccelerationProfile": ACCELERATION_PROFILES["ECO"],
  })
  response = malformed_client.put("/api/personality_profiles", json={
    "profile": "standard", "category": "acceleration", "preset": "custom", "curve": [6.0] * 7,
  })
  assert response.status_code == 200
  document = strict_profile_document(malformed_params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["standard"]["acceleration"]["curve"] == get_accel_profile_curve_values(
    ACCELERATION_PROFILES["ECO"], False, False
  )


def test_following_custom_seeds_from_legacy_profile_and_then_persists_edits(monkeypatch):
  client, params = _client(monkeypatch, {
    "IsOnroad": False,
    "CustomPersonalities": True,
    "StandardFollow": 1.4,
    "StandardFollowHigh": 1.1,
  })
  response = client.put("/api/personality_profiles", json={
    "profile": "standard", "category": "following", "preset": "custom", "curve": [3.0] * 10,
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  expected = [round(float(value), 4) for value in np.interp(FOLLOWING_SPEEDS_MPH, [45.0, 70.0], [1.4, 1.1])]
  assert document["profiles"]["standard"]["following"] == {"preset": "custom", "curve": expected}

  edited = [0.75 + index * 0.1 for index in range(10)]
  response = client.put("/api/personality_profiles", json={
    "profile": "standard", "category": "following", "preset": "custom", "curve": edited,
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["standard"]["following"]["curve"] == [round(value, 4) for value in edited]


def test_following_custom_preserves_builtin_when_legacy_custom_personalities_is_off(monkeypatch):
  client, params = _client(monkeypatch, {
    "IsOnroad": False,
    "CustomPersonalities": False,
    "RelaxedFollow": 1.1,
    "RelaxedFollowHigh": 0.9,
  })
  response = client.put("/api/personality_profiles", json={
    "profile": "relaxed", "category": "following", "preset": "custom", "curve": [],
  })
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document["profiles"]["relaxed"]["following"]["curve"] == [1.75] * len(FOLLOWING_SPEEDS_MPH)


def test_traffic_following_seed_matches_legacy_runtime_speed_units(monkeypatch):
  client, params = _client(monkeypatch, {
    "IsOnroad": False,
    "TrafficFollow": 0.8,
    "RelaxedFollow": 1.6,
  })
  response = client.put("/api/personality_profiles", json={
    "profile": "traffic", "category": "following", "preset": "custom", "curve": [],
  })

  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  expected = [
    round(float(value), 4)
    for value in np.interp(np.array(FOLLOWING_SPEEDS_MPH) * 0.44704, [0.0, 25.0], [0.8, 1.6])
  ]
  assert document["profiles"]["traffic"]["following"]["curve"] == expected


def test_invalid_payload_never_writes(monkeypatch):
  client, params = _client(monkeypatch)
  for payload in (
    {"profile": "standard", "category": "braking", "preset": "custom", "curve": [True] * 7},
    {"profile": "standard", "category": "following", "preset": "custom", "curve": [0.74] * 10},
  ):
    response = client.put("/api/personality_profiles", json=payload)
    assert response.status_code == 400
  assert PERSONALITY_PROFILES_PARAM not in params.values


def test_dedicated_and_generic_profile_mutations_are_blocked_onroad_with_unchanged_value(monkeypatch):
  original = profile_document(default_personality_profiles(False), enabled=False)
  client, params = _client(monkeypatch, {"IsOnroad": True, PERSONALITY_PROFILES_PARAM: original})
  before = json.loads(json.dumps(params.values))

  dedicated = client.put("/api/personality_profiles", json={
    "profile": "traffic", "category": "acceleration", "preset": "eco", "curve": [1.0] * 7,
  })
  generic = client.put("/api/params", json={"key": PERSONALITY_PROFILES_PARAM, "value": {"enabled": True}})
  legacy_parent = client.put("/api/params", json={"key": "CustomPersonalities", "value": True})

  assert dedicated.status_code == 403
  assert generic.status_code == 403
  assert legacy_parent.status_code == 403
  assert "parked" in legacy_parent.get_json()["error"].lower()
  assert params.values == before


def test_generic_profile_mutation_is_also_rejected_while_parked(monkeypatch):
  client, params = _client(monkeypatch, {"IsOnroad": False})
  response = client.put("/api/params", json={"key": PERSONALITY_PROFILES_PARAM, "value": {"enabled": True}})
  assert response.status_code == 403
  assert PERSONALITY_PROFILES_PARAM not in params.values


def test_enabling_requires_explicit_parked_boolean_mutation(monkeypatch):
  client, params = _client(monkeypatch, {"IsOnroad": False})
  assert client.put("/api/personality_profiles", json={"enabled": 1}).status_code == 400
  response = client.put("/api/personality_profiles", json={"enabled": True})
  assert response.status_code == 200
  document = strict_profile_document(params.values[PERSONALITY_PROFILES_PARAM])
  assert document is not None and document["enabled"] is True


def test_malformed_stored_document_reads_as_unconfigured_dom_defaults(monkeypatch):
  client, _ = _client(monkeypatch, {"IsOnroad": False, PERSONALITY_PROFILES_PARAM: {"schemaVersion": 99}})
  response = client.get("/api/personality_profiles")
  assert response.status_code == 200
  assert response.get_json()["configured"] is False
  assert response.get_json()["profiles"] == default_personality_profiles(False)
