import threading
import time

from test_navigation_params import _params_client, the_galaxy

from openpilot.starpilot.common.longitudinal_personality_profiles import (
  PERSONALITY_PROFILES_PARAM,
  default_personality_profiles,
)


def _client(monkeypatch, values=None):
  client, params = _params_client(monkeypatch, values or {"IsOnroad": False}, "tici")
  monkeypatch.setattr(the_galaxy, "_get_detected_ev_tuning", lambda: False)
  monkeypatch.setattr(the_galaxy, "_safe_params_get_live_raw", lambda key, default=None: params.values.get(key, default))
  return client, params


def test_get_personality_profiles_returns_defaults_and_graph_metadata(monkeypatch):
  client, _ = _client(monkeypatch)

  response = client.get("/api/personality_profiles")

  assert response.status_code == 200
  body = response.get_json()
  assert body["profiles"] == default_personality_profiles(False)
  assert body["configured"] is False
  assert body["default_profiles"] == default_personality_profiles(False)
  assert body["speed_breakpoints_mph"] == {
    "acceleration": [0, 11, 22, 34, 45, 56, 89],
    "braking": [0, 11, 22, 34, 45, 56, 89],
    "following": list(range(0, 91, 10)),
  }
  assert body["bounds"]["following"] == [0.75, 3.0]


def test_get_malformed_personality_profiles_preserves_unconfigured_fallback(monkeypatch):
  client, _ = _client(monkeypatch, {
    "IsOnroad": False,
    PERSONALITY_PROFILES_PARAM: "  { }  ",
  })

  response = client.get("/api/personality_profiles")

  assert response.status_code == 200
  assert response.get_json()["configured"] is False
  assert response.get_json()["profiles"] == default_personality_profiles(False)


def test_put_personality_profile_validates_and_persists_atomically(monkeypatch):
  client, params = _client(monkeypatch)
  curve = [1.0 + index * 0.1 for index in range(10)]

  response = client.put("/api/personality_profiles", json={
    "profile": "standard",
    "category": "following",
    "preset": "custom",
    "curve": curve,
  })

  assert response.status_code == 200
  assert response.get_json()["configured"] is True
  stored_value = params.values[PERSONALITY_PROFILES_PARAM]
  assert isinstance(stored_value, dict)
  stored = stored_value
  assert stored["standard"]["following"] == {"preset": "custom", "curve": [round(value, 4) for value in curve]}
  assert len([write for write in params.writes if write[0] == PERSONALITY_PROFILES_PARAM]) == 1

  reread = client.get("/api/personality_profiles")
  assert reread.status_code == 200
  assert reread.get_json()["configured"] is True
  assert reread.get_json()["profiles"] == stored


def test_concurrent_profile_updates_preserve_both_edits(monkeypatch):
  client, params = _client(monkeypatch)
  first_put_entered = threading.Event()
  release_first_put = threading.Event()
  original_put = params.put
  delayed_once = False

  def delayed_put(key, value):
    nonlocal delayed_once
    if key == PERSONALITY_PROFILES_PARAM and not delayed_once:
      delayed_once = True
      first_put_entered.set()
      release_first_put.wait(timeout=1.0)
    original_put(key, value)

  monkeypatch.setattr(params, "put", delayed_put)
  responses = []

  def save(payload):
    with client.application.test_client() as thread_client:
      responses.append(thread_client.put("/api/personality_profiles", json=payload))

  first = threading.Thread(target=save, args=({
    "profile": "standard",
    "category": "following",
    "preset": "custom",
    "curve": [1.4] * 10,
  },))
  second = threading.Thread(target=save, args=({
    "profile": "aggressive",
    "category": "braking",
    "preset": "custom",
    "curve": [1.1] * 7,
  },))

  first.start()
  assert first_put_entered.wait(timeout=1.0)
  second.start()
  time.sleep(0.05)
  release_first_put.set()
  first.join(timeout=1.0)
  second.join(timeout=1.0)

  assert not first.is_alive()
  assert not second.is_alive()
  assert sorted(response.status_code for response in responses) == [200, 200]
  stored = params.values[PERSONALITY_PROFILES_PARAM]
  assert stored["standard"]["following"] == {"preset": "custom", "curve": [1.4] * 10}
  assert stored["aggressive"]["braking"] == {"preset": "custom", "curve": [1.1] * 7}


def test_put_personality_profile_rejects_invalid_curve_without_writing(monkeypatch):
  client, params = _client(monkeypatch)

  response = client.put("/api/personality_profiles", json={
    "profile": "standard",
    "category": "following",
    "preset": "custom",
    "curve": [3.1] * 10,
  })

  assert response.status_code == 400
  assert PERSONALITY_PROFILES_PARAM not in params.values


def test_put_personality_profile_rejects_non_string_identifiers_without_writing(monkeypatch):
  client, params = _client(monkeypatch)
  valid_payload = {
    "profile": "standard",
    "category": "following",
    "preset": "custom",
    "curve": [1.2] * 10,
  }

  for field in ("profile", "category", "preset"):
    for invalid in ([], {}, None, True):
      payload = dict(valid_payload)
      payload[field] = invalid

      response = client.put("/api/personality_profiles", json=payload)

      assert response.status_code == 400
      assert PERSONALITY_PROFILES_PARAM not in params.values


def test_generic_params_api_cannot_write_personality_profiles(monkeypatch):
  for is_onroad in (False, True):
    client, params = _client(monkeypatch, {"IsOnroad": is_onroad})
    before = dict(params.values)

    response = client.put("/api/params", json={
      "key": PERSONALITY_PROFILES_PARAM,
      "value": default_personality_profiles(False),
    })

    assert response.status_code == 403
    assert params.values == before


def test_toggle_backup_restore_cannot_write_personality_profiles(monkeypatch):
  client, params = _client(monkeypatch)
  monkeypatch.setattr(the_galaxy, "EXCLUDED_KEYS", set(the_galaxy.EXCLUDED_KEYS) | {PERSONALITY_PROFILES_PARAM})
  allowed_keys = the_galaxy._get_toggle_backup_keys()
  assert PERSONALITY_PROFILES_PARAM not in allowed_keys
  monkeypatch.setattr(the_galaxy, "_get_toggle_backup_keys", lambda: allowed_keys)
  monkeypatch.setattr(the_galaxy, "_params_raw", params)
  monkeypatch.setattr(
    the_galaxy.utilities,
    "decode_parameters",
    lambda _encoded: {PERSONALITY_PROFILES_PARAM: default_personality_profiles(False)},
  )

  response = client.post("/api/toggles/restore", json={
    "format": the_galaxy.TOGGLE_BACKUP_FORMAT,
    "version": the_galaxy.TOGGLE_BACKUP_VERSION,
    "data": "valid-encoded-data",
  })

  assert response.status_code == 400
  assert PERSONALITY_PROFILES_PARAM not in params.values


def test_put_personality_profile_is_blocked_onroad(monkeypatch):
  client, params = _client(monkeypatch, {"IsOnroad": True})

  response = client.put("/api/personality_profiles", json={
    "profile": "traffic",
    "category": "acceleration",
    "preset": "eco",
    "curve": [1.0] * 7,
  })

  assert response.status_code == 403
  assert PERSONALITY_PROFILES_PARAM not in params.values
