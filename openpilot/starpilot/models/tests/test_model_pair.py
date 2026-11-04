import json
import time
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.starpilot.models import model_pair
from openpilot.starpilot.models.laboratory_outputs import LATERAL_PLAN_COLUMNS
from openpilot.starpilot.models.model_pair import ModelPair, PairGuard, PairPlan, plan_pair, read_status, setup_pair, write_status


def outputs(value: float) -> dict:
  return {"plan": np.full((33, 15), value, dtype=np.float32), "lane_lines": np.full((4, 33, 2), value, dtype=np.float32),
          "lead": np.full((3, 6, 4), value, dtype=np.float32), "desire_state": np.full((1, 8), value, dtype=np.float32),
          "pose": np.full((1, 12), value, dtype=np.float32), "meta": np.full((1, 55), value, dtype=np.float32)}


class FakeModel:
  def __init__(self, model_id, value, *, chestnut, events, descriptor="warp", cost=0.0, clock=None, fail=False):
    self.model_id, self.value, self.chestnut, self.events = model_id, value, chestnut, events
    self.camera_warp_descriptor = descriptor
    self.cost, self.clock, self.fail = cost, clock, fail
    self.npy = {"desire": np.zeros(8)}
    self.last_warp = None
    self.shared = []

  def run(self, bufs, transforms, inputs, after_enqueue=None, *, shared_warp=None):
    self.events.append(f"{self.model_id}:start")
    if self.fail:
      raise RuntimeError("boom")
    self.shared.append(shared_warp)
    if after_enqueue is not None:
      after_enqueue()
    if self.clock is not None:
      self.clock.t += self.cost
    self.last_warp = object()
    self.events.append(f"{self.model_id}:end")
    return outputs(self.value)

  def get_action_from_model(self, out, prev, lat_t, long_t, v_ego):
    v = float(out["plan"][0, 0])
    return log.ModelDataV2.Action(desiredCurvature=v, desiredAcceleration=10 * v, shouldStop=False)


class Clock:
  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t


def make_pair(tmp_path, *, mode="shadow", partner_chestnut=False, role="longitudinal", **partner_kwargs):
  events, clock = [], Clock()
  primary = FakeModel("lightspeed", 1.0, chestnut=True, events=events, cost=0.040, clock=clock)
  partner = FakeModel("rdf43", 2.0, chestnut=partner_chestnut, events=events, clock=clock, **partner_kwargs)
  lateral, longitudinal = ("lightspeed", "rdf43") if role == "longitudinal" else ("rdf43", "lightspeed")
  plan = PairPlan(mode, lateral, longitudinal, "rdf43", role)
  return ModelPair(primary, partner, plan, now=clock, status_path=tmp_path / "pair.json"), primary, partner, events, clock


def step(pair, drop=0.0):
  pair.frame_drop_ratio = drop
  out = pair.run({}, {}, {}, None)
  return out, pair.get_action_from_model(out, log.ModelDataV2.Action(), 0.3, 0.5, 20.0)


def test_plan_pair_needs_the_running_model():
  assert plan_pair({"enabled": False}, "lightspeed") == (None, "off")
  cfg = {"enabled": True, "lateralModel": "lightspeed", "longitudinalModel": "rdf43", "mode": "drive"}
  plan, reason = plan_pair(cfg, "lightspeed")
  assert reason == "" and plan.partner_id == "rdf43" and plan.partner_role == "longitudinal" and plan.mode == "drive"
  plan, _ = plan_pair({**cfg, "lateralModel": "rdf43", "longitudinalModel": "lightspeed"}, "lightspeed")
  assert plan.partner_role == "lateral"
  assert plan_pair({**cfg, "longitudinalModel": "lightspeed"}, "lightspeed")[0] is None
  plan, reason = plan_pair({**cfg, "lateralModel": "resfit"}, "lightspeed")
  assert plan is None and "must include the model that is running (lightspeed)" in reason
  assert plan_pair({**cfg, "mode": "fast"}, "lightspeed")[0] is None
  assert plan_pair({k: v for k, v in cfg.items() if k != "mode"}, "lightspeed")[0].mode == "drive"


def test_pair_selection_loads_the_pairs_own_models():
  from openpilot.starpilot.models.model_pair import pair_kind_error, pair_selection
  entries = {"lightspeed": {"uses_external_gpu": True}, "resfit": {"uses_external_gpu": True}, "rdf43": {}, "tr16": {}}
  on = {"enabled": True, "mode": "drive"}
  assert pair_selection({**on, "lateralModel": "rdf43", "longitudinalModel": "lightspeed"}, entries) == {"big": "lightspeed", "small": "rdf43"}
  assert pair_selection({**on, "lateralModel": "tr16", "longitudinalModel": "rdf43"}, entries) == {"big": "", "small": "tr16"}
  assert pair_selection({**on, "lateralModel": "resfit", "longitudinalModel": "lightspeed"}, entries) is None
  assert pair_selection({**on, "enabled": False, "lateralModel": "tr16", "longitudinalModel": "rdf43"}, entries) is None
  assert pair_selection({**on, "lateralModel": "gone", "longitudinalModel": "rdf43"}, entries) is None
  assert pair_kind_error(True, True) and not pair_kind_error(True, False) and not pair_kind_error(False, False)


def test_shadow_drives_with_the_primary_and_runs_a_comma_partner_while_the_chestnut_computes(tmp_path):
  pair, primary, partner, events, _ = make_pair(tmp_path)
  out, action = step(pair)
  assert events == ["lightspeed:start", "rdf43:start", "rdf43:end", "lightspeed:end"]
  assert float(out["plan"][0, 0]) == 1.0 and action.desiredCurvature == 1.0
  assert pair.summary()["curvatureDiffP50"] == 1.0 and pair.summary()["accelDiffP50"] == 10.0
  assert pair.chestnut and pair.model_id == "lightspeed" and pair.npy is primary.npy


def test_chestnut_partner_runs_after_the_primary_and_shares_its_warp(tmp_path):
  pair, primary, partner, events, _ = make_pair(tmp_path, partner_chestnut=True)
  step(pair)
  assert events == ["lightspeed:start", "lightspeed:end", "rdf43:start", "rdf43:end"]
  assert pair.share_warp and partner.shared[-1] is primary.last_warp
  other, *_ = make_pair(tmp_path, partner_chestnut=True, descriptor="other")
  assert not other.share_warp


def test_drive_mode_combines_lateral_and_longitudinal(tmp_path):
  pair, *_ = make_pair(tmp_path, mode="drive", role="longitudinal")
  out, action = step(pair)
  cols = list(LATERAL_PLAN_COLUMNS)
  assert np.all(out["plan"][:, cols] == 1.0) and np.all(np.delete(out["plan"], cols, axis=1) == 2.0)
  assert np.all(out["lane_lines"] == 1.0) and np.all(out["lead"] == 2.0)
  assert action.desiredCurvature == 1.0 and action.desiredAcceleration == 20.0
  pair, *_ = make_pair(tmp_path, mode="drive", role="lateral")
  out, action = step(pair)
  assert action.desiredCurvature == 2.0 and action.desiredAcceleration == 10.0 and np.all(out["lead"] == 1.0)


def test_partner_failure_leaves_the_primary_driving(tmp_path):
  pair, *_ = make_pair(tmp_path, mode="drive", fail=True)
  out, action = step(pair)
  assert not pair.partner_active and "partner failed" in pair.stop_reason
  assert float(out["plan"][0, 0]) == 1.0 and action.desiredAcceleration == 10.0
  status = json.loads((tmp_path / "pair.json").read_text())
  assert status["state"] == "stopped" and "boom" in status["reason"]


def test_guard_stops_a_slow_partner(tmp_path):
  pair, _, partner, _, clock = make_pair(tmp_path, partner_chestnut=True, mode="drive", cost=0.012)
  for _ in range(model_pair.WARMUP_FRAMES + model_pair.SLOW_LIMIT + 1):
    step(pair)
  assert not pair.partner_active and "too slow" in pair.stop_reason
  out, action = step(pair)
  assert float(out["plan"][0, 0]) == 1.0 and action.desiredAcceleration == 10.0


def test_guard_stops_on_dropped_frames_after_warmup():
  guard = PairGuard()
  assert all(guard.check(0.040, 0.5) == "" for _ in range(model_pair.WARMUP_FRAMES))
  assert "frames dropped" in guard.check(0.040, 0.05)
  assert PairGuard().check(0.040, 0.0) == ""


def test_status_round_trip_and_staleness(tmp_path):
  path = tmp_path / "status.json"
  write_status({"state": "testing"}, path)
  assert read_status(path)["fresh"] is True
  data = json.loads(path.read_text())
  data["wallTime"] = time.time() - 60
  path.write_text(json.dumps(data))
  assert read_status(path)["fresh"] is False
  assert read_status(tmp_path / "missing.json") is None


def test_setup_pair_reuses_the_loaded_small_model_and_never_fails_the_drive(tmp_path, monkeypatch):
  from openpilot.starpilot.models import runner
  monkeypatch.setattr(model_pair, "STATUS_PATH", tmp_path / "status.json")

  class Catalog(FakeModel, runner.CatalogModelState):
    def __init__(self, *args, **kwargs):
      FakeModel.__init__(self, *args, **kwargs)

  events = []
  primary = Catalog("lightspeed", 1.0, chestnut=True, events=events)
  small = Catalog("rdf43", 2.0, chestnut=False, events=events)
  cfg = {"enabled": True, "lateralModel": "lightspeed", "longitudinalModel": "rdf43", "mode": "shadow"}
  model, reason = setup_pair(primary, small, 1928, 1208, config=cfg, load=lambda *a: pytest.fail("must reuse the small model"))
  assert reason == "" and isinstance(model, ModelPair) and model.partner is small and model.partner_device == "comma"

  model, reason = setup_pair(primary, small, 1928, 1208, config={**cfg, "lateralModel": "resfit"})
  assert model is primary and "must include" in reason
  assert read_status(tmp_path / "status.json")["state"] == "refused"

  monkeypatch.setattr(model_pair, "partner_artifact", lambda mid, chestnut_ok=True: (tmp_path / "x.pkl", "v16", "0" * 64, True))
  def broken(*args):
    raise RuntimeError("no memory")
  model, reason = setup_pair(primary, small, 1928, 1208, config={**cfg, "longitudinalModel": "resfit"}, load=broken)
  assert model is primary and "no memory" in reason

  loaded = SimpleNamespace()
  def load(cam_w, cam_h, path, version, chestnut, mid, sha):
    loaded.args = (version, chestnut, mid)
    return Catalog(mid, 3.0, chestnut=chestnut, events=events), None
  model, reason = setup_pair(primary, small, 1928, 1208, config={**cfg, "longitudinalModel": "resfit"}, load=load)
  assert reason == "" and model.partner.model_id == "resfit" and loaded.args == ("v16", True, "resfit") and model.partner_device == "chestnut"

  model, reason = setup_pair(primary, small, 1928, 1208, config={**cfg, "enabled": False})
  assert model is primary and reason == "off"

  # without the Chestnut running, a big partner never loads
  small_primary = Catalog("rdf43", 2.0, chestnut=False, events=events)
  model, reason = setup_pair(small_primary, small_primary, 1928, 1208, config={**cfg, "lateralModel": "rdf43", "longitudinalModel": "resfit"},
                             load=lambda *a: pytest.fail("must not load a big model without the Chestnut"))
  assert model is small_primary and "Chestnut is not running" in reason


def test_called_exactly_as_modeld_calls_the_driving_model(tmp_path):
  # modeld: sets lat_delay/frame_drop_ratio, reads npy['desire'] and vision_input_names, passes after_enqueue positionally
  pair, primary, partner, events, _ = make_pair(tmp_path, mode="drive")
  sent = []
  pair.lat_delay, pair.frame_drop_ratio = 0.2, 0.0
  assert pair.npy.get("desire") is primary.npy["desire"] and pair.vision_input_names == ("img", "big_img")
  bufs = {name: object() for name in pair.vision_input_names}
  out = pair.run(bufs, {}, {"desire_pulse": np.zeros(8)}, lambda: sent.append("chestnut state"))
  assert sent == ["chestnut state"] and events.index("rdf43:start") > events.index("lightspeed:start")
  action = pair.get_action_from_model(out, log.ModelDataV2.Action(), 0.3, 0.5, 20.0)
  assert (action.desiredCurvature, action.desiredAcceleration) == (1.0, 20.0) and pair.chestnut is True and pair.model_id == "lightspeed"
  assert getattr(pair, "primary", pair) is primary and partner.lat_delay == 0.2


def test_last_run_summary_survives_power_off(tmp_path, monkeypatch):
  pair, *_ = make_pair(tmp_path)
  step(pair)
  pair.write_status(force=True)
  (tmp_path / "pair.json").unlink()        # /dev/shm is wiped when the comma powers down
  monkeypatch.setattr(model_pair, "STATUS_PATH", tmp_path / "pair.json")
  monkeypatch.setattr(model_pair, "LAST_RUN_PATH", tmp_path / "last_run.json")
  saved = read_status()
  assert saved["state"] == "testing" and saved["frames"] == 1 and saved["fresh"] is True


def test_a_chestnut_build_made_on_this_comma_is_used_only_when_its_digest_matches(tmp_path, monkeypatch):
  import hashlib
  from openpilot.starpilot.models import manager
  from openpilot.starpilot.models.model_pair import local_variant, local_variant_ids, partner_artifact
  amd = tmp_path / "polaris" / "polaris_driving_amd_tinygrad.pkl"
  amd.parent.mkdir()
  amd.write_bytes(b"chestnut build")
  monkeypatch.setattr(manager, "artifact_path", lambda mid, root, variant="standard": amd if variant == "amd" else tmp_path / "x")
  entry = {"artifact_sha256": hashlib.sha256(b"chestnut build").hexdigest(), "artifact_size": 14, "behavior_version": "v15"}
  (tmp_path / "local_variants.json").write_text(json.dumps({"polaris": {"amd": entry}}))
  assert local_variant_ids(tmp_path) == {"polaris"}
  assert local_variant("polaris", tmp_path)["path"] == amd

  rows = {"polaris": {"name": "Polaris", "version": "v15", "artifact_sha256": "s" * 64}}
  monkeypatch.setattr(manager, "catalog", lambda root: rows)
  monkeypatch.setattr(manager, "verified_artifact", lambda *a, **k: tmp_path / "standard.pkl" if len(a) < 4 else None)
  assert partner_artifact("polaris", root=tmp_path) == (amd, "v15", entry["artifact_sha256"], True)
  # without the Chestnut running, or with a changed file, the comma-GPU model is used instead
  assert partner_artifact("polaris", root=tmp_path, chestnut_ok=False)[3] is False
  amd.write_bytes(b"chestnut BUILD")
  assert local_variant("polaris", tmp_path) is None and local_variant("polaris", tmp_path, verify=False) is not None
  assert partner_artifact("polaris", root=tmp_path)[3] is False
  # a build for another model version is never used
  amd.write_bytes(b"chestnut build")
  (tmp_path / "local_variants.json").write_text(json.dumps({"polaris": {"amd": {**entry, "behavior_version": "v16"}}}))
  assert partner_artifact("polaris", root=tmp_path)[3] is False
