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


def test_guard_ignores_slow_warmup_frames():
  guard = PairGuard()
  assert all(guard.check(0.700, 0.0) == "" for _ in range(model_pair.WARMUP_FRAMES))
  assert all(guard.check(0.040, 0.0) == "" for _ in range(model_pair.SLOW_WINDOW))
  assert all(guard.check(0.060, 0.0) == "" for _ in range(model_pair.SLOW_LIMIT))
  assert "too slow" in guard.check(0.060, 0.0)


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
  monkeypatch.setattr(model_pair, "partner_artifact", lambda mid, chestnut_ok=True: (tmp_path / "s.pkl", "v15", "1" * 64, False))
  model, reason = setup_pair(primary, small, 1928, 1208, config=cfg, load=lambda *a: pytest.fail("must reuse the small model"))
  assert reason == "" and isinstance(model, ModelPair) and model.partner is small and model.partner_device == "comma"
  # with a Chestnut build of the same small model, that build loads instead of reusing the comma-GPU copy
  monkeypatch.setattr(model_pair, "partner_artifact", lambda mid, chestnut_ok=True: (tmp_path / "a.pkl", "v15", "2" * 64, chestnut_ok))
  built = []
  def load_chestnut(cam_w, cam_h, path, version, chestnut, mid, sha):
    built.append((path.name, chestnut))
    return Catalog(mid, 2.0, chestnut=chestnut, events=events), None
  model, reason = setup_pair(primary, small, 1928, 1208, config=cfg, load=load_chestnut)
  assert reason == "" and built == [("a.pkl", True)] and model.partner is not small and model.partner_device == "chestnut"

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


def test_a_chestnut_partner_reads_the_image_the_primary_uploaded(tmp_path):
  pair, primary, partner, events, _ = make_pair(tmp_path, partner_chestnut=True)
  image = object()
  primary.input_queues = {"mapped_gpu_image": image}
  seen = []
  partner.accepts_device_image = lambda img: img is image
  partner.run_on_device_image = lambda img, inputs: (seen.append(img), outputs(2.0))[1]
  step(pair)
  assert seen == [image] and pair.partner_image == "primary's Chestnut upload"
  assert "rdf43:start" not in events                      # the partner's own warp and run never happened
  assert json.loads((tmp_path / "pair.json").read_text())["partnerImage"] == "primary's Chestnut upload"
  # an image the partner cannot take (other shape, device or build) falls back to the shared warp
  partner.accepts_device_image = lambda img: False
  step(pair)
  assert pair.partner_image == "shared warp" and "rdf43:start" in events
  # a comma-GPU partner never reads Chestnut memory
  other, p2, s2, *_ = make_pair(tmp_path)
  p2.input_queues = {"mapped_gpu_image": image}
  s2.accepts_device_image = lambda img: True
  s2.run_on_device_image = lambda img, inputs: pytest.fail("must not read Chestnut memory")
  step(other)
  assert other.partner_image == "own warp"


class FakeRemote:
  """A partner process: answers each submitted job by collect time, or not (late), or fails."""
  chestnut = False
  remote = True

  def __init__(self, events, value=2.0, *, answer=True, fail=False, clock=None, ready=lambda seq: 0.039):
    self.model_id, self.events, self.value = "polaris", events, value
    self.answer, self.fail, self.seq, self.closed = answer, fail, 0, False
    self.jobs = []
    self.clock, self.ready, self.queued = clock, ready, []

  def submit(self, frame_ids, transforms, inputs):
    self.seq += 1
    self.jobs.append(frame_ids)
    self.events.append("polaris:job")
    sent = self.clock() if self.clock is not None else 0.0
    self.queued.append((self.seq, sent + self.ready(self.seq)))
    return self.seq

  def drain(self):
    if self.fail:
      raise RuntimeError("partner process ended")
    answers, self.queued = self.queued, []
    return [(seq, outputs(self.value + seq / 1000), {"doneAt": done, "waitMs": 30.0}, "") for seq, done in answers]

  def collect(self, seq, timeout_s):
    self.events.append("polaris:collect")
    if self.fail:
      raise RuntimeError("partner process ended")
    if self.answer is False or (callable(self.answer) and not self.answer(seq)):
      return None, None, "late"
    return outputs(self.value + seq / 1000), 37.0, ""

  def get_action_from_model(self, out, prev, lat_t, long_t, v_ego):
    v = float(out["plan"][0, 0])
    return log.ModelDataV2.Action(desiredCurvature=v, desiredAcceleration=10 * v, shouldStop=False)

  def close(self):
    self.closed = True


def make_remote_pair(tmp_path, *, probing=False, **remote_kwargs):
  events, clock = [], Clock()
  primary = FakeModel("lightspeed", 1.0, chestnut=True, events=events, cost=0.039, clock=clock)
  partner = FakeRemote(events, clock=clock, **remote_kwargs)
  pair = ModelPair(primary, partner, PairPlan("drive", "polaris", "lightspeed", "polaris", "lateral"), now=clock,
                   status_path=tmp_path / "pair.json")
  pair.frame_ids = (100, 100)
  pair.probing = probing    # most tests start from driving; a real partner process starts probing
  return pair, primary, partner, events


def test_a_partner_process_gets_its_job_at_the_start_of_the_frame(tmp_path):
  pair, primary, partner, events = make_remote_pair(tmp_path)
  out, action = step(pair)
  # the job goes out before the primary runs (the comma GPU puts the primary's warp first); outputs are collected after
  assert events == ["polaris:job", "lightspeed:start", "lightspeed:end", "polaris:collect"]
  assert partner.jobs == [(100, 100)]
  assert action.desiredCurvature == pytest.approx(2.001) and action.desiredAcceleration == 10.0
  assert pair.partner_device == "comma" and pair.partner_image == "own process"
  pair.write_status(force=True)
  status = json.loads((tmp_path / "pair.json").read_text())
  assert status["partnerProcess"] is True and status["partnerMsP50"] == 37.0 and status["lateFrames"] == 0


def test_a_late_partner_reuses_its_previous_outputs_once_then_the_primary_takes_the_role_back(tmp_path):
  answers = {1: True, 2: False, 3: True, 4: False, 5: False, 6: True}
  pair, primary, partner, events = make_remote_pair(tmp_path, answer=lambda seq: answers.get(seq, True))
  _, first = step(pair)
  _, late_once = step(pair)
  _, back = step(pair)
  assert first.desiredCurvature == pytest.approx(2.001)
  assert late_once.desiredCurvature == pytest.approx(2.001)      # previous partner outputs, one frame old
  assert back.desiredCurvature == pytest.approx(2.003) and not pair.probing
  _, late_again = step(pair)
  _, late_twice = step(pair)
  assert late_again.desiredCurvature == pytest.approx(2.003)
  # two in a row: the primary steers and keeps steering (the partner probes again), never alternating per frame
  assert late_twice.desiredCurvature == 1.0 and pair.probing and pair.demotions == 1
  _, after = step(pair)
  assert after.desiredCurvature == 1.0 and pair.summary()["lateFrames"] == 3


def test_probing_stops_a_partner_that_is_never_on_time(tmp_path, monkeypatch):
  monkeypatch.setattr(model_pair, "PROBE_LIMIT_FRAMES", 60)
  pair, primary, partner, events = make_remote_pair(tmp_path, probing=True, ready=lambda seq: 0.050)
  for _ in range(60):
    step(pair)
  assert not pair.partner_active and "never on time" in pair.stop_reason
  assert pair.summary()["probeReadyMsP50"] == pytest.approx(50.0)


def test_a_partner_process_that_keeps_missing_its_deadline_stops_steering(tmp_path):
  pair, primary, partner, events = make_remote_pair(tmp_path, answer=lambda seq: seq <= model_pair.WARMUP_FRAMES)
  pair.demotions = model_pair.MAX_DEMOTIONS      # no more chances: stopped for the drive
  for _ in range(model_pair.WARMUP_FRAMES + model_pair.SLOW_LIMIT + 1):
    step(pair)
  # the second late frame in a row acts at once (sooner than the guard's 5 in 40)
  assert not pair.partner_active and "late in a row" in pair.stop_reason and partner.closed
  _, action = step(pair)
  assert action.desiredCurvature == 1.0 and partner.seq == model_pair.WARMUP_FRAMES + 2


def test_a_failed_partner_process_stops_without_stopping_the_drive(tmp_path):
  pair, primary, partner, events = make_remote_pair(tmp_path, fail=True)
  out, action = step(pair)
  assert not pair.partner_active and "partner process ended" in pair.stop_reason and partner.closed
  assert float(out["plan"][0, 0]) == 1.0 and action.desiredCurvature == 1.0
  # without camera frame ids from modeld the partner process cannot run
  pair, primary, partner, events = make_remote_pair(tmp_path)
  pair.frame_ids = None
  step(pair)
  assert not pair.partner_active and "frame ids" in pair.stop_reason


def test_setup_pair_starts_a_comma_partner_process_beside_a_chestnut_primary(tmp_path, monkeypatch):
  from openpilot.starpilot.models import runner
  monkeypatch.setattr(model_pair, "STATUS_PATH", tmp_path / "status.json")
  monkeypatch.setattr(model_pair, "_is_big", lambda mid: mid == "lightspeed")

  class Catalog(FakeModel, runner.CatalogModelState):
    def __init__(self, *args, **kwargs):
      FakeModel.__init__(self, *args, **kwargs)

  events, asked, started = [], [], []
  primary = Catalog("lightspeed", 1.0, chestnut=True, events=events)

  def artifact(mid, chestnut_ok=True):
    asked.append(chestnut_ok)
    return tmp_path / "p.pkl", "v15", "3" * 64, False

  monkeypatch.setattr(model_pair, "partner_artifact", artifact)

  class Remote(FakeRemote):
    def __init__(self, mid, version, path, sha, cam_w, cam_h, *, main_wide_camera, use_extra_client):
      super().__init__(events)
      started.append((mid, version, path.name, cam_w, cam_h, main_wide_camera, use_extra_client))

    def wait_ready(self):
      return {}

  cfg = {"enabled": True, "lateralModel": "polaris", "longitudinalModel": "lightspeed", "mode": "drive"}
  model, reason = setup_pair(primary, None, 1344, 760, config=cfg, streams=(False, True), remote=Remote,
                             load=lambda *a: pytest.fail("the partner runs in its own process"))
  assert reason == "" and isinstance(model, ModelPair) and model.remote and asked == [False]
  assert started == [("polaris", "v15", "p.pkl", 1344, 760, False, True)]

  class Broken(Remote):
    def wait_ready(self):
      raise RuntimeError("no GPU")

    def close(self):
      started.append("closed")

  model, reason = setup_pair(primary, None, 1344, 760, config=cfg, streams=(False, True), remote=Broken)
  assert model is primary and "no GPU" in reason and started[-1] == "closed"
  assert read_status(tmp_path / "status.json")["state"] == "refused"


def test_the_partner_process_is_modelds_spare_and_the_pair_reuses_it(tmp_path, monkeypatch):
  from openpilot.starpilot.models import runner
  from openpilot.starpilot.models.model_pair import start_partner_spare
  from openpilot.starpilot.models.pair_partner import PartnerDriver
  monkeypatch.setattr(model_pair, "STATUS_PATH", tmp_path / "status.json")
  monkeypatch.setattr(model_pair, "_is_big", lambda mid: mid == "lightspeed")
  monkeypatch.setattr(model_pair, "partner_artifact", lambda mid, chestnut_ok=True: (tmp_path / "p.pkl", "v15", "3" * 64, False))
  events, started = [], []

  class Remote(FakeRemote):
    def __init__(self, mid, version, path, sha, cam_w, cam_h, *, main_wide_camera, use_extra_client):
      super().__init__(events)
      self.info = {"desireLen": 8}
      started.append(mid)

    def wait_ready(self):
      return self.info

  spare = start_partner_spare("rdf43", 1344, 760, (False, True), remote=Remote)
  assert isinstance(spare, PartnerDriver) and spare.model_id == "polaris" and spare.npy["desire"].shape == (8,)

  class Catalog(FakeModel, runner.CatalogModelState):
    def __init__(self, *args, **kwargs):
      FakeModel.__init__(self, *args, **kwargs)

  primary = Catalog("lightspeed", 1.0, chestnut=True, events=events)
  spare.model_id = "rdf43"
  cfg = {"enabled": True, "lateralModel": "rdf43", "longitudinalModel": "lightspeed", "mode": "drive"}
  model, reason = setup_pair(primary, spare, 1344, 760, config=cfg, streams=(False, True), remote=Remote)
  # no second process: the pair drives with the spare's process and keeps it when the pair stops
  assert reason == "" and model.partner is spare.partner and model.keep_partner and started == ["rdf43"]
  model._stop_partner("test")
  assert not spare.partner.closed

  # a spare that cannot start leaves modeld to load its own small model
  class Broken(Remote):
    def wait_ready(self):
      raise RuntimeError("no memory")

  reports = []
  assert start_partner_spare("rdf43", 1344, 760, (False, True), remote=Broken, report=reports.append) is None
  assert "no memory" in reports[0]


def test_the_spare_partner_drives_alone_if_the_chestnut_fails():
  from openpilot.starpilot.models.pair_partner import PartnerDriver, PartnerError
  events, enqueued = [], []
  answers = {1: True, 2: False, 3: False}
  partner = FakeRemote(events, answer=lambda seq: answers.get(seq, True))
  driver = PartnerDriver(partner, 8)
  with pytest.raises(PartnerError, match="frame ids"):
    driver.run({}, {}, {})
  driver.frame_ids = (7, 7)
  first = driver.run({}, {}, {}, lambda: enqueued.append(True))
  assert float(first["plan"][0, 0]) == pytest.approx(2.001) and enqueued == [True] and partner.jobs == [(7, 7)]
  assert driver.run({}, {}, {}) is first                      # one missed frame reuses the previous outputs
  with pytest.raises(PartnerError, match="late"):
    driver.run({}, {}, {})                                    # a second one is modeld's to handle
  action = driver.get_action_from_model(first, log.ModelDataV2.Action(), 0.3, 0.5, 20.0)
  assert action.desiredCurvature == pytest.approx(2.001) and driver.chestnut is False


def test_a_tuned_comma_gpu_build_is_used_for_the_partner_only_when_registered_and_intact(tmp_path, monkeypatch):
  import hashlib
  from openpilot.starpilot.models import manager
  from openpilot.starpilot.models.model_pair import local_build_path, partner_artifact
  rows = {"rdf43": {"name": "RDF V4", "version": "v15", "artifact_sha256": "s" * 64}}
  monkeypatch.setattr(manager, "catalog", lambda root: rows)
  monkeypatch.setattr(manager, "verified_artifact", lambda *a, **k: tmp_path / "standard.pkl" if len(a) < 4 else None)
  assert partner_artifact("rdf43", root=tmp_path, chestnut_ok=False) == (tmp_path / "standard.pkl", "v15", "s" * 64, False)
  tuned = local_build_path("rdf43", tmp_path, "tuned")
  assert tuned == tmp_path / "local_builds" / "rdf43_driving_tuned_tinygrad.pkl"
  tuned.parent.mkdir()
  tuned.write_bytes(b"tuned build")
  entry = {"artifact_sha256": hashlib.sha256(b"tuned build").hexdigest(), "artifact_size": 11, "behavior_version": "v15"}
  (tmp_path / "local_variants.json").write_text(json.dumps({"rdf43": {"tuned": entry}}))
  assert partner_artifact("rdf43", root=tmp_path, chestnut_ok=False) == (tuned, "v15", entry["artifact_sha256"], False)
  # a changed file, or one for another model version, falls back to the catalog build
  tuned.write_bytes(b"tuned BUILD")
  assert partner_artifact("rdf43", root=tmp_path, chestnut_ok=False)[0] == tmp_path / "standard.pkl"
  tuned.write_bytes(b"tuned build")
  (tmp_path / "local_variants.json").write_text(json.dumps({"rdf43": {"tuned": {**entry, "behavior_version": "v16"}}}))
  assert partner_artifact("rdf43", root=tmp_path, chestnut_ok=False)[0] == tmp_path / "standard.pkl"


def test_a_partner_process_probes_first_and_takes_over_only_when_on_time_and_not_steering(tmp_path):
  pair, primary, partner, events = make_remote_pair(tmp_path, probing=True)
  assert pair.probing and json.loads(json.dumps(pair.summary()))["probing"] is True
  # probing: the primary drives, modeld never waits for the partner (no collect), every answer is scored
  for _ in range(model_pair.PROBE_FRAMES - 1):
    _, action = step(pair)
    assert action.desiredCurvature == 1.0
  assert "polaris:collect" not in events and pair.probing
  pair.lat_active = True                         # 40 on time, but the car is steering itself: no handover yet
  step(pair)
  step(pair)
  assert pair.probing and pair.summary()["probeOnTime"] == f"{model_pair.PROBE_FRAMES}/{model_pair.PROBE_FRAMES}"
  pair.lat_active = False
  step(pair)                                     # handed over at the end of this frame
  assert not pair.probing
  _, action = step(pair)
  assert action.desiredCurvature > 2.0 and "polaris:collect" in events
  pair.write_status(force=True)
  assert json.loads((tmp_path / "pair.json").read_text())["state"] == "driving"


def test_a_slow_probing_partner_never_takes_over_and_never_slows_modeld(tmp_path):
  pair, primary, partner, events = make_remote_pair(tmp_path, probing=True, ready=lambda seq: 0.046)
  for _ in range(3 * model_pair.PROBE_FRAMES):
    _, action = step(pair)
    assert action.desiredCurvature == 1.0
  assert pair.probing and pair.partner_active and pair.summary()["probeReadyMsP95"] == pytest.approx(46.0)
  assert pair.summary()["runMsMax"] == pytest.approx(39.0)     # modeld's frame is the primary's alone


def test_a_driving_partner_that_runs_late_goes_back_to_probing_then_stops_after_max_demotions(tmp_path):
  pair, primary, partner, events = make_remote_pair(tmp_path, answer=lambda seq: seq <= model_pair.WARMUP_FRAMES)
  for _ in range(model_pair.WARMUP_FRAMES + model_pair.SLOW_LIMIT + 1):
    step(pair)
  assert pair.partner_active and pair.probing and pair.demotions == 1 and "probing again" in pair.stop_reason
  _, action = step(pair)
  assert action.desiredCurvature == 1.0                         # the primary steers again at once
  pair.demotions = model_pair.MAX_DEMOTIONS
  pair.probing = False
  pair.guard = model_pair.PairGuard()
  for _ in range(model_pair.WARMUP_FRAMES + model_pair.SLOW_LIMIT + 1):
    step(pair)
  assert not pair.partner_active and "late" in pair.stop_reason and partner.closed
