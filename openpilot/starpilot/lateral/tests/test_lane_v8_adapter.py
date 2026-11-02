"""Lane version 8 adapter: saved-settings reading and the dispatcher-facing controller interface.
On the comma: PYTHONPATH=/tmp/shim:/data/openpilot python /tmp/shim/run_tests.py openpilot.starpilot.lateral.tests.test_lane_v8_adapter
"""
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile

import numpy as np
import pytest

from openpilot.common.params import UnknownKeyName
from openpilot.starpilot.lateral.lane_centering import ControlMode, LaneCenteringInput, LaneCenteringResult
from openpilot.starpilot.lateral.lane_led_settings import LaneLedSettings
from openpilot.starpilot.lateral.lane_motion import Motion
from openpilot.starpilot.lateral.lane_v8_adapter import LEVER_STATE_KEY, V8_DEFAULTS, V8_MAP, V8Controller, settings
from openpilot.starpilot.lateral.lane_versions import LaneCenteringVersions, read_selected, DEFAULTS
from openpilot.starpilot.lateral.tests.lane_led_plant import Road, X_IDXS


class FakeParams:
  """Just enough of Params for the readers: files under a temp dir, unknown keys raise."""

  def __init__(self, root):
    self.root = Path(root)
    self.known = set(V8_DEFAULTS) | {"LaneCentering", "LaneCenteringVersion", LEVER_STATE_KEY}

  def get_param_path(self, key):
    if key not in self.known:
      raise UnknownKeyName(key)
    return str(self.root / key)

  def put(self, key, value):
    (self.root / key).write_bytes(value if isinstance(value, bytes) else str(value).encode())


def test_defaults_when_nothing_saved():
  with tempfile.TemporaryDirectory() as d:
    s = settings(FakeParams(d))
    assert s == LaneLedSettings(enabled=True)
    assert all(key in DEFAULTS for key in V8_DEFAULTS), "the dispatcher's DEFAULTS carry the v8 keys for the Galaxy owner"


def test_saved_values_and_master_switch_round_trip():
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d)
    p.put(V8_MAP["recovery_time"], "4.5"); p.put(V8_MAP["bias_enabled"], "0"); p.put("LaneCentering", "0")
    s = settings(p)
    assert s.recovery_time == 4.5 and s.bias_enabled is False and s.enabled is False
    p.put("LaneCenteringVersion", "8"); p.put("LaneCentering", "1")
    version, selected = read_selected(p)
    assert version == 8 and isinstance(selected, LaneLedSettings) and selected.recovery_time == 4.5 and selected.enabled


def test_invalid_saved_value_disables_rather_than_guessing():
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d)
    p.put(V8_MAP["recovery_time"], "0.1")      # below the range
    assert settings(p) == LaneLedSettings(enabled=False)
    p.put(V8_MAP["recovery_time"], "3.0"); p.put(V8_MAP["drop_confidence"], "0.5")  # above adopt
    assert settings(p) == LaneLedSettings(enabled=False)


def _model(road, X, Y, psi, idx):
  g = road.frame(X, Y, psi, idx)
  x = g["x"]
  line = lambda y: NS(x=x.tolist(), y=y.tolist(), z=np.zeros(len(x)).tolist())
  return NS(laneLines=[line(g["left"] - 3.5), line(g["left"]), line(g["right"]), line(g["right"] + 3.5)],
            laneLineProbs=[.3, .95, .95, .3], laneLineStds=[.3, .05, .05, .3],
            roadEdges=[line(g["edge_left"]), line(g["edge_right"])], roadEdgeStds=[.3, .3],
            position=NS(x=x.tolist(), y=g["centre"].tolist(), yStd=np.full(len(x), .1).tolist()),
            meta=NS(laneChangeState=0), action=NS(desiredCurvature=0.))


def test_controller_interface_through_the_dispatcher():
  road = Road([(400., 0., 0.)])
  X, Y, psi, idx = road.start(-0.3, 0.)
  versions = LaneCenteringVersions()
  s = LaneLedSettings(enabled=True)
  kappa_prev = 0.
  result = None
  for i in range(400):                      # 4 s at 100 Hz, straight road, car 30 cm left of centre
    t_ns = 1_000_000_000 + i * 10_000_000
    model = _model(road, X, Y, psi, idx)
    obs = LaneCenteringInput(model=model, model_sample_id=1_000_000_000 + (i // 5) * 50_000_000, model_valid=True,
                             base_curvature=0., speed_mps=18., mode=ControlMode.LATERAL_ONLY, lateral_active=True,
                             settings=s, elapsed_seconds=.01)
    motion = Motion(t_ns, 0., kappa_prev, 0., .33)
    result = versions.update(obs, version=8, motion=motion, measured_curvature=0.)
    assert isinstance(result, LaneCenteringResult)
    kappa_prev = result.candidate_curvature
    assert abs(kappa_prev) < .01
  assert result.reason == "lane_led", result.reason
  assert result.candidate_curvature > 0., "centre to the right must turn right"
  assert versions.version == 8 and versions.diagnostic["algorithm"] == 8 and versions.diagnostic["state"] == "lane_led"
  assert result.correction == pytest.approx(result.candidate_curvature - 0.)


def test_lever_learner_state_persists_through_params():
  """The lever learner's regression state is stored under LaneCenteringV8LeverState once it has new frames and when
  lateral control ends, and a fresh controller loads it on its first update (change 23)."""
  with tempfile.TemporaryDirectory() as d:
    p = FakeParams(d)
    c = V8Controller(params=p)
    for k in range(50):
      c.controller.lever_learner.observe(0.0063, 1 / 300., 18.)
    c._store_state(100., force=True)
    saved = (Path(d) / LEVER_STATE_KEY).read_bytes()
    assert b'"A"' in saved and b'"accepted": 50' in saved
    c2 = V8Controller(params=p)
    c2._load_state()
    assert c2.controller.lever_learner.accepted == 50
    assert c2.controller.lever_learner.lever(18., 1.5) == pytest.approx(c.controller.lever_learner.lever(18., 1.5))
    # nothing new: no write; new frames but inside the interval: no write unless forced
    (Path(d) / LEVER_STATE_KEY).write_bytes(b"marker")
    c2._store_state(200., force=True)
    assert (Path(d) / LEVER_STATE_KEY).read_bytes() == b"marker"
    c2.controller.lever_learner.observe(0.0063, 1 / 300., 18.)
    c2._store_state(200.)
    assert (Path(d) / LEVER_STATE_KEY).read_bytes() == b"marker", "20 frames and 60 s before an unforced store"
    for k in range(25):
      c2.controller.lever_learner.observe(0.0063, 1 / 300., 18.)
    c2._store_state(200.)
    assert (Path(d) / LEVER_STATE_KEY).read_bytes() != b"marker"
    # rubbish on disk is ignored
    (Path(d) / LEVER_STATE_KEY).write_bytes(b"{not json")
    c3 = V8Controller(params=p); c3._load_state()
    assert c3.controller.lever_learner.accepted == 0 and c3.controller.lever_learner.lever(18., 1.5) == 1.5


def test_missing_motion_is_a_plain_pass_through():
  c = V8Controller()
  obs = LaneCenteringInput(model=None, model_sample_id=1, model_valid=True, base_curvature=.002, speed_mps=18.,
                           mode=ControlMode.LATERAL_ONLY, lateral_active=True, settings=LaneLedSettings(), elapsed_seconds=.01)
  r = c.update(obs, motion=None)
  assert r.candidate_curvature == .002 and r.correction == 0. and r.reason == "v8_no_motion"
