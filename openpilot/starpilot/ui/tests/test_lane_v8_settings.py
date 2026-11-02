"""Real Params, owner and Galaxy transactions for the v8 lane-centring menu.
Builds the Galaxy fixture by hand so the device's pytest stand-in (no fixture injection) can run it:
  PYTHONPATH=/tmp/shim:/data/openpilot python /tmp/shim/run_tests.py openpilot.starpilot.ui.tests.test_lane_v8_settings
"""
from dataclasses import replace
import pytest
from openpilot.starpilot.galaxy.tests import test_settings as fixtures
from openpilot.starpilot.galaxy.settings import SettingsChanged
from openpilot.starpilot.ui.tests.test_lane_v4_settings import edit
from openpilot.starpilot.ui import lane_v8_menu
from openpilot.starpilot.lateral.lane_v8_adapter import V8_MAP
from openpilot.starpilot.lateral.lane_versions import read_selected
from openpilot.starpilot.lateral.lane_led_settings import LaneLedSettings


def _fixture():
  v = fixtures.SettingsGatewayTest(); v.setUp()
  v.params.put_bool("IsMetric", True, block=True)
  return v


def test_select_v8_shows_groups_and_runtime_reads_defaults():
  f = _fixture()
  try:
    edit(f, 'lane', 'Lane centering version', 'v8')
    assert f.params.get('LaneCenteringVersion') == 8
    version, s = read_selected(f.params)
    assert version == 8 and s == LaneLedSettings(enabled=True)
    page = f.page('lane')
    labels = [r['label'] for r in page['rows']]
    for title in ('Recovery', 'Limits', 'Trust', 'One line and clearance', 'Hand-over', 'Bias learning'):
      assert title in labels, labels
    assert 'Lane Centering Strength' not in labels
    for group in lane_v8_menu.PAGES:
      sub = f.page(group)
      assert sub['rows'], group
      assert all(r['label'] for r in sub['rows'])
  finally:
    f.doCleanups()


def test_edit_saves_scaled_values_and_rejects_out_of_range():
  f = _fixture()
  try:
    edit(f, 'lane', 'Lane centering version', 'v8')
    edit(f, 'lane/recovery', 'Recovery time', 4.5)
    assert f.params.get(V8_MAP['recovery_time']) == pytest.approx(4.5)
    assert read_selected(f.params)[1].recovery_time == pytest.approx(4.5)
    edit(f, 'lane/trust', 'Adopt confidence', 40)
    assert f.params.get(V8_MAP['adopt_confidence']) == pytest.approx(.40)
    edit(f, 'lane/bias', 'Bias limit', 0.4)
    assert f.params.get(V8_MAP['bias_limit_accel']) == pytest.approx(.4)
    edit(f, 'lane/bias', 'Learn bias', 'Off')
    assert read_selected(f.params)[1].bias_enabled is False
    with pytest.raises(SettingsChanged):
      edit(f, 'lane/recovery', 'Recovery time', 9.)
    with pytest.raises(SettingsChanged):
      edit(f, 'lane/trust', 'Drop confidence', 45)   # above the adopt confidence just set
    assert read_selected(f.params)[1].recovery_time == pytest.approx(4.5)
  finally:
    f.doCleanups()


def test_switching_away_keeps_v8_tune_and_other_versions_intact():
  f = _fixture()
  try:
    f.params.put('LaneCenteringStrength', 1.5, block=True)
    edit(f, 'lane', 'Lane centering version', 'v8')
    edit(f, 'lane/recovery', 'Recovery time', 2.)
    edit(f, 'lane', 'Lane centering version', 'v4')
    assert read_selected(f.params)[0] == 4
    assert f.params.get(V8_MAP['recovery_time']) == pytest.approx(2.)
    edit(f, 'lane', 'Lane centering version', 'v8')
    assert read_selected(f.params)[1].recovery_time == pytest.approx(2.)
    assert f.params.get('LaneCenteringStrength') == 1.5
  finally:
    f.doCleanups()
