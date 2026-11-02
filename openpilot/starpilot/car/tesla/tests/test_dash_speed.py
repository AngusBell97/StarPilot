"""Device copy of tests/test_dash_speed.py: same checks through the package import."""
import math
import pytest
from openpilot.starpilot.car.tesla import dash_speed as ds

MPH = ds.MPH_TO_MS


def test_overread_matches_measurements():
  for mph, over in ((25, 0.2), (45, 0.36), (60, 0.55), (77, 0.93)):
    assert abs(ds.dash_overread_mps(mph * MPH) / MPH - over) < 0.1, mph


def test_overread_zero_at_rest_and_monotonic():
  assert ds.dash_overread_mps(0.0) == 0.0
  assert ds.dash_overread_mps(float('nan')) == 0.0
  last = -1.0
  for mph in range(0, 120):
    cur = ds.dash_overread_mps(mph * MPH)
    assert cur >= last
    last = cur


@pytest.mark.parametrize('set_mph', [20, 25, 30, 33, 40, 42, 50, 60, 65, 70, 77, 78, 80, 90])
def test_dash_shows_set_speed(set_mph):
  target = ds.planner_target(set_mph * MPH)
  assert target <= set_mph * MPH
  assert abs(ds.dash_reading_mps(target) / MPH - set_mph) < 0.02
  assert set_mph - target / MPH <= 1.3


def test_seventy_seven_case():
  target = ds.planner_target(77 * MPH) / MPH
  assert 75.9 < target < 76.2 and 76.5 < target + 0.93 < 77.5


def test_low_speed_and_unset_untouched():
  for mps in (0.0, 1.0, 4.9 * MPH):
    assert ds.planner_target(mps) == mps
  assert ds.planner_target(math.inf) == math.inf


def test_ui_speed_conversion():
  assert ds.ui_speed_mps(255, 0) is None
  assert abs(ds.ui_speed_mps(78, 0) - 78 * MPH) < 1e-9
  assert abs(ds.ui_speed_mps(120, 1) - 120 / 3.6) < 1e-9


class _CP:
  def __init__(self, fp, long_):
    self.carFingerprint = fp
    self.openpilotLongitudinalControl = long_


def test_enabled_gate():
  assert ds.enabled(_CP('TESLA_MODEL_3', True)) and ds.enabled(_CP('TESLA_MODEL_Y', True))
  assert not ds.enabled(_CP('TESLA_MODEL_3', False)) and not ds.enabled(_CP('HYUNDAI_IONIQ_6', True))
  assert not ds.enabled(object())
