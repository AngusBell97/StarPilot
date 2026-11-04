"""SP-040: the close-gap lane change only acts for overtakes. Right-hand drive (UK) -> right-hand lane changes only;
left-hand drive -> left only; unknown wheel side -> both, as before."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.longitudinal import lane_change_gap as lcg
from openpilot.starpilot.longitudinal.lane_change_gap import (
  LaneChangeGap, LaneChangePace, LaneGapPreferences, overtake_manoeuvre, overtake_side_from_rhd, read_overtake_side)
from openpilot.starpilot.longitudinal.lane_change_chill import LaneChangeChill
from openpilot.starpilot.longitudinal.tests.test_lane_change_response import warmed, cycle, frame, POLICY, LEADS

S = 1_000_000_000


def sided(i, side, direction, **kw):
  return frame(i, signal_direction=direction, direction=direction, overtake_side=side, leads=LEADS, **kw)


def test_side_mapping():
  assert overtake_side_from_rhd(True) == 'right' and overtake_side_from_rhd(False) == 'left'
  assert overtake_side_from_rhd(None) is None and overtake_side_from_rhd(1) is None


def test_overtake_manoeuvre_rules():
  assert overtake_manoeuvre(None)
  assert overtake_manoeuvre(sided(0, None, 'left')) and overtake_manoeuvre(sided(0, None, 'right'))
  assert overtake_manoeuvre(sided(0, 'right', 'right')) and not overtake_manoeuvre(sided(0, 'right', 'left'))
  assert overtake_manoeuvre(sided(0, 'left', 'left')) and not overtake_manoeuvre(sided(0, 'left', 'right'))
  assert overtake_manoeuvre(sided(0, 'right', 'none'))  # nothing signalled: keep observing
  # The model's direction counts once the stalk has self-cancelled mid-manoeuvre.
  assert not overtake_manoeuvre(frame(0, signal_direction='none', direction='left', overtake_side='right', leads=LEADS))
  assert overtake_manoeuvre(SimpleNamespace(signal_direction='left'))  # no side attribute at all (chill test frames)


@pytest.mark.parametrize('side,allowed,blocked', [('right', 'right', 'left'), ('left', 'left', 'right')])
def test_response_only_on_the_overtaking_side(side, allowed, blocked):
  g = warmed()
  g.step(POLICY, sided(15, side, blocked, state='preLaneChange'), base_follow=1.5, long_active=True)
  assert g.phase == 'idle' and g.reason == 'not_overtake_side'
  assert g.comfort_target(-.6, cruise=.8, previous=-.6, positive_jerk=.6, blocked=False, experimental=False) == -.6
  assert g.limit_acceleration(.5) == .5
  g = warmed()
  g.step(POLICY, sided(15, side, allowed, state='preLaneChange'), base_follow=1.5, long_active=True)
  assert g.phase == 'preparing'
  assert g.comfort_target(-.6, cruise=.8, previous=-.6, positive_jerk=.6, blocked=False, experimental=False) > -.6


def test_unknown_side_keeps_both_directions():
  for direction in ('left', 'right'):
    g = warmed()
    g.step(POLICY, sided(15, None, direction, state='preLaneChange'), base_follow=1.5, long_active=True)
    assert g.phase == 'preparing'


def test_blocked_side_after_an_overtake_never_clamps():
  # Overtake to the right, settle, then the driver signals left to return: the module goes idle and passes through.
  g = warmed()
  prev = 0.
  for i in range(15, 25):
    prev = cycle(g, i, moving=True, previous=prev, signal_direction='right', direction='right', overtake_side='right')
  assert g.response_raised and prev > 0.
  g.step(POLICY, sided(25, 'right', 'left', state='preLaneChange'), base_follow=1.5, long_active=True)
  assert g.phase == 'idle' and g.reason == 'not_overtake_side' and g.acceleration_ceiling is None
  assert g.limit_acceleration(.6) == .6
  assert g.limit_acceleration(-.4) == -.4
  for i in range(26, 30):
    g.step(POLICY, sided(i, 'right', 'left', state='laneChangeStarting'), base_follow=1.5, long_active=True)
    assert g.comfort_target(-.2, cruise=.8, previous=.3, positive_jerk=.6, blocked=False, experimental=False) == -.2
    assert g.limit_acceleration(.6) == .6


def test_fast_pace_only_on_the_overtaking_side():
  policy = replace(POLICY, close_gap_fast_pace=True)
  p = LaneChangePace()
  normal = policy.duration_s
  for i in range(15):
    p.duration(policy, frame(i, state='off', signal_direction='none', direction='none', overtake_side='right', leads=LEADS), long_active=True)
  assert p.duration(policy, sided(15, 'right', 'left', state='laneChangeStarting'), long_active=True) == normal
  assert p.reason == 'not_overtake_side' and not p.selected


def test_chill_window_only_on_the_overtaking_side():
  c = LaneChangeChill()
  assert c.step(POLICY, sided(1, 'right', 'left', state='preLaneChange'), long_active=True, now_ns=S) is False
  assert c.reason == 'not_overtake_side'
  assert c.step(POLICY, sided(2, 'right', 'right', state='preLaneChange'), long_active=True, now_ns=S + 50_000_000) is True
  assert c.reason == 'manoeuvre'
  c = LaneChangeChill()
  assert c.step(POLICY, sided(1, 'left', 'right', state='laneChangeStarting'), long_active=True, now_ns=S) is False
  assert c.step(POLICY, sided(1, None, 'right', state='laneChangeStarting'), long_active=True, now_ns=S) is True


def test_preferences_refresh_the_wheel_side_from_params():
  class FakeParams:
    def __init__(self, rhd):
      self.rhd = rhd
    def get(self, key, *a, **k):
      return None
    def get_bool(self, key, *a, **k):
      assert key == 'IsRhdDetected'
      return self.rhd
  assert read_overtake_side(FakeParams(True)) == 'right' and read_overtake_side(FakeParams(False)) == 'left'
  assert read_overtake_side(SimpleNamespace()) is None
  old = lcg.WHEEL_SIDE['overtake']
  try:
    LaneGapPreferences(FakeParams(True)).sample(S)
    assert lcg.WHEEL_SIDE['overtake'] == 'right'
    LaneGapPreferences(FakeParams(False)).sample(S)
    assert lcg.WHEEL_SIDE['overtake'] == 'left'
  finally:
    lcg.WHEEL_SIDE['overtake'] = old


def test_frame_default_side_is_unknown():
  assert frame(0).overtake_side is None
