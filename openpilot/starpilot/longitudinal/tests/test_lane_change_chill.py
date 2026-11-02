"""Pure-python tests for the lane-change Chill window (no cereal needed)."""
from types import SimpleNamespace

import pytest

from openpilot.starpilot.longitudinal.lane_change_chill import LaneChangeChill, HOLD_NS, MAX_WINDOW_NS

S = 1_000_000_000


def policy(enabled=True, close_gap=True):
  return SimpleNamespace(enabled=enabled, close_gap=close_gap)


def frame(state='preLaneChange', signal='left', speed=30.0, gas=False, brake=False, standstill=False):
  return SimpleNamespace(lane_state=state, signal_direction=signal, speed_mps=speed, gas_pressed=gas,
                         brake_pressed=brake, standstill=standstill)


def test_off_without_preference_or_long_control():
  c = LaneChangeChill()
  assert c.step(None, frame(), long_active=True, now_ns=S) is False
  assert c.step(policy(close_gap=False), frame(), long_active=True, now_ns=S) is False
  assert c.step(policy(enabled=False), frame(), long_active=True, now_ns=S) is False
  assert c.step(policy(), frame(), long_active=False, now_ns=S) is False
  assert c.reason == 'disabled'


def test_signalled_and_moving_states_open_the_window():
  c = LaneChangeChill()
  assert c.step(policy(), frame('off', 'none'), long_active=True, now_ns=S) is False
  assert c.step(policy(), frame('preLaneChange', 'left'), long_active=True, now_ns=2 * S) is True
  assert c.reason == 'manoeuvre'
  # indicator released while moving still counts (auto cancel on signal-off is handled by desire_helper)
  assert c.step(policy(), frame('laneChangeStarting', 'none'), long_active=True, now_ns=3 * S) is True
  assert c.step(policy(), frame('laneChangeFinishing', 'none'), long_active=True, now_ns=4 * S) is True


def test_pre_lane_change_without_signal_does_not_qualify():
  c = LaneChangeChill()
  assert c.step(policy(), frame('preLaneChange', 'none'), long_active=True, now_ns=S) is False


def test_hold_then_release_after_manoeuvre():
  c = LaneChangeChill()
  assert c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=S)
  assert c.step(policy(), frame('off', 'none'), long_active=True, now_ns=S + HOLD_NS) is True
  assert c.reason == 'hold'
  assert c.step(policy(), frame('off', 'none'), long_active=True, now_ns=S + HOLD_NS + 1) is False
  assert c.reason == 'idle' and c.started_ns is None


def test_window_timeout():
  c = LaneChangeChill()
  t = S
  while t <= S + MAX_WINDOW_NS:
    assert c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=t)
    t += S // 2
  assert c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=t) is False
  assert c.reason == 'window_timeout'


def test_driver_pedal_hands_mode_back_immediately():
  c = LaneChangeChill()
  assert c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=S)
  assert c.step(policy(), frame('laneChangeStarting', gas=True), long_active=True, now_ns=S + 1000) is False
  assert c.reason == 'driver_override'
  assert c.step(policy(), frame('laneChangeStarting', brake=True), long_active=True, now_ns=S + 2000) is False


def test_low_speed_and_missing_frame_do_not_open():
  c = LaneChangeChill()
  assert c.step(policy(), frame(speed=5.0), long_active=True, now_ns=S) is False
  assert c.step(policy(), None, long_active=True, now_ns=2 * S) is False


def test_shape_limits_only_upward_moves():
  c = LaneChangeChill()
  c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=S)
  # braking release at 2 m/s^3: from -1.0 in 0.05 s at most -0.9
  assert c.shape(0.5, previous=-1.0, dt=0.05, positive_jerk=1.0) == pytest.approx(-0.9)
  assert c.limited is True
  # downward moves pass through untouched
  assert c.shape(-1.5, previous=-1.0, dt=0.05, positive_jerk=1.0) == -1.5
  assert c.limited is False
  # zero crossing splits the step: 0.02 s to reach zero at 2 m/s^3, then 0.03 s at cruise jerk 1.0
  assert c.shape(0.5, previous=-0.04, dt=0.05, positive_jerk=1.0) == pytest.approx(0.03)
  # positive requests ramp at the cruise jerk
  assert c.shape(0.5, previous=0.1, dt=0.05, positive_jerk=0.8) == pytest.approx(0.14)


def test_shape_is_identity_when_inactive_or_invalid():
  c = LaneChangeChill()
  assert c.shape(0.5, previous=-1.0, dt=0.05, positive_jerk=1.0) == 0.5
  c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=S)
  assert c.shape(0.5, previous=-1.0, dt=0.0, positive_jerk=1.0) == 0.5
  assert c.shape(0.5, previous=float('nan'), dt=0.05, positive_jerk=1.0) == 0.5
  assert c.shape(0.5, previous=-1.0, dt=0.05, positive_jerk=0.0) == 0.5


def test_diagnostic_fields():
  c = LaneChangeChill()
  d = c.diagnostic()
  assert d == dict(chill=False, chill_reason='idle', chill_limit=None, chill_limited=False)
  c.step(policy(), frame('laneChangeStarting'), long_active=True, now_ns=S)
  c.shape(1.0, previous=0.0, dt=0.05, positive_jerk=1.0)
  d = c.diagnostic()
  assert d['chill'] is True and d['chill_reason'] == 'manoeuvre' and d['chill_limited'] is True
