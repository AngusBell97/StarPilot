"""A stalled selfdrived (the screen's own red System Unresponsive alert) wakes a sleeping screen."""
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.selfdrive.ui import ui_state as native
from openpilot.starpilot.ui.display_preferences import DisplayPreferences

NOW = 100.0


def device():
  d = native.Device.__new__(native.Device)
  d._ignition = True
  d._awake = False
  d._screen_off = False
  d._screen_off_started = True
  d._override_interactive_timeout = None
  d._interaction_time = 0  # long timed out: Turn Off Comma Screen has switched the screen off
  d._prev_timed_out = True
  d._interactive_timeout_callbacks = []
  d._wake_touch_active = False
  d._display_preferences = DisplayPreferences(turn_off_screen=True)
  d._set_awake = Mock()
  return d


def ui(last_heard: float, recv_frame: int = 500, started_frame: int = 100, started: bool = True):
  sm = NS(valid={}, alive={}, recv_frame={'selfdriveState': recv_frame}, recv_time={'selfdriveState': last_heard})
  return NS(started=started, ignition=True, chestnut_loading=False, sm=sm, started_frame=started_frame)


def wake(state, pc=False):
  d = device()
  with (
    patch.object(native, 'ui_state', state),
    patch.object(native, 'PC', pc),
    patch.object(native.gui_app, '_mouse_events', []),
    patch.object(native.time, 'monotonic', return_value=NOW),
  ):
    stalled = d._selfdrive_stalled()
    d._update_wakefulness()
  return stalled, d._set_awake.call_args[0][0]


@pytest.mark.parametrize('silent_for,expected', [(1.0, False), (4.9, False), (5.5, True), (30.0, True)])
def test_screen_wakes_once_selfdrive_has_been_silent_past_the_alert_timeout(silent_for, expected):
  assert wake(ui(last_heard=NOW - silent_for)) == (expected, expected)


def test_not_heard_since_going_onroad_is_waiting_to_start_not_a_stall():
  assert wake(ui(last_heard=0.0, recv_frame=50, started_frame=100)) == (False, False)


def test_offroad_and_pc_never_count_as_a_stall():
  assert wake(ui(last_heard=0.0, started=False))[0] is False
  assert wake(ui(last_heard=0.0), pc=True)[0] is False


def test_missing_fields_are_not_a_stall():
  state = NS(started=True, ignition=True, chestnut_loading=False, sm=NS(valid={}, alive={}))
  assert wake(state) == (False, False)
