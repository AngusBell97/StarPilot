"""Sleep Eyes Only When Parked: awake eyes in Park after the parked timeout, the driving screen stays on out of Park."""
import random
import tempfile
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.ui import ui_state as native
from openpilot.starpilot.ui.display_owner import DisplayOwner, EYES_NEEDED_FOR_PARKED
from openpilot.starpilot.ui.display_preferences import (SCREEN_OFF, SLEEP_EYES, SLEEP_EYES_PARKED, DisplayPreferences,
                                                        read_preferences)
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
from openpilot.starpilot.ui.presentation import Profile
from openpilot.starpilot.ui.sleep_eyes import controller as controller_module
from openpilot.starpilot.ui.sleep_eyes.car import CarWatcher
from openpilot.starpilot.ui.sleep_eyes.engine import IDLE_LOUNGE, Engine
from openpilot.starpilot.ui.sleep_eyes.tests.test_car import finish, sm

NOW = 100.0


# --- the settings row -------------------------------------------------------------------------------------------
@pytest.fixture
def owner():
  with tempfile.TemporaryDirectory() as root:
    params = Params(root)
    yield params, DisplayOwner(params, lambda: True, gpu_capable=False)


def row(display, key):
  return next(r for r in display.snapshot(Profile.LARGE).rows if r.key == key)


def change(display, key, value):
  r = row(display, key)
  return display.apply(FeatureSettingsRequest(key, r.source, value, dependencies=r.dependencies))


def test_row_sits_under_sleep_eyes_and_needs_it(owner):
  params, display = owner
  keys = [r.key for r in display.snapshot(Profile.LARGE).rows]
  assert keys[keys.index(SLEEP_EYES) + 1] == SLEEP_EYES_PARKED
  assert change(display, SCREEN_OFF, "On")
  parked = row(display, SLEEP_EYES_PARKED)
  assert not parked.available and parked.reason == EYES_NEEDED_FOR_PARKED
  assert change(display, SLEEP_EYES, "On")
  assert row(display, SLEEP_EYES_PARKED).available
  assert change(display, SLEEP_EYES_PARKED, "On")
  assert read_preferences(params, large=False).sleep_eyes_parked_only
  # Saved On but not in effect while Sleep Eyes (or Turn Off Comma Screen) is off.
  assert change(display, SLEEP_EYES, "Off")
  assert not read_preferences(params, large=False).sleep_eyes_parked_only
  assert row(display, SLEEP_EYES_PARKED).value == "On"


# --- the car watcher: awake in Park -------------------------------------------------------------------------------
def test_in_park_the_eyes_stay_awake_and_lounge():
  engine, watcher = Engine(random.Random(5)), CarWatcher()
  watcher.update(engine, 0, sm('park'), True, parked_only=True)
  assert not engine.parked and engine.lounging
  now = finish(engine, 0, 60000)
  assert not engine.parked
  watcher.update(engine, now, sm('drive'), True, parked_only=True)
  assert not engine.lounging and engine.current != 'ignOn'


def test_lounging_uses_the_playful_idle_set_and_random_events():
  engine = Engine(random.Random(6))
  engine.lounging = True
  seen = set()
  now = 0.0
  while now < 240000:
    now += 50
    engine.step(now)
    if engine.current:
      seen.add(engine.current)
  assert seen & set(IDLE_LOUNGE) - {'blink'}
  assert seen & {'eclipse', 'aurora', 'fireworks', 'snow', 'rainbow', 'butterfly', 'bubbles', 'storm', 'constellation',
                 'shoot', 'meteor', 'fly', 'heart', 'planet', 'sneeze', 'hiccup'}


def test_without_the_option_park_still_dozes():
  engine, watcher = Engine(random.Random(7)), CarWatcher()
  watcher.update(engine, 0, sm('park'), True)
  assert engine.parked and not engine.lounging


# --- the controller: never out of Park ----------------------------------------------------------------------------
@pytest.fixture
def ctl(monkeypatch):
  clock = [1000.0]
  monkeypatch.setattr(controller_module.time, "monotonic", lambda: clock[0])
  c = controller_module.SleepEyesController.__new__(controller_module.SleepEyesController)
  c.view = NS(visible=False, finished=False)
  c.engine = NS(parked=False)
  c.sticky, c.asleep_since, c.slept_black, c.limit_hit, c._started_prev = False, None, False, False, None
  c.boot_pending, c._touched = False, False
  return c


def test_out_of_park_the_eyes_never_show_even_after_boot(ctl):
  ctl.boot_pending = True
  assert not ctl.decide(started=True, timed_out=True, blocked=False, touched=False, wake_alert=False, driving_on=True)
  assert not ctl.boot_pending and not ctl.sticky
  # Back in Park: only after the timeout, and with no stale boot animation queued.
  assert not ctl.decide(started=True, timed_out=False, blocked=False, touched=False, wake_alert=False)
  assert ctl.decide(started=True, timed_out=True, blocked=False, touched=False, wake_alert=False)


# --- the screen: on while driving, eyes after the parked timeout in Park -------------------------------------------
def device(prefs):
  d = native.Device.__new__(native.Device)
  d._ignition = True
  d._awake = True
  d._screen_off = False
  d._screen_off_started = True
  d._override_interactive_timeout = None
  d._interaction_time = 0  # long timed out
  d._prev_timed_out = True
  d._interactive_timeout_callbacks = []
  d._wake_touch_active = False
  d._display_preferences = prefs
  d._set_awake = Mock()
  d._update_sleep_eyes = Mock(return_value=False)
  return d


class FakeSM(NS):
  def __getitem__(self, key):
    return self.messages[key]


def state(gear):
  messages = FakeSM(valid={}, alive={}, recv_frame={'selfdriveState': 500}, recv_time={'selfdriveState': NOW},
                    messages={'carState': NS(gearShifter=gear)})
  return NS(started=True, ignition=True, chestnut_loading=False, sm=messages, started_frame=100)


def run(d, gear, *, was_in_park=None):
  if was_in_park is not None:
    d._was_in_park = was_in_park
  with (
    patch.object(native, 'ui_state', state(gear)),
    patch.object(native, 'PC', False),
    patch.object(native.gui_app, '_mouse_events', []),
    patch.object(native.time, 'monotonic', return_value=NOW),
  ):
    d._update_wakefulness()
    timeout = d.interactive_timeout
  return d._set_awake.call_args[0][0], d._update_sleep_eyes.call_args[0], timeout


PARKED_ONLY = DisplayPreferences(turn_off_screen=True, sleep_eyes=True, sleep_eyes_parked_only=True, parked_timeout=45,
                                 driving_timeout=5)


def test_driving_keeps_the_screen_on_and_the_eyes_away():
  awake, eyes_args, timeout = run(device(PARKED_ONLY), 'drive', was_in_park=False)
  assert awake and eyes_args[3] is True and timeout == 5


def test_in_park_the_screen_gives_way_to_the_eyes_after_the_parked_timeout():
  awake, eyes_args, timeout = run(device(PARKED_ONLY), 'park', was_in_park=True)
  assert not awake and eyes_args[2] is True and eyes_args[3] is False  # timed out: eyes (or off without them)
  assert timeout == 45


def test_shifting_into_park_restarts_the_timeout():
  d = device(PARKED_ONLY)
  run(d, 'park', was_in_park=False)
  assert d._interaction_time == NOW + 45


def test_without_the_option_turn_off_comma_screen_works_as_before():
  prefs = DisplayPreferences(turn_off_screen=True, sleep_eyes=True, driving_timeout=5)
  awake, eyes_args, timeout = run(device(prefs), 'drive', was_in_park=False)
  assert not awake and eyes_args[3] is False and timeout == 5
