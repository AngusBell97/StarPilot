"""Sleep Eyes settings rows (greyed while Turn Off Comma Screen is on) and the sleep decision rules."""
import tempfile
from types import SimpleNamespace as NS

import pytest

from openpilot.common.params import Params
from openpilot.starpilot.ui.display_owner import DisplayOwner, EYES_OFF_BLOCKS_BLACK, SCREEN_OFF_BLOCKS_EYES
from openpilot.starpilot.ui.display_preferences import SCREEN_OFF, SLEEP_EYES, SLEEP_EYES_BLACK, read_preferences
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
from openpilot.starpilot.ui.presentation import Profile
from openpilot.starpilot.ui.sleep_eyes import controller as controller_module
from openpilot.starpilot.ui.sleep_eyes.car import orange_alert, wakes_screen


@pytest.fixture
def owner():
  with tempfile.TemporaryDirectory() as root:
    params = Params(root)
    yield params, DisplayOwner(params, lambda: True, gpu_capable=False)


def row(owner, key):
  return next(r for r in owner.snapshot(Profile.LARGE).rows if r.key == key)


def change(owner, key, value):
  r = row(owner, key)
  return owner.apply(FeatureSettingsRequest(key, r.source, value, dependencies=r.dependencies))


def test_rows_sit_next_to_turn_off_comma_screen_and_black_needs_eyes(owner):
  params, display = owner
  keys = [r.key for r in display.snapshot(Profile.LARGE).rows]
  assert keys[keys.index(SCREEN_OFF) + 1:keys.index(SCREEN_OFF) + 3] == [SLEEP_EYES, SLEEP_EYES_BLACK]
  assert row(display, SLEEP_EYES).available
  black = row(display, SLEEP_EYES_BLACK)
  assert not black.available and black.reason == EYES_OFF_BLOCKS_BLACK
  assert change(display, SLEEP_EYES, "On")
  assert row(display, SLEEP_EYES_BLACK).available
  assert change(display, SLEEP_EYES_BLACK, "On")
  prefs = read_preferences(params, large=True)
  assert prefs.sleep_eyes and prefs.sleep_eyes_black


def test_turn_off_comma_screen_greys_out_and_disables_sleep_eyes(owner):
  params, display = owner
  assert change(display, SLEEP_EYES, "On")
  stale = row(display, SLEEP_EYES)
  assert change(display, SCREEN_OFF, "On")
  eyes = row(display, SLEEP_EYES)
  assert not eyes.available and eyes.reason == SCREEN_OFF_BLOCKS_EYES and eyes.value == "On"
  assert not read_preferences(params, large=True).sleep_eyes  # saved On, but not in effect
  # A request made from a page opened before Turn Off Comma Screen changed is refused.
  params.put_bool(SLEEP_EYES, False)
  assert not display.apply(FeatureSettingsRequest(SLEEP_EYES, row(display, SLEEP_EYES).source, "On",
                                                  dependencies=stale.dependencies))
  assert change(display, SCREEN_OFF, "Off")
  assert change(display, SLEEP_EYES, "On")


def test_alert_routing():
  def ss(status, size, kind, text1="", text2=""):
    return NS(alertStatus=status, alertSize=size, alertType=kind, alertText1=text1, alertText2=text2)
  assert wakes_screen(ss("critical", "full", "x/permanent"))
  assert wakes_screen(ss("userPrompt", "full", "steerTempUnavailable/softDisable", "TAKE CONTROL IMMEDIATELY"))
  assert not wakes_screen(ss("userPrompt", "mid", "driverDistracted2/permanent"))
  assert orange_alert(ss("userPrompt", "mid", "driverDistracted2/permanent", "Pay Attention")) == "alAttn"
  assert orange_alert(ss("userPrompt", "mid", "driverUnresponsive2/permanent")) == "alWheel"
  assert orange_alert(ss("userPrompt", "mid", "steerSaturated/warning")) == "alSteer"
  assert orange_alert(ss("userPrompt", "small", "ldw/permanent")) == "alLdw"
  assert orange_alert(ss("userPrompt", "small", "", "Car Detected in Blindspot")) == "alBlind"
  assert orange_alert(ss("userPrompt", "small", "brakeHold/warning")) == "alResume"
  assert orange_alert(ss("userPrompt", "mid", "speedTooHigh/warning", "Speed Too High")) == "alTake"
  assert orange_alert(ss("userPrompt", "small", "", "Max Regen Being Used")) is None
  assert orange_alert(ss("normal", "small", "x/warning")) is None
  assert orange_alert(ss("userPrompt", "full", "x/softDisable")) is None


class FakeView:
  def __init__(self):
    self.visible = False

  @property
  def finished(self):
    return False


@pytest.fixture
def ctl(monkeypatch):
  clock = [1000.0]
  monkeypatch.setattr(controller_module.time, "monotonic", lambda: clock[0])
  c = controller_module.SleepEyesController.__new__(controller_module.SleepEyesController)
  c.view = FakeView()
  c.sticky, c.parked_since, c.limit_hit, c._started_prev = False, None, False, None
  c.boot_pending, c._touched = False, False
  return c, clock


def test_ignition_off_keeps_visible_eyes_and_touch_wakes(ctl):
  c, clock = ctl
  assert c.decide(started=True, timed_out=True, blocked=False, touched=False, wake_alert=False)
  c.view.visible = True
  # Ignition off resets the timeout, but the eyes stay and doze off.
  assert c.decide(started=False, timed_out=False, blocked=False, touched=False, wake_alert=False)
  assert not c.decide(started=False, timed_out=False, blocked=False, touched=True, wake_alert=False)
  assert not c.decide(started=False, timed_out=False, blocked=False, touched=False, wake_alert=False)


def test_alerts_and_blockers_beat_the_eyes(ctl):
  c, _ = ctl
  assert not c.decide(started=True, timed_out=True, blocked=False, touched=False, wake_alert=True)
  assert not c.decide(started=True, timed_out=True, blocked=True, touched=False, wake_alert=False)


def test_parked_eyes_stop_after_ten_minutes(ctl):
  c, clock = ctl
  c._started_prev = False
  assert c.decide(started=False, timed_out=True, blocked=False, touched=False, wake_alert=False)
  clock[0] += controller_module.PARKED_LIMIT_S + 1
  assert not c.decide(started=False, timed_out=True, blocked=False, touched=False, wake_alert=False)
  assert c.limit_hit


def test_boot_shows_the_eyes_until_a_tap_even_through_the_drive_starting(ctl):
  c, _ = ctl
  c.boot_pending = True
  assert c.decide(started=False, timed_out=False, blocked=False, touched=False, wake_alert=False)
  c.boot_pending = False  # update() clears it once the eyes are entered
  c.view.visible = True
  assert c.decide(started=True, timed_out=False, blocked=False, touched=False, wake_alert=False)
  assert not c.decide(started=True, timed_out=False, blocked=False, touched=True, wake_alert=False)
  assert not c.decide(started=True, timed_out=False, blocked=False, touched=False, wake_alert=False)
