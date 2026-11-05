"""Glue between the screen-sleep logic (Device) and the Sleep Eyes engine, car watcher and view."""
from __future__ import annotations

import time

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.ui.sleep_eyes.car import CarWatcher, wakes_screen
from openpilot.starpilot.ui.sleep_eyes.engine import Engine

BOOT_WINDOW_S = 30        # Sleep Eyes switched on within this long of the UI starting means the comma just booted
ASLEEP_LIMIT_S = 600       # after 10 minutes asleep (in Park or offroad) the screen goes fully black
FADE_TO_BLACK_S = 4        # the sleeping eyes fade out over the last seconds before that
BOOT_GRACE_S = 25          # right after boot the comma is briefly offroad: don't doze off for that
BRIGHTNESS_DRIVING = 50    # caps while the eyes show (percent)
BRIGHTNESS_PARKED = 25
EYES_FPS = 20              # the whole UI loop runs at this rate while the eyes show


class SleepEyesController:
  def __init__(self, booting: bool = False):
    from openpilot.starpilot.ui.sleep_eyes.view import SleepEyesView
    self.engine = Engine()
    self.watcher = CarWatcher(grace_until=(time.monotonic() + BOOT_GRACE_S) * 1000 if booting else 0.0)
    self.view = SleepEyesView(self.engine)
    self.sticky = False
    self.asleep_since: float | None = None
    self.slept_black = False   # the screen went black after the limit: come back with the power-on animation
    self.limit_hit = False
    self._started_prev: bool | None = None
    self._logged_want: bool | None = None
    self._slow = False
    self._touched = False
    self.boot_pending = booting

  @property
  def visible(self) -> bool:
    return self.view.visible

  @staticmethod
  def wake_alert(sm, started: bool) -> bool:
    if not started:
      return False
    try:
      return wakes_screen(sm['selfdriveState'])
    except Exception:
      return False

  def brightness_cap(self, started: bool) -> int:
    return BRIGHTNESS_DRIVING if started else BRIGHTNESS_PARKED

  def decide(self, *, started: bool, timed_out: bool, blocked: bool, touched: bool, wake_alert: bool) -> bool:
    """Whether the eyes should be on screen this frame. blocked = critical alert, eGPU hold or manual screen off."""
    now = time.monotonic()
    self.limit_hit = False
    self._touched = touched
    if self.boot_pending:
      self.sticky = True  # boot straight into the eyes; they stay until a tap
    if self._started_prev is not None and started != self._started_prev and not started and self.visible:
      self.sticky = True  # offroad while the eyes show: keep them (only seen if the comma stays powered)
    self._started_prev = started
    # The sleep timer runs while the eyes are asleep (in Park, or offroad); a tap or waking up restarts it.
    if touched or not self.engine.parked:
      self.asleep_since = None
    elif self.asleep_since is None:
      self.asleep_since = now
    if touched or wake_alert or blocked:
      self.sticky = False
      return False
    want = timed_out or self.sticky
    if want and self.asleep_since is not None and now - self.asleep_since > ASLEEP_LIMIT_S:
      self.sticky = False
      self.limit_hit = True  # straight to a black screen, no animation
      self.slept_black = True
      return False
    return want

  def update(self, gui_app, sm, *, started: bool, want: bool, urgent: bool) -> None:
    engine, view = self.engine, self.view
    urgent = urgent or self.limit_hit
    self.watcher.update(engine, time.monotonic() * 1000, sm, started)
    if self.asleep_since is not None:
      engine.dim = min(1.0, max(0.0, (ASLEEP_LIMIT_S - (time.monotonic() - self.asleep_since)) / FADE_TO_BLACK_S))
    else:
      engine.dim = 1.0
    if want != self._logged_want:
      self._logged_want = want
      cloudlog.info(f"sleep eyes: {'show' if want else 'hide'} (started={started}, urgent={urgent}, phase={view.phase})")
    if want:
      if not view.visible or view.phase in ('settle_out', 'capture_out', 'morph_out', 'pix_out'):
        engine.reset_timers(time.monotonic() * 1000)
        view.enter(boot=self.boot_pending or self.slept_black)
        self.boot_pending = self.slept_black = False
      gui_app.set_screen_override(view)
    elif view.visible:
      # Let a wake-up finish before switching to the driving view, unless it was a tap.
      if urgent or self._touched or engine.current != 'ignOn':
        view.leave(animate=not urgent)
    if not want and (view.finished or not view.visible):
      if gui_app.screen_override is view:
        gui_app.set_screen_override(None)
      view.reset()
    self._set_slow(gui_app, view.visible)

  def _set_slow(self, gui_app, slow: bool) -> None:
    if slow != self._slow:
      self._slow = slow
      gui_app.set_render_fps(EYES_FPS if slow else None)

  def shutdown(self, gui_app) -> None:
    if gui_app.screen_override is self.view:
      gui_app.set_screen_override(None)
    self.view.reset()
    self._set_slow(gui_app, False)
