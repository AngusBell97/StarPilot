"""Glue between the screen-sleep logic (Device) and the Sleep Eyes engine, car watcher and view."""
from __future__ import annotations

import time

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.ui.sleep_eyes.car import CarWatcher, wakes_screen
from openpilot.starpilot.ui.sleep_eyes.engine import Engine

PARKED_LIMIT_S = 600       # parked eyes doze for up to 10 minutes, then the screen switches off as before
BRIGHTNESS_DRIVING = 50    # caps while the eyes show (percent)
BRIGHTNESS_PARKED = 25
EYES_FPS = 20              # the whole UI loop runs at this rate while the eyes show


class SleepEyesController:
  def __init__(self):
    from openpilot.starpilot.ui.sleep_eyes.view import SleepEyesView
    self.engine = Engine()
    self.watcher = CarWatcher()
    self.view = SleepEyesView(self.engine)
    self.sticky = False
    self.parked_since: float | None = None
    self.limit_hit = False
    self._started_prev: bool | None = None
    self._logged_want: bool | None = None
    self._slow = False

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
    if self._started_prev is not None and started != self._started_prev:
      # Ignition off while the eyes show: keep them (they doze off). Ignition on: back to the normal timing.
      self.sticky = self.visible and not started
    self._started_prev = started
    if touched or wake_alert or blocked:
      self.sticky = False
      self.parked_since = None
      return False
    want = timed_out or self.sticky
    if want and not started:
      if self.parked_since is None:
        self.parked_since = now
      elif now - self.parked_since > PARKED_LIMIT_S:
        self.sticky = False
        self.limit_hit = True  # switch straight off, no animation back to the home screen
        return False
    elif started:
      self.parked_since = None
    return want

  def update(self, gui_app, sm, *, started: bool, want: bool, urgent: bool, pure_black: bool) -> None:
    engine, view = self.engine, self.view
    engine.pure_black = pure_black
    urgent = urgent or self.limit_hit
    self.watcher.update(engine, time.monotonic() * 1000, sm, started)
    if want != self._logged_want:
      self._logged_want = want
      cloudlog.info(f"sleep eyes: {'show' if want else 'hide'} (started={started}, urgent={urgent}, phase={view.phase})")
    if want:
      if not view.visible or view.phase in ('capture_out', 'morph_out', 'pix_out'):
        engine.reset_timers(time.monotonic() * 1000)
        view.enter()
      gui_app.set_screen_override(view)
    elif view.visible:
      if urgent or engine.current != 'ignOn':  # let a wake-up finish before switching to the driving view
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
