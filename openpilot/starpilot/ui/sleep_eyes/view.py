"""Draws Sleep Eyes in place of the normal UI, with the pixelating transitions to and from the driving view."""
from __future__ import annotations

import time

from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.ui.raylib import rl
from openpilot.starpilot.ui.sleep_eyes.engine import BG_PURPLE, C, R, Engine
from openpilot.system.ui.lib.application import gui_app

STEP_MS = 45          # engine updates at most ~20 Hz (the UI runs at 20 fps while the eyes show)
PIXELATE_IN_MS = 800
PIXELATE_OUT_MS = 500
COLOR_CACHE_LIMIT = 4096


def _now_ms() -> float:
  return time.monotonic() * 1000


class SleepEyesView:
  """Phases: off -> capture_in -> pix_in -> eyes -> capture_out -> morph_out -> pix_out -> done."""

  def __init__(self, engine: Engine):
    self.engine = engine
    self.phase = 'off'
    self._t0 = 0.0
    self._last_step = -1e9
    self._cells: list = []
    self._levels: list = []
    self._colors: dict = {}

  # --- control from Device ------------------------------------------------------------------------------------
  @property
  def visible(self) -> bool:
    return self.phase not in ('off', 'done')

  @property
  def finished(self) -> bool:
    return self.phase == 'done'

  @property
  def needs_stack(self) -> bool:
    # Only the capture frames draw the normal UI (to grab it); otherwise its offscreen work is skipped.
    return self.phase in ('capture_in', 'capture_out')

  def enter(self, boot: bool = False) -> None:
    if self.phase in ('off', 'done'):
      self.engine.reset_display()
    if boot and self.phase in ('off', 'done'):
      # Start-up: the screen is black already, so no capture; the eyes power on from black.
      self.engine.exit_done = False
      self.engine.play('boot', _now_ms(), force=True)
      self.phase = 'eyes'
    elif self.phase in ('off', 'done'):
      self.engine.exit_done = False
      self.phase = 'capture_in'
    elif self.phase in ('capture_out', 'morph_out', 'pix_out'):
      # Timed out again while leaving: go straight back to the eyes.
      self._free_levels()
      self.engine.play('morphIn', _now_ms(), force=True)
      self.phase = 'eyes'

  def leave(self, *, animate: bool) -> None:
    if not animate:
      self._free_levels()
      self.phase = 'done'
    elif self.phase in ('eyes', 'pix_in', 'capture_in'):
      self.phase = 'capture_out'

  def reset(self) -> None:
    self._free_levels()
    self.phase = 'off'

  # --- capture and pixel levels -----------------------------------------------------------------------------
  def _grid(self, rect: rl.Rectangle) -> tuple[int, int, int, int]:
    cell = max(2, int(min(rect.width / C, rect.height / R)))
    gap = max(1, cell // 9)
    return cell, gap, int(rect.x + (rect.width - cell * C) // 2), int(rect.y + (rect.height - cell * R) // 2)

  def _free_levels(self) -> None:
    for texture in self._levels:
      try:
        rl.unload_texture(texture)
      except Exception:
        pass
    self._levels = []

  def _capture(self, rect: rl.Rectangle) -> None:
    """Grab what was just drawn, build coarser and coarser copies, and sample the 40 x 18 grid colours."""
    self._free_levels()
    if gui_app._render_texture is not None:  # scaled PC window: the screen read would not match
      self.engine.scene_cells = None
      return
    try:
      rl.rl_draw_render_batch_active()
      image = rl.load_image_from_screen()
      cell = self._grid(rect)[0]
      blocks = sorted({max(2, cell // 16), max(2, cell // 8), max(3, cell // 4), max(4, cell // 2)})
      width, height = image.width, image.height
      for block in blocks:
        rl.image_resize(image, max(C, width // block), max(R, height // block))
        texture = rl.load_texture_from_image(image)
        rl.set_texture_filter(texture, rl.TEXTURE_FILTER_POINT)
        self._levels.append(texture)
      rl.image_resize(image, C, R)
      data = image.to_bytes(C * R * 4)
      texture = rl.load_texture_from_image(image)
      rl.set_texture_filter(texture, rl.TEXTURE_FILTER_POINT)
      self._levels.append(texture)
      rl.unload_image(image)
      self.engine.scene_cells = [(data[k * 4], data[k * 4 + 1], data[k * 4 + 2]) for k in range(C * R)]
    except Exception:
      cloudlog.exception("sleep eyes: screen capture failed")
      self._free_levels()
      self.engine.scene_cells = None

  def _draw_level(self, rect: rl.Rectangle, progress: float) -> None:
    """progress 0 = sharpest copy, 1 = the 40 x 18 grid. The picture eases from the full screen into the grid's
    rectangle and the grid's gaps fade in, so the last step lines up exactly with the eyes' squares."""
    if not self._levels:
      return
    progress = min(max(progress, 0.0), 1.0)
    index = min(len(self._levels) - 1, int(progress * len(self._levels)))
    texture = self._levels[index]
    cell, gap, ox, oy = self._grid(rect)
    k = progress * progress * (3 - 2 * progress)
    dest = rl.Rectangle(rect.x + (ox - rect.x) * k, rect.y + (oy - rect.y) * k,
                        rect.width + (cell * C - rect.width) * k, rect.height + (cell * R - rect.height) * k)
    rl.draw_texture_pro(texture, rl.Rectangle(0, 0, texture.width, texture.height), dest, rl.Vector2(0, 0), 0.0, rl.WHITE)
    alpha = int(255 * min(1.0, max(0.0, (progress - 0.45) / 0.55)))
    if alpha:
      # Gap lines follow the zooming picture: a 40 x 18 lattice over the destination rectangle.
      lines = rl.Color(0, 0, 0, alpha)
      cw, ch = dest.width / C, dest.height / R
      g = max(1, round(gap * cw / cell))
      for i in range(1, C + 1):
        rl.draw_rectangle(int(dest.x + i * cw) - g, int(dest.y), g, int(dest.height), lines)
      for j in range(1, R + 1):
        rl.draw_rectangle(int(dest.x), int(dest.y + j * ch) - g, int(dest.width), g, lines)

  # --- drawing ----------------------------------------------------------------------------------------------
  def _color(self, c) -> rl.Color:
    color = self._colors.get(c)
    if color is None:
      if len(self._colors) > COLOR_CACHE_LIMIT:
        self._colors.clear()
      color = self._colors[c] = rl.Color(c[0], c[1], c[2], 255)
    return color

  def _draw_grid(self, rect: rl.Rectangle) -> None:
    cell, gap, ox, oy = self._grid(rect)
    size = cell - gap
    color = self._color
    background = None if self.engine.pure_black else BG_PURPLE
    if background is not None:
      # One fill plus black gap lines draws every unlit square; only lit squares are drawn one by one.
      rl.draw_rectangle(ox, oy, cell * C, cell * R, color(background))
      for i in range(1, C + 1):
        rl.draw_rectangle(ox + i * cell - gap, oy, gap, cell * R, rl.BLACK)
      for j in range(1, R + 1):
        rl.draw_rectangle(ox, oy + j * cell - gap, cell * C, gap, rl.BLACK)
    for k, c in enumerate(self._cells):
      if c != background and (c[0] or c[1] or c[2]):
        rl.draw_rectangle(ox + (k % C) * cell, oy + (k // C) * cell, size, size, color(c))

  def _step(self, now: float) -> None:
    if now - self._last_step >= STEP_MS or not self._cells:
      self._last_step = now
      self._cells = self.engine.step(now)

  def render(self, rect: rl.Rectangle, draw_stack) -> None:
    now = _now_ms()
    phase = self.phase
    if phase in ('off', 'done'):
      draw_stack(rect)
      return
    if phase == 'capture_in':
      draw_stack(rect)
      self._capture(rect)
      self.phase, self._t0 = 'pix_in', now
      return
    if phase == 'pix_in':
      k = (now - self._t0) / PIXELATE_IN_MS
      if k < 1 and self._levels:
        self._draw_level(rect, k)
        return
      self.engine.play('morphIn', now, force=True)
      self.phase = phase = 'eyes'
    if phase == 'capture_out':
      draw_stack(rect)
      self._capture(rect)
      rl.clear_background(rl.BLACK)
      self.engine.exit_done = False
      self.engine.play('morphOut', now, force=True)
      self.phase = phase = 'morph_out'
    if phase in ('eyes', 'morph_out'):
      self._step(now)
      self._draw_grid(rect)
      if phase == 'morph_out' and self.engine.exit_done:
        self.phase, self._t0 = 'pix_out', now
      return
    if phase == 'pix_out':
      k = (now - self._t0) / PIXELATE_OUT_MS
      if k < 1 and self._levels:
        self._draw_level(rect, 1 - k)
        return
      self._free_levels()
      self.phase = 'done'
      draw_stack(rect)
