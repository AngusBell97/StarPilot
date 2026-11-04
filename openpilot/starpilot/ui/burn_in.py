"""Burn-in protection for the comma screen.

Pixel orbit: the whole picture moves by at most a couple of pixels, one pixel
at a time, so fixed edges (text, icons, the border) never sit on exactly the
same pixels for long. GuiApplication applies the offset; this module only
says where the picture should be.

Static dimming: HUD widgets are drawn inside `dimmer.scope(key)`. The scope
records what the widget drew (texts, positions, colours). When that has not
changed for DIM_AFTER_S the widget's colours fade to DIM_LEVEL of their
opacity; any change brings it straight back to full brightness.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import math
import time

from openpilot.starpilot.ui.raylib import rl


ORBIT_STEP_S = 45.0
ORBIT_AMPLITUDE_COMPACT = 2
ORBIT_AMPLITUDE_LARGE = 4

DIM_AFTER_S = 10.0
DIM_FADE_S = 20.0
DIM_LEVEL = 0.75
# A widget that was not drawn for this long (an alert covered it, a page was
# open) counts as new when it comes back, so it returns at full brightness.
DIM_GAP_S = 0.5

# Draw calls used by the onroad HUD. Each one is swapped for a recording,
# dimming wrapper only while a scope is open.
DRAW_FUNCTIONS = (
  "draw_circle", "draw_circle_gradient", "draw_circle_lines", "draw_circle_v", "draw_ellipse_lines",
  "draw_line", "draw_line_ex", "draw_rectangle", "draw_rectangle_gradient_ex", "draw_rectangle_gradient_h",
  "draw_rectangle_gradient_v", "draw_rectangle_lines", "draw_rectangle_lines_ex", "draw_rectangle_rec",
  "draw_rectangle_rounded", "draw_rectangle_rounded_lines_ex", "draw_ring", "draw_spline_segment_bezier_cubic",
  "draw_text_ex", "draw_texture_ex", "draw_texture_pro", "draw_texture_rec", "draw_texture_v", "draw_triangle",
  "draw_triangle_strip", "draw_triangle_fan", "draw_poly",
  # GuiApplication keeps the unpatched text call here and BitmapFonts uses it.
  "_orig_draw_text_ex",
)


def orbit_path(amplitude: int) -> tuple[tuple[int, int], ...]:
  """Every offset within +-amplitude, one pixel apart, there and back."""
  span = range(-amplitude, amplitude + 1)
  forward = [(x if row % 2 == 0 else -x, y) for row, y in enumerate(span) for x in span]
  return tuple(forward + forward[-2:0:-1]) if len(forward) > 1 else tuple(forward)


_PATHS: dict[int, tuple[tuple[int, int], ...]] = {}


def orbit_offset(now_s: float, amplitude: int, step_s: float = ORBIT_STEP_S) -> tuple[int, int]:
  if amplitude <= 0 or step_s <= 0 or not math.isfinite(now_s):
    return 0, 0
  path = _PATHS.get(amplitude)
  if path is None:
    path = _PATHS[amplitude] = orbit_path(amplitude)
  return path[int(now_s // step_s) % len(path)]


def dim_level(static_s: float) -> float:
  """Opacity factor for a widget that has looked the same for static_s seconds."""
  if static_s < DIM_AFTER_S:
    return 1.0
  return 1.0 - (1.0 - DIM_LEVEL) * min(1.0, (static_s - DIM_AFTER_S) / DIM_FADE_S)


# Keyword names raylib uses for colour arguments.
_COLOR_KEYWORDS = frozenset(("color", "tint", "color1", "color2", "color3", "color4", "top_left", "bottom_left",
                             "top_right", "bottom_right", "left", "right", "top", "bottom", "inner_color", "outer_color"))


def _is_color(value) -> bool:
  color_type = getattr(rl, "Color", None)
  return color_type is not None and isinstance(value, color_type)


def dim_color(color, factor: float):
  """The same colour with its opacity scaled by factor (Color, tuple or list)."""
  if factor >= 1.0:
    return color
  try:
    r, g, b, a = color.r, color.g, color.b, color.a
  except AttributeError:
    try:
      r, g, b, a = color
    except (TypeError, ValueError):
      return color
  return rl.Color(int(r), int(g), int(b), max(0, min(255, int(round(a * factor)))))


_STRUCT_FIELDS = ("r", "g", "b", "a", "x", "y", "z", "w", "width", "height", "id", "mipmaps", "format",
                  "baseSize", "base_size", "glyphCount", "glyph_count")


def _freeze(value):
  """A comparable, hashable summary of a draw argument (floats to whole pixels)."""
  if isinstance(value, bool) or value is None or isinstance(value, (int, str, bytes)):
    return value
  if isinstance(value, float):
    return round(value) if math.isfinite(value) else repr(value)
  if isinstance(value, (tuple, list)):
    return tuple(_freeze(item) for item in value)
  # Native raylib structs (Color, Vector2, Rectangle, Texture, Font...): summarise their plain fields.
  fields = tuple((name, getattr(value, name)) for name in _STRUCT_FIELDS if hasattr(value, name))
  if fields:
    return (type(value).__name__,) + tuple((name, _freeze(item)) for name, item in fields
                                           if isinstance(item, (bool, int, float, str)) or item is None)
  return type(value).__name__


class _Recorder:
  def __init__(self, factor: float):
    self.factor = factor
    self.calls: list = []
    self.depth = 0

  def wrap(self, name: str, original: Callable) -> Callable:
    def call(*args, **kwargs):
      if self.depth:
        # A patched call that forwards to another wrapped call: record and dim once.
        return original(*args, **kwargs)
      self.calls.append((name, _freeze(args), _freeze(sorted(kwargs.items())) if kwargs else ()))
      if self.factor < 1.0:
        args = [dim_color(arg, self.factor) if _is_color(arg) else arg for arg in args]
        for key, value in kwargs.items():
          if key in _COLOR_KEYWORDS and (_is_color(value) or isinstance(value, (tuple, list))):
            kwargs[key] = dim_color(value, self.factor)
      self.depth += 1
      try:
        return original(*args, **kwargs)
      finally:
        self.depth -= 1
    return call


class StaticDimmer:
  def __init__(self, clock: Callable[[], float] = time.monotonic):
    self.clock = clock
    self.enabled = False
    # key -> [signature, unchanged since, last drawn]
    self._widgets: dict[str, list] = {}
    self._current: _Recorder | None = None

  def set_enabled(self, enabled: bool) -> None:
    enabled = bool(enabled)
    if enabled != self.enabled:
      self._widgets.clear()
    self.enabled = enabled

  def level(self, key: str, now: float | None = None) -> float:
    entry = self._widgets.get(key)
    if entry is None:
      return 1.0
    now = self.clock() if now is None else now
    if now - entry[2] > DIM_GAP_S:
      return 1.0
    return dim_level(now - entry[1])

  def color(self, color):
    """Dim a colour drawn by a native call that bypasses the patched module functions."""
    recorder = self._current
    if recorder is None:
      return color
    recorder.calls.append(("color", _freeze(color)))
    return dim_color(color, recorder.factor)

  @contextmanager
  def scope(self, key: str) -> Iterator[None]:
    if not self.enabled or self._current is not None:
      yield
      return
    now = self.clock()
    recorder = _Recorder(self.level(key, now))
    saved = {}
    for name in DRAW_FUNCTIONS:
      original = getattr(rl, name, None)
      if original is not None:
        saved[name] = original
        setattr(rl, name, recorder.wrap(name, original))
    self._current = recorder
    try:
      yield
    finally:
      self._current = None
      for name, original in saved.items():
        setattr(rl, name, original)
    signature = tuple(recorder.calls)
    entry = self._widgets.get(key)
    if entry is None or entry[0] != signature or now - entry[2] > DIM_GAP_S:
      self._widgets[key] = [signature, now, now]
    else:
      entry[2] = now


dimmer = StaticDimmer()
