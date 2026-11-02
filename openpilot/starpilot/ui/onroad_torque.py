"""Shared StarPilot torque bar with caller-qualified utilization."""

import math
from functools import lru_cache
import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.starpilot.lateral.lane_feedback import BLUE, DIRECTION_EPSILON, MAX_CORRECTION, STOCK_CORRECTION
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.starpilot.ui.onroad_state import OnroadState
from openpilot.starpilot.ui.presentation import BitmapFonts, FontRole
from openpilot.starpilot.ui.onroad_customization import offset as widget_offset, placement, LANE_BAR_WIDGET
from openpilot.starpilot.ui.onroad_torque_geometry import (
  lane_bar_geometry, CAP_RADIUS, LARGE_CAP_RADIUS, MAX_HEIGHT, MAX_OFFSET, MIN_HEIGHT, MIN_OFFSET, RADIUS, TORQUE_ANGLE_SPAN,
)

from openpilot.starpilot.ui.onroad_torque_draw import TorqueBarPainter

class TorqueBarWidget:
  def __init__(self):
    self._torque_filter = FirstOrderFilter(0.0, 0.1, 1 / 60)
    self._alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / 60)
    self._drive_frame = None
    self._painter = TorqueBarPainter()

  def prepare(self) -> None:
    self._painter.prepare()

  def render(self, rect: rl.Rectangle, state: OnroadState, screen_width: float) -> None:
    drive_frame = getattr(state, 'torque_drive_frame', None)
    if drive_frame != self._drive_frame:
      self._torque_filter.x = self._alpha_filter.x = 0.0
      self._drive_frame = drive_frame
    profile = 'large' if screen_width == 2160 else 'compact'
    enabled = placement(state.customization, profile, 'torque_bar')['enabled']
    if not state.lateral_active or not getattr(state, 'torque_source_available', True):
      self._torque_filter.x = 0.0
      self._alpha_filter.x = 0.0
      return
    utilization = max(-1.0, min(1.0, state.torque_utilization))
    torque = self._torque_filter.update(utilization)
    alpha = self._alpha_filter.update(1.0)
    if not enabled:
      return
    cap_radius = LARGE_CAP_RADIUS if profile == 'large' else CAP_RADIUS
    self._painter.render(rect, torque, alpha, screen_width, cap_radius, widget_offset(state.customization, profile, 'torque_bar'))

  def close(self) -> None:
    self._painter.close()


@lru_cache(maxsize=128)
def _lane_bar_points(radius, thickness, half_span, start, end):
  """Rounded ribbon inside the existing arc bounds, clipped to the signed fill."""
  cap = math.degrees(thickness / (2 * radius))
  straight = half_span - cap
  tips = np.concatenate((np.linspace(-half_span, -straight, 9), np.linspace(straight, half_span, 9)))
  angles = np.unique(np.concatenate((np.linspace(start, end, 33), np.clip(tips, start, end))))
  taper = np.maximum(0., np.abs(angles) - straight) / cap
  half = thickness / 2 * np.sqrt(np.maximum(0., 1 - taper ** 2))
  theta = np.deg2rad(270 + angles)
  unit = np.column_stack((np.cos(theta), np.sin(theta)))
  points = np.vstack((unit * (radius + half)[:, None], (unit * (radius - half)[:, None])[::-1])).astype(np.float32)
  points.flags.writeable = False
  return points


@lru_cache(maxsize=128)
def _clean_lane_fill(radius, thickness, start, end):
  """Round both ends inside the exact signed extent, including tiny fills."""
  cap = min(math.degrees(thickness / (2 * radius)), (end - start) / 2)
  angles = np.unique(np.concatenate((np.linspace(start, start + cap, 9),
                                    np.linspace(start + cap, end - cap, 25),
                                    np.linspace(end - cap, end, 9))))
  taper = np.maximum(0., np.maximum(start + cap - angles, angles - (end - cap))) / cap
  half = min(thickness / 2, math.radians(cap) * radius) * np.sqrt(np.maximum(0., 1 - taper ** 2))
  theta = np.deg2rad(270 + angles)
  unit = np.column_stack((np.cos(theta), np.sin(theta)))
  points = np.vstack((unit * (radius + half)[:, None], (unit * (radius - half)[:, None])[::-1])).astype(np.float32)
  points.flags.writeable = False
  return points


def draw_lane_bar(rect: rl.Rectangle, state: OnroadState, screen_width: float, torque: float = 0.0,
                  fonts: BitmapFonts | None = None, *, projection: bool = False) -> None:
  """Signed lane request / live cap; fixed capacity track and no display filter."""
  profile = 'large' if screen_width != 536 else 'compact'
  if not state.customization.get("layouts", {}).get(profile, {}).get(LANE_BAR_WIDGET, {}).get("enabled", False):
    return
  sample = state.visual_preview is not None and state.visual_preview.layout_sample
  correction, limit = state.lane_correction, state.lane_correction_limit
  if (not (state.camera_available or sample) or not state.lateral_active or not math.isfinite(correction) or
      not math.isfinite(limit) or not STOCK_CORRECTION - 1e-8 <= limit <= MAX_CORRECTION + 1e-8 or
      not DIRECTION_EPSILON < abs(correction) <= limit + 1e-8 or
      rect.width <= 0 or rect.height <= 0 or screen_width <= 0):
    return
  fraction = max(-1.0, min(1.0, correction / limit))
  # Native sidebar changes translate the widget without resizing it. Projection has its own canvas geometry.
  frame = (rect.x, rect.y, rect.width, rect.height, screen_width) if projection else (
    (30, 30, 1800, 1020, 2160) if profile == 'large' else (0, 0, 476, 240, 536))
  cx, cy, blue_radius, thickness, half_span, scale = lane_bar_geometry(*frame)
  dx, dy = widget_offset(state.customization, profile, LANE_BAR_WIDGET)
  if profile == 'large' and not projection:
    dx += (rect.width - 1800) / 2
  center = rl.Vector2(cx + dx, cy + dy)
  offset = np.array([center.x, center.y], dtype=np.float32)
  start, end = half_span * min(0.0, fraction), half_span * max(0.0, fraction)
  fill = (_clean_lane_fill(blue_radius, thickness, start, end) if state.lane_clean_arc else
          _lane_bar_points(blue_radius, thickness, half_span, start, end)) + offset
  track = _lane_bar_points(blue_radius, thickness, half_span, -half_span, half_span) + offset
  draw_polygon(rect, track, color=rl.Color(*((220, 227, 234) if state.lane_clean_arc else BLUE), 63))
  draw_polygon(rect, fill, color=rl.Color(*BLUE, 255 if state.lane_clean_arc else 229))
  if state.lane_clean_arc:
    # The neutral origin mark stays within the existing widget footprint.
    marker = np.array([[-.5, -1], [.5, -1], [.5, 1], [-.5, 1]], dtype=np.float32)
    marker *= [scale, thickness / 2 + scale]
    marker += [center.x, center.y - blue_radius]
    draw_polygon(rect, marker, color=rl.Color(220, 227, 234, 140))
  if fonts is not None and not state.lane_clean_arc:
    text, size = f"{abs(fraction):.0%}", 16.0 * scale
    bounds = fonts.measure(text, FontRole.SEMI_BOLD, size)
    x, y = center.x - bounds.width / 2, center.y - blue_radius - thickness / 2 - bounds.height - 3.0 * scale
    fonts.draw(text, FontRole.SEMI_BOLD, size, x + scale, y + scale, rl.Color(0, 0, 0, 180))
    fonts.draw(text, FontRole.SEMI_BOLD, size, x, y, rl.Color(*BLUE, 255))
