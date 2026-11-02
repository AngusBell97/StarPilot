"""Frozen torque arc dimensions and cap-inclusive logical-pixel bounds."""

import math

TORQUE_ANGLE_SPAN = 12.7
RADIUS = 1200
MIN_HEIGHT, MAX_HEIGHT = 14, 56
MIN_OFFSET, MAX_OFFSET = 22, 26
CAP_RADIUS = 7
LARGE_CAP_RADIUS = 3 * CAP_RADIUS


def maximum_footprint(x, y, width, height, screen_width):
  scale = height / 240 * (width / screen_width)
  cap_radius = LARGE_CAP_RADIUS if screen_width == 2160 else CAP_RADIUS
  half_angle = math.radians(TORQUE_ANGLE_SPAN / 2)
  radius = RADIUS * scale
  center_x = x + width / 2 + 8
  extent_x = (radius + MAX_HEIGHT * scale) * math.sin(half_angle) + cap_radius * (1 - math.sin(half_angle))
  left, right = math.floor(center_x - extent_x), math.ceil(center_x + extent_x)
  top = math.floor(y + height - (MAX_OFFSET + MAX_HEIGHT) * scale)
  bottom = math.ceil(y + height - MIN_OFFSET * scale + (radius + cap_radius) * (1 - math.cos(half_angle)))
  return left, top, right - left, bottom - top


# Fixed lane-bar envelope above the maximum torque-bar expansion, including its percentage.
def lane_bar_geometry(x, y, width, height, screen_width):
  scale = height / 240 * width / screen_width
  radius, thickness = RADIUS * scale, 7 * scale
  blue_radius = radius + (MAX_HEIGHT + 10) * scale + thickness / 2
  half_width = (radius + MIN_HEIGHT * scale / 2) * math.sin(math.radians(TORQUE_ANGLE_SPAN / 2))
  half_span = math.degrees(math.asin(half_width / blue_radius))
  return x + width / 2 + 8, y + height + radius - MAX_OFFSET * scale, blue_radius, thickness, half_span, scale


def lane_bar_footprint(x, y, width, height, screen_width):
  cx, cy, radius, thickness, span, scale = lane_bar_geometry(x, y, width, height, screen_width)
  half_width = (radius + thickness / 2) * math.sin(math.radians(span))
  left, right = math.floor(cx - half_width), math.ceil(cx + half_width)
  top = math.floor(cy - radius - thickness / 2 - 20 * scale)
  bottom = math.ceil(cy - (radius - thickness / 2) * math.cos(math.radians(span)) + scale)
  return left, top, right - left, bottom - top
