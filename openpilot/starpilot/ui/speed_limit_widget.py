"""The latest limit, its attention animation and override mark on both screens."""

import time
from typing import cast

import pyray as rl

from openpilot.starpilot.ui.large_speed_geometry import LARGE_SIGN_SCALE, CONFIRMATION_SCALE, CONFIRMATION_PADDING
from openpilot.starpilot.ui.onroad_state import OnroadState, manual_slc_confirmation, speed_limit_bounds, slc_controls, slc_accept_available
from openpilot.starpilot.ui.presentation import BitmapFonts, FontRole, Profile
from openpilot.starpilot.ui.speed_limit_pulse import SpeedLimitPulse
from openpilot.starpilot.ui.speed_limit_shapes import SpeedLimitShapes
from openpilot.starpilot.ui.unified_speed_presentation import displayed_adjusted_limit_mps, positive, resolve_unified_speed


INK = rl.Color(241, 246, 243, 255)
MUTED = rl.Color(190, 208, 202, 255)
SOURCE_LABELS = {'dashboard': 'DASH', 'map': 'OSM', 'vision': 'VISION', 'mapbox': 'NAV'}


def draw_text(fonts, text, role, size, x, y, color=INK):
  if isinstance(color, (tuple, list)):
    color = rl.Color(*color)
  radius = 1.15 if fonts.profile == Profile.LARGE else 1.
  contour = rl.Color(6, 14, 10, round(color.a * .90))
  for dx, dy in ((-radius, 0), (radius, 0), (0, -radius), (0, radius)):
    fonts.draw(text, role, size, x + dx, y + dy, contour)
  fonts.draw(text, role, size, x, y + (1.5 if fonts.profile == Profile.LARGE else 1.2),
             rl.Color(7, 16, 12, round(color.a * .32)))
  fonts.draw(text, role, size, x, y, color)


def draw_line(profile, start, end, stroke, color):
  radius = 1.4 if profile == Profile.LARGE else 1.
  rl.draw_line_ex(start, end, stroke + 2 * radius, rl.Color(6, 14, 10, round(color.a * .87)))
  rl.draw_line_ex(start, end, stroke, color)


def draw_ink(fonts, text, role, size, x, top, color=INK):
  ink_top, ink_bottom = fonts.vertical_ink(text, role, size)
  draw_text(fonts, text, role, size, x, top - ink_top, color)
  return fonts.measure(text, role, size).width, ink_bottom - ink_top


def draw_caption(fonts, label, x, top, color=MUTED, *, unit=None, size=None, centered=False):
  large = fonts.profile == Profile.LARGE
  label_size = size or (28 if large else 24)
  unit_size, gap = (28, 12) if large else (20, 8)
  if centered:
    width = fonts.measure(label, FontRole.MEDIUM, label_size).width
    if unit:
      width += gap + fonts.measure(unit, FontRole.NORMAL, unit_size).width
    x -= width / 2
  width, height = draw_ink(fonts, label, FontRole.MEDIUM, label_size, x, top, color)
  if unit:
    unit_top, _ = fonts.vertical_ink(unit, FontRole.NORMAL, unit_size)
    # Exclude the p descender when aligning mph to the caption baseline.
    _, baseline = fonts.vertical_ink('mh' if unit == 'mph' else unit, FontRole.NORMAL, unit_size)
    unit_width, _ = draw_ink(fonts, unit, FontRole.NORMAL, unit_size, x + width + gap,
                             top + height - (baseline - unit_top), color)
    width += gap + unit_width
  return width, height


class SpeedLimitWidget:
  def __init__(self, fonts: BitmapFonts, profile: Profile):
    self.fonts = fonts
    self.profile = profile
    self.scale = LARGE_SIGN_SCALE if profile == Profile.LARGE else 1
    self.text_scale = self.scale * Profile.COMPACT.font_scale / profile.font_scale
    self.pulse = SpeedLimitPulse()
    self.shapes = SpeedLimitShapes(self.scale, headless_context=getattr(fonts, '_headless_context', False))

  def close(self):
    self.shapes.close()

  def render(self, state: OnroadState, *, defer_confirmation=False) -> None:
    bounds = speed_limit_bounds(self.profile, state)
    now = state.observed_ns / 1e9 if state.observed_ns else time.monotonic()
    self.pulse.update(state, now, visible=bounds is not None)
    if bounds is None:
      return
    if manual_slc_confirmation(self.profile, state):
      if not defer_confirmation:
        self.render_confirmation(state)
      return
    self._sign(state, bounds, now)

  def render_confirmation(self, state: OnroadState, pressed=None):
    bounds = speed_limit_bounds(self.profile, state)
    if bounds is None or not manual_slc_confirmation(self.profile, state):
      return
    now = state.observed_ns / 1e9 if state.observed_ns else time.monotonic()
    self.pulse.update(state, now)
    scale = CONFIRMATION_SCALE[self.profile]
    x, y, right, _ = bounds
    width = (right - x) / scale
    shown = resolve_unified_speed(state, confirmation=True)
    source = SOURCE_LABELS.get(shown.source, 'SPEED')
    white = rl.Color(242, 246, 244, 255)
    secondary = rl.Color(209, 222, 218, 255)

    def text_size(size):
      return size * scale * Profile.COMPACT.font_scale / self.profile.font_scale

    def ink_width(text, size, role=FontRole.MEDIUM):
      left, right = self.fonts.horizontal_ink(text, role, text_size(size))
      return (right - left) / scale

    def text(label, size, cx, cy, color=white, role=FontRole.MEDIUM):
      size = text_size(size)
      left, right = self.fonts.horizontal_ink(label, role, size)
      top, bottom = self.fonts.vertical_ink(label, role, size)
      self.fonts.draw(label, role, size, x + cx * scale - (left + right) / 2,
                      y + cy * scale - (top + bottom) / 2, color)

    def rounded(left, top, w, h, color, radius=8):
      rect = rl.Rectangle(x + left * scale, y + top * scale, w * scale, h * scale)
      rl.draw_rectangle_rounded(rect, 2 * radius / min(w, h), 12, color)

    if state.slc_prompt_open:
      self.shapes.confirmation(bounds, opened=True)
      target_size = 40
      nw, uw = ink_width(shown.proposal_text, target_size, FontRole.BOLD), ink_width(shown.unit, 20)
      context = source + (f' {shown.posted_text} {shown.offset_text}' if shown.offset_text else '')
      context_size = 20 * min(1, (width - 24 - nw - 6 - uw - 16) / max(1, ink_width(context, 20)))
      context_width = ink_width(context, context_size)
      start = (width - nw - 6 - uw - 16 - context_width) / 2
      text(shown.proposal_text, target_size, start + nw / 2, 24, white, FontRole.BOLD)
      nt, nb = self.fonts.vertical_ink(shown.proposal_text, FontRole.BOLD, text_size(target_size))
      ut, ub = self.fonts.vertical_ink(shown.unit, FontRole.MEDIUM, text_size(20))
      text(shown.unit, 20, start + nw + 6 + uw / 2, 24 + (nb - nt - ub + ut) / scale / 2, secondary)
      text(context, context_size, start + nw + 6 + uw + 16 + context_width / 2, 24, rl.Color(203, 197, 215, 255))
      for control, (fill, border, ink) in zip(slc_controls(self.profile, state), (
          ((35, 74, 57), (106, 153, 127), (224, 243, 232)),
          ((77, 46, 57), (173, 122, 134), (248, 229, 234))), strict=True):
        if not control.enabled:
          fill, border, ink = (43, 49, 48), (94, 105, 101), (161, 174, 168)
        left = (control.bounds[0] - x) / scale
        top = (control.bounds[1] - y) / scale
        w, h = (control.bounds[2] - control.bounds[0]) / scale, (control.bounds[3] - control.bounds[1]) / scale
        # Keep the full touch area around the shorter, visually balanced face.
        face_top, face_height = top + CONFIRMATION_PADDING, h - 2 * CONFIRMATION_PADDING
        rounded(left, face_top, w, face_height, rl.Color(*border, 255), 16)
        color = tuple(min(255, channel + 14) for channel in fill) if pressed == control.request else fill
        rounded(left + 1.5, face_top + 1.5, w - 3, face_height - 3, rl.Color(*color, 255), 14.5)
        text(control.label, 37, left + w / 2, top + h / 2, rl.Color(*ink, 255))
      return

    self.shapes.confirmation(bounds, strength=self.pulse.confirmation_strength(now))
    source_color = rl.Color(197, 212, 208, 255)
    label_width = ink_width(source, 22, FontRole.SEMI_BOLD)
    target = state.speed_limit.pending_adjusted_limit_mps
    if not positive(target):
      target = displayed_adjusted_limit_mps(state.speed_limit)
    direction = (-1 if target > state.speed_mps else 1) if target is not None and state.speed_mps is not None and target != state.speed_mps else 0
    start = (width - label_width - (19.2 if direction else 0)) / 2
    text(source, 22, start + label_width / 2, 22, source_color, FontRole.SEMI_BOLD)
    if direction:
      self.shapes.arrow('confirmation', x + (start + label_width + 12.6) * scale, y + 22.5 * scale,
                        direction, source_color, scale=1.1 * scale)
    nw = ink_width(shown.posted_text, 64, FontRole.BOLD)
    ow = ink_width(shown.offset_text, 32) if shown.offset_text else 0
    start = (width - nw - (10 + ow if ow else 0)) / 2
    # Align the posted value and offset at their visible lower edge.
    number_top, number_bottom = self.fonts.vertical_ink(shown.posted_text, FontRole.BOLD, text_size(64))
    text(shown.posted_text, 64, start + nw / 2, 84 - (number_bottom - number_top) / scale / 2, white, FontRole.BOLD)
    if ow:
      offset_text = cast(str, shown.offset_text)
      top, bottom = self.fonts.vertical_ink(offset_text, FontRole.MEDIUM, text_size(32))
      text(offset_text, 32, start + nw + 10 + ow / 2, 84 - (bottom - top) / scale / 2, secondary)
    label = 'New Limit' if slc_accept_available(state) else 'Unavailable'
    caption = ink_width(label, 30, FontRole.SEMI_BOLD)
    cx = (width - caption - 10) / 2
    accent = rl.Color(214, 188, 240, 255)
    text(label, 30, cx + caption / 2, 104, accent, FontRole.SEMI_BOLD)
    ax, ay = x + (cx + caption + 8) * scale, y + 104 * scale
    rl.draw_line_ex(rl.Vector2(ax - 2 * scale, ay - 4 * scale), rl.Vector2(ax + 2 * scale, ay), 1.5 * scale, accent)
    rl.draw_line_ex(rl.Vector2(ax + 2 * scale, ay), rl.Vector2(ax - 2 * scale, ay + 4 * scale), 1.5 * scale, accent)
    if state.slc_wheel_confirmation_available:
      accept, reject = ink_width('Accept', 26), ink_width('Reject', 26)
      start = (width - 18 - 4 - accept - 10 - 18 - 4 - reject) / 2
      for left, plus in ((start, True), (start + 18 + 4 + accept + 10, False)):
        self.shapes.wheel_key(plus, x + (left + 9) * scale, y + 135.5 * scale, scale=scale)
      hint_ink = rl.Color(210, 216, 216, 255)
      text('Accept', 26, start + 22 + accept / 2, 135.5, hint_ink if slc_accept_available(state) else MUTED)
      text('Reject', 26, start + 18 + 4 + accept + 10 + 22 + reject / 2, 135.5, hint_ink)

  def _sign(self, state: OnroadState, bounds, now: float) -> None:
    shown, obs = resolve_unified_speed(state), state.speed_limit
    heading = SOURCE_LABELS.get(shown.source, 'SPEED')
    x, top = bounds[0] + 12, bounds[1] + 12
    scale = self.scale
    pending = shown.pending and not obs.automatic_adoption_pending
    target = displayed_adjusted_limit_mps(obs) if pending else None
    arrow_direction = 0
    if target is not None and state.speed_mps is not None and target != state.speed_mps:
      arrow_direction = -1 if target > state.speed_mps else 1
    white = self.pulse.color(rl.Color(244, 247, 245, 255), now)
    secondary = self.pulse.color(rl.Color(223, 232, 227, 255), now)

    def ink(text, role, size):
      return (*self.fonts.horizontal_ink(text, role, size * self.text_scale),
              *self.fonts.vertical_ink(text, role, size * self.text_scale))

    def centered(text, role, size, ink_top, color, center_x=x + 59 * scale):
      if isinstance(color, (tuple, list)): color = rl.Color(*color)
      left, right, a, b = ink(text, role, size)
      origin = center_x - (left + right) / 2
      self.fonts.draw(text, role, size * self.text_scale, origin, ink_top - a, color)
      return origin + left, ink_top, origin + right, ink_top + b - a

    if state.appearance.use_vienna_sign:
      cx, cy = x + 59 * scale, top + 60 * scale
      rl.draw_circle_v(rl.Vector2(cx, cy), 52 * scale, rl.WHITE)
      rl.draw_ring(rl.Vector2(cx, cy), 42 * scale, 52 * scale, 0, 360, 96,
                   self.pulse.color(rl.Color(201, 34, 49, 255), now))
      color = rl.BLACK if pending else self.pulse.color(rl.BLACK, now)
      size = (48 if pending else 52) if len(shown.posted_text) <= 2 else 42
      number_center = cy + 8 * scale
      centered(heading, FontRole.SEMI_BOLD, 14, cy - 30 * scale, color, cx - (7 * scale if arrow_direction else 0))
      while True:
        left, right, a, b = ink(shown.posted_text, FontRole.BOLD, size)
        if ((right - left) / 2) ** 2 + (abs(number_center - cy) + (b - a) / 2) ** 2 <= (41 * scale) ** 2 or size <= 28:
          break
        size -= 2
      number = centered(shown.posted_text, FontRole.BOLD, size, number_center - (b - a) / 2, color)
      if shown.offset_text:
        left, right, a, b = ink(shown.offset_text, FontRole.MEDIUM, 30)
        pill_width = max(62 * scale, right - left + 20 * scale)
        pill_top, pill_height = top + 120 * scale, b - a + 10 * scale
        rl.draw_rectangle_rounded(rl.Rectangle(cx - pill_width / 2, pill_top, pill_width, pill_height),
                                  .45, 16, rl.Color(10, 17, 19, 230))
        centered(shown.offset_text, FontRole.MEDIUM, 30, pill_top + 5 * scale, secondary)
    else:
      if self.profile == Profile.COMPACT:
        rect = rl.Rectangle(x + 2, top, 116, 132 + (44 if shown.offset_text else 0))
        rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(rect.x + 8, rect.y + 8, rect.width - 16, rect.height - 16),
                                         .14, 16, 2, white)
        for label, size, label_top, role in ((heading, 20, 18, FontRole.SEMI_BOLD),
                                              ('LIMIT', 20, 36, FontRole.SEMI_BOLD),
                                              (shown.posted_text, 50 if len(shown.posted_text) <= 2 else 42, 66, FontRole.BOLD)):
          measured = self.fonts.measure(label, role, size)
          origin = rect.x + (rect.width - measured.width) / 2
          if arrow_direction and role == FontRole.SEMI_BOLD:
            origin -= 6  # Keep four pixels between the measured header ink and the pending arrow.
          self.fonts.draw(label, role, size, origin, rect.y + label_top, white)
          if label == shown.posted_text:
            number = origin, rect.y + label_top, origin + measured.width, rect.y + label_top + measured.height
        if shown.offset_text:
          measured = self.fonts.measure(shown.offset_text, FontRole.MEDIUM, 30)
          self.fonts.draw(shown.offset_text, FontRole.MEDIUM, 30, rect.x + (rect.width - measured.width) / 2,
                          rect.y + 118, secondary)
      else:
        rect = rl.Rectangle(x + 2 * scale, top + 2 * scale, 114 * scale, 128 * scale)
        rl.draw_rectangle_rounded(rect, .1, 16, rl.Color(10, 17, 19, 230))
        rl.draw_rectangle_rounded_lines_ex(rect, .1, 16, 2 * scale, white)
        for label, y in ((heading, top + 11 * scale), ('LIMIT', top + 27 * scale)):
          centered(label, FontRole.SEMI_BOLD, 20, y, white, x + (53 if arrow_direction else 59) * scale)
        number = centered(shown.posted_text, FontRole.BOLD, 58, top + (50 if shown.offset_text else 61) * scale, white)
        if shown.offset_text:
          centered(shown.offset_text, FontRole.MEDIUM, 36, top + 98 * scale, secondary)
    if arrow_direction:
      ax, ay = ((x + 82 * scale, top + 41 * scale) if state.appearance.use_vienna_sign else (x + 99 * scale, top + 26 * scale))
      arrow_color = color if state.appearance.use_vienna_sign else white
      self.shapes.arrow('compact' if state.appearance.use_vienna_sign else 'large', ax, ay, arrow_direction, arrow_color)
    # No cross over the limit when the set speed overrides it (removed 4 Nov 2026 at the driver's request).

