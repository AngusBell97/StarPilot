from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from openpilot.starpilot.ui.raylib import rl
import pytest

from openpilot.common.params import Params
from openpilot.selfdrive.ui import ui_state as native
from openpilot.starpilot.ui import burn_in
from openpilot.starpilot.ui.burn_in import (
  DIM_AFTER_S, DIM_FADE_S, DIM_GAP_S, DIM_LEVEL, StaticDimmer, dim_level, orbit_offset, orbit_path,
)
from openpilot.starpilot.ui.display_owner import DisplayOwner
from openpilot.starpilot.ui.display_preferences import MASTER, PIXEL_SHIFT, STATIC_DIM, DisplayPreferences, read_preferences
from openpilot.starpilot.ui.feature_settings_state import row_change
from openpilot.starpilot.ui.presentation import Profile


class Clock:
  def __init__(self, now=100.0):
    self.now = now

  def __call__(self):
    return self.now


@pytest.mark.parametrize('amplitude', [1, 2, 4])
def test_orbit_covers_the_square_in_single_pixel_steps(amplitude):
  path = orbit_path(amplitude)
  side = 2 * amplitude + 1
  assert set(path) == {(x, y) for x in range(-amplitude, amplitude + 1) for y in range(-amplitude, amplitude + 1)}
  assert len(path) == 2 * side * side - 2
  for (x0, y0), (x1, y1) in zip(path, path[1:] + path[:1], strict=True):
    assert abs(x1 - x0) + abs(y1 - y0) == 1


def test_orbit_offset_holds_each_position_for_a_step():
  assert orbit_offset(0.0, 2, 45.0) == orbit_offset(44.9, 2, 45.0)
  assert orbit_offset(44.9, 2, 45.0) != orbit_offset(45.0, 2, 45.0)
  assert orbit_offset(123.0, 0) == (0, 0)
  assert orbit_offset(float('nan'), 2) == (0, 0)
  assert all(max(abs(v) for v in orbit_offset(t * 7.0, 2)) <= 2 for t in range(2000))


def test_dim_level_waits_then_fades_to_the_floor():
  assert dim_level(0) == 1.0
  assert dim_level(DIM_AFTER_S - 0.01) == 1.0
  assert dim_level(DIM_AFTER_S + DIM_FADE_S / 2) == pytest.approx(1 - (1 - DIM_LEVEL) / 2)
  assert dim_level(DIM_AFTER_S + DIM_FADE_S) == pytest.approx(DIM_LEVEL)
  assert dim_level(10_000) == pytest.approx(DIM_LEVEL)


def _alpha(color):
  return color.a if hasattr(color, 'a') else color[3]


def _frames(dimmer, clock, draw, seconds, *, step=0.1):
  for _ in range(int(round(seconds / step))):
    clock.now += step
    with dimmer.scope('max_speed'):
      draw()


def test_static_widget_dims_and_any_change_restores_it():
  drawn = []
  fake = lambda x, y, w, h, color: drawn.append(color)
  clock = Clock()
  dimmer = StaticDimmer(clock)
  dimmer.set_enabled(True)
  with patch.object(rl, 'draw_rectangle', fake, create=True):
    same = lambda: rl.draw_rectangle(10, 20, 30, 40, rl.Color(255, 255, 255, 200))
    _frames(dimmer, clock, same, DIM_AFTER_S - 1)
    assert _alpha(drawn[-1]) == 200
    _frames(dimmer, clock, same, 1 + DIM_FADE_S + 0.5)
    assert _alpha(drawn[-1]) == round(200 * DIM_LEVEL)
    # A new set speed (different text or geometry) is back at full brightness on the next frame.
    _frames(dimmer, clock, lambda: rl.draw_rectangle(10, 20, 31, 40, rl.Color(255, 255, 255, 200)), 0.2)
    assert _alpha(drawn[-1]) == 200
    # Sub-pixel jitter does not count as a change.
    jitter = iter(range(10**6))
    _frames(dimmer, clock, lambda: rl.draw_rectangle(10 + (next(jitter) % 2) * 0.2, 20, 31, 40,
                                                     rl.Color(255, 255, 255, 200)), DIM_AFTER_S + DIM_FADE_S + 1)
    assert _alpha(drawn[-1]) == round(200 * DIM_LEVEL)
  assert rl.draw_rectangle is not fake


def test_widget_returning_after_a_gap_starts_bright():
  drawn = []
  clock = Clock()
  dimmer = StaticDimmer(clock)
  dimmer.set_enabled(True)
  with patch.object(rl, 'draw_rectangle', lambda *a: drawn.append(a[-1]), create=True):
    same = lambda: rl.draw_rectangle(0, 0, 5, 5, rl.Color(0, 200, 0, 255))
    _frames(dimmer, clock, same, DIM_AFTER_S + DIM_FADE_S + 1)
    assert _alpha(drawn[-1]) < 255
    clock.now += DIM_GAP_S + 1  # an alert covered it
    _frames(dimmer, clock, same, 0.1)
    assert _alpha(drawn[-1]) == 255


def test_disabled_dimmer_leaves_draw_calls_alone():
  sentinel = object()
  dimmer = StaticDimmer(Clock())
  with patch.object(rl, 'draw_rectangle', sentinel, create=True):
    with dimmer.scope('border'):
      assert rl.draw_rectangle is sentinel
    dimmer.set_enabled(True)
    with dimmer.scope('border'):
      assert rl.draw_rectangle is not sentinel
    assert rl.draw_rectangle is sentinel


def test_forwarding_text_call_is_dimmed_once():
  seen = []
  def inner(font, text, position, size, spacing, tint):
    seen.append(tint)
  def outer(font, text, position, size, spacing, tint):
    return rl._orig_draw_text_ex(font, text, position, size, spacing, tint)
  clock = Clock()
  dimmer = StaticDimmer(clock)
  dimmer.set_enabled(True)
  with patch.object(rl, '_orig_draw_text_ex', inner, create=True), patch.object(rl, 'draw_text_ex', outer, create=True):
    for _ in range(int((DIM_AFTER_S + DIM_FADE_S + 1) / 0.5)):
      clock.now += 0.5
      with dimmer.scope('clock'):
        rl.draw_text_ex(None, '12:00', rl.Vector2(1, 2), 30, 0, rl.Color(255, 255, 255, 240))
  assert _alpha(seen[-1]) == round(240 * DIM_LEVEL)


def test_native_line_colour_helper():
  clock = Clock()
  dimmer = StaticDimmer(clock)
  white = rl.Color(255, 255, 255, 255)
  assert dimmer.color(white) is white  # no open scope
  dimmer.set_enabled(True)
  for _ in range(int((DIM_AFTER_S + DIM_FADE_S + 1) / 0.5)):
    clock.now += 0.5
    with dimmer.scope('rail'):
      out = dimmer.color(white)
  assert _alpha(out) == round(255 * DIM_LEVEL)


def test_personality_widget_dims_and_restores_on_change():
  """Dom's shared personality artwork only uses wrapped draw calls, so the compact rail scope dims it."""
  from openpilot.starpilot.ui.onroad_personality import PersonalityWidget
  seen = []
  clock = Clock()
  dimmer = StaticDimmer(clock)
  dimmer.set_enabled(True)
  widget = PersonalityWidget(None)
  art = lambda w, h: (NS(width=10, height=10), w, h)
  widget._lead_car = (art(35, 27), art(50, 42))
  widget._distance = [(art(32, 7), art(60, 35)), (art(40, 9), art(68, 37)), (art(48, 11), art(76, 39))]
  rect = rl.Rectangle(476, 60, 60, 120)
  def frame(personality):
    clock.now += 0.05
    seen.clear()
    with dimmer.scope('following_distance'):
      widget.draw(rect, personality=personality, has_lead=True, longitudinal_active=True)
    return [_alpha(c) for c in seen]
  with patch.object(rl, 'draw_texture_pro', lambda *a: seen.append(a[-1]), create=True), \
       patch.object(rl, 'get_time', lambda: clock.now, create=True):
    for _ in range(100):
      full = frame(1)
    for _ in range(int((DIM_AFTER_S + DIM_FADE_S + 1) / 0.05)):
      dim = frame(1)
    assert len(dim) == len(full) and all(abs(d - round(f * DIM_LEVEL)) <= 1 for d, f in zip(dim, full, strict=True))
    frame(2)
    assert max(frame(2)) == 255


@pytest.mark.parametrize('profile', [Profile.LARGE, Profile.COMPACT])
def test_burn_in_rows_save_without_custom_display(tmp_path, profile):
  params = Params(str(tmp_path))
  owner = DisplayOwner(params, lambda: True, gpu_capable=True)
  large = profile == Profile.LARGE
  prefs = read_preferences(params, large=large)
  assert not prefs.pixel_shift and not prefs.static_dim
  for key, field in ((PIXEL_SHIFT, 'pixel_shift'), (STATIC_DIM, 'static_dim')):
    row = next(r for r in owner.snapshot(profile).rows if r.key == key)
    assert row.value == 'Off' and row.available and row.default_value == 'Off' and row.choices == ('Off', 'On')
    assert row.dependencies == ()
    request = row_change(row, 1)
    assert request is not None and owner.apply(request)
    assert getattr(read_preferences(params, large=large), field)
  assert not params.get_bool(MASTER)
  Path(params.get_param_path(PIXEL_SHIFT)).write_bytes(b'bad')
  assert not read_preferences(params, large=large).pixel_shift


def test_device_update_sets_offset_and_dimmer():
  device = native.Device.__new__(native.Device)
  offsets = []
  fake_app = NS(big_ui=lambda: False, frame_offset=(1, 1), set_frame_offset=lambda x, y: offsets.append((x, y)))
  dimmer = StaticDimmer(Clock())
  with patch.object(native, 'gui_app', fake_app), patch.object(burn_in, 'dimmer', dimmer), \
       patch.object(native.time, 'monotonic', return_value=0.0):
    device._display_preferences = DisplayPreferences(pixel_shift=True, static_dim=True)
    device._update_burn_in()
    assert offsets[-1] == orbit_offset(0.0, burn_in.ORBIT_AMPLITUDE_COMPACT) and dimmer.enabled
    device._display_preferences = DisplayPreferences()
    device._update_burn_in()
    assert offsets[-1] == (0, 0) and not dimmer.enabled
