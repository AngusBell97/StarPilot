"""SP-032 persistent comparison stays inside MAX and never owns onroad alerts."""
from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

from openpilot.starpilot.ui.tesla_speed_sync_warning import SpeedSyncCaption


def inputs():
  return dict(car_params=NS(brand='tesla', openpilotLongitudinalControl=True,
                           safetyConfigs=[NS(safetyParam=2049)]),
              car=NS(canValid=True, canTimeout=False, vCruise=40 / .621371,
                     cruiseState=NS(enabled=True, speed=38 / .621371 / 3.6)),
              selfdrive=NS(enabled=True), comma_kph=40 / .621371, drive_frame=1, metric=False,
              slc_enabled=True, alert_present=False)


def advance(projector, values, start=1_000_000_000, seconds=3):
  result = None
  for now in range(start, start + seconds * 1_000_000_000 + 1, 100_000_000):
    result = projector.update(now_ns=now, **values)
  return result


def test_persistence_uses_displayed_units_and_clears_on_match():
  caption, values = SpeedSyncCaption(), inputs()
  assert advance(caption, values, seconds=2) is None
  assert advance(caption, values, start=3_000_000_000, seconds=1) == 'Tesla 38'
  values['car'].cruiseState.speed = 39.8 / .621371 / 3.6
  assert caption.update(now_ns=4_100_000_000, **values) is None
  values['car'].cruiseState.speed = 38 / .621371 / 3.6
  assert caption.update(now_ns=4_200_000_000, **values) is None


@pytest.mark.parametrize('reason', ['car', 'selfdrive', 'can', 'timeout', 'cruise', 'disabled', 'slc',
                                   'alert', 'flag', 'brand', 'long', 'nan', 'inf', 'unset'])
def test_unavailable_or_ineligible_resets_persistence(reason):
  caption, values = SpeedSyncCaption(), inputs()
  assert advance(caption, values) == 'Tesla 38'
  if reason in ('car', 'selfdrive'):
    values[reason] = None
  elif reason == 'can':
    values['car'].canValid = False
  elif reason == 'timeout':
    values['car'].canTimeout = True
  elif reason == 'cruise':
    values['car'].cruiseState.enabled = False
  elif reason == 'disabled':
    values['selfdrive'].enabled = False
  elif reason == 'slc':
    values['slc_enabled'] = False
  elif reason == 'alert':
    values['alert_present'] = True
  elif reason == 'flag':
    values['car_params'].safetyConfigs[0].safetyParam = 1
  elif reason == 'brand':
    values['car_params'].brand = 'toyota'
  elif reason == 'long':
    values['car_params'].openpilotLongitudinalControl = False
  else:
    values['comma_kph'] = {'nan': float('nan'), 'inf': float('inf'), 'unset': 255}[reason]
  assert caption.update(now_ns=4_100_000_000, **values) is None
  assert caption.update(now_ns=4_200_000_000, **inputs()) is None


@pytest.mark.parametrize('change', ['drive', 'units', 'backward', 'gap'])
def test_new_identity_or_clock_gap_requires_new_persistence(change):
  caption, values = SpeedSyncCaption(), inputs()
  assert advance(caption, values) == 'Tesla 38'
  now = 4_100_000_000
  if change == 'drive':
    values['drive_frame'] = 5
  elif change == 'units':
    values['metric'] = True
  elif change == 'backward':
    now = 2_000_000_000
  else:
    now = 6_000_000_000
  assert caption.update(now_ns=now, **values) is None
  expected = 'Tesla 61' if change == 'units' else 'Tesla 38'
  assert advance(caption, values, start=now) == expected


def test_actual_snapshot_never_promotes_mismatch_to_an_alert():
  from openpilot.starpilot.ui.tests.test_runtime_snapshot import ui_fake, RuntimeSnapshotAdapter, NOW
  from openpilot.starpilot.ui.onroad_state import AlertSize
  from openpilot.starpilot.ui.shell import ShellMode
  from opendbc.car.structs import CarState, CarParams

  ui = ui_fake()
  ui.CP = CarParams.new_message(brand='tesla', openpilotLongitudinalControl=True,
                               safetyConfigs=[{'safetyModel': 'tesla', 'safetyParam': 2049}])
  # The caption compares the displayed MAX, even if the uncorrected target
  # differs from the cluster value. It never feeds back into either target.
  car = CarState.new_message(canValid=True, vCruise=38 / .621371, vCruiseCluster=40 / .621371,
                            vEgo=17, vEgoCluster=17)
  car.cruiseState.enabled = True
  car.cruiseState.speed = 38 / .621371 / 3.6
  ui.sm.put('carState', car.as_reader())
  ui.sm['selfdriveState'].enabled = True
  adapter = RuntimeSnapshotAdapter(ui)
  stamps = dict(ui.sm.logMonoTime)
  receipts = dict(ui.sm.recv_time)

  def sample(now):
    for service in stamps:
      ui.sm.logMonoTime[service] = stamps[service] + now - NOW
      ui.sm.recv_time[service] = receipts[service] + (now - NOW) / 1e9
    return adapter.build(ShellMode.ONROAD, now_ns=now).onroad

  for now in range(NOW, NOW + 3_000_000_001, 100_000_000):
    result = sample(now)
    assert result.alert.size == AlertSize.NONE
    assert result.experimental_available
  assert result.tesla_sync_caption == 'Tesla 38'
  ui.sm['selfdriveState'].alertSize = NS(raw=3)
  ui.sm['selfdriveState'].alertText1 = 'TAKE CONTROL'
  ui.sm['selfdriveState'].alertStatus = NS(raw=2)
  result = sample(NOW + 3_100_000_000)
  assert result.alert.text1 == 'TAKE CONTROL'
  assert result.tesla_sync_caption is None
  ui.sm['selfdriveState'].alertSize = NS(raw=0)
  ui.sm['selfdriveState'].alertText1 = ''
  ui.sm.valid['carState'] = False
  result = sample(NOW + 3_200_000_000)
  assert result.alert.size == AlertSize.NONE
  assert result.tesla_sync_caption is None


@pytest.mark.parametrize('profile', ['large', 'compact'])
def test_caption_follows_max_visibility_and_alert_priority(profile, monkeypatch):
  import pyray as rl
  from openpilot.starpilot.ui.tests.test_unified_speed_native import fonts, state, foreground_calls
  from openpilot.starpilot.ui.onroad_compact_widgets import CompactHudRenderer
  from openpilot.starpilot.ui.onroad_large_widgets import UnifiedSpeedWidget
  from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert
  from openpilot.starpilot.ui.presentation import Profile

  monkeypatch.setattr(rl, 'draw_circle_gradient', Mock())
  font = fonts(Profile(profile))
  current = state(tesla_sync_caption='Tesla 38')
  if profile == 'large':
    widget = UnifiedSpeedWidget(font)
    widget.sign.render = Mock()
    render = lambda value: widget.render(rl.Rectangle(0, 0, 2160, 1080), value)
  else:
    widget = CompactHudRenderer(font, Mock())
    widget._set_speed_opacity = lambda *args: 1
    render = widget.render_max_speed
  render(current)
  assert 'Tesla 38' in [c.args[0] for c in foreground_calls(font.draw)]
  font.draw.reset_mock()
  render(replace(current, alert=OnroadAlert(AlertSize.SMALL, 'Native warning')))
  assert 'Tesla 38' not in [c.args[0] for c in foreground_calls(font.draw)]
  font.draw.reset_mock()
  if profile == 'large':
    current.customization['showMaxSpeed'] = False
  else:
    current.customization['layouts']['compact']['max_speed']['enabled'] = False
  render(current)
  assert 'Tesla 38' not in [c.args[0] for c in foreground_calls(font.draw)]
