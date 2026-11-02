"""SP-011: saved wheel mappings cannot own a Tesla target while SLC is off."""
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import pytest
from openpilot.cereal import messaging
from openpilot.common.params import Params
from opendbc.car.structs import car
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.starpilot.common.car_control_config import PARAM, SPEED_UP, software_cruise_requested
from openpilot.starpilot.controllers.cruise_action import CruiseActionConsumer, CruiseActionPublisher, ui_authority
from openpilot.starpilot.controllers.tests.test_cruise_action import State, Publisher, cp as mock_cp
from openpilot.starpilot.controllers.tests.test_tesla_controls_v7 import vehicle, slc
from openpilot.starpilot.tests.port_settings_fixture import Saved

MPH_TO_KPH = 1.609344
NOW = 5_000_000_000


def state(mph=25):
  return car.CarState.new_message(canValid=True, cruiseState={
    'available': True, 'enabled': True, 'speed': mph * .44704, 'speedCluster': mph * .44704})


@pytest.mark.parametrize('saved', [None, b'0', b'1', b'2', b'true', b''])
def test_mappings_require_enabled_saved_slc_and_remain_saved(tmp_path, saved):
  params = Saved(tmp_path)
  cp = vehicle()
  raw = json.dumps({'version': 1, 'vehicle': cp.carFingerprint, 'bindings': {
    'right_wheel_up': {'key': SPEED_UP, 'value': None, 'label': 'Up'}}}).encode()
  params.put(PARAM, raw, block=True)
  if saved is not None:
    params.put('SpeedLimitController', saved, block=True)
  assert software_cruise_requested(params, cp) is (saved == b'1')
  assert (params.path / PARAM).read_bytes() == raw
  params.put('SpeedLimitController', b'1', block=True)
  assert software_cruise_requested(params, cp)
  params.put('SafeMode', b'1', block=True)
  assert not software_cruise_requested(params, cp)


@pytest.mark.parametrize('model', ['TESLA_MODEL_3', 'TESLA_MODEL_Y'])
@pytest.mark.parametrize('engaged', [False, True])
@pytest.mark.parametrize('stock', [25, 50, 60])
def test_slc_off_matches_stock_even_with_latched_mapping(model, engaged, stock):
  cp = vehicle(); cp.carFingerprint = model
  h = VCruiseHelper(cp); h.v_cruise_kph = 55 * MPH_TO_KPH
  h.tesla_controls_pcm = True
  h.update_v_cruise(state(stock), engaged, False, slc_enabled=False, tesla_controls=True)
  assert h.v_cruise_kph / MPH_TO_KPH == pytest.approx(stock, abs=.001)
  assert h.v_cruise_cluster_kph / MPH_TO_KPH == pytest.approx(stock, abs=.001)
  assert not h.slc_pcm and not h.tesla_controls_pcm
  assert h.slc_cruise_change is None


def test_live_toggle_relinks_then_keeps_independence_when_reenabled():
  h = VCruiseHelper(vehicle()); h.v_cruise_kph = 55 * MPH_TO_KPH
  cs = state()
  h.update_v_cruise(cs, True, False, slc_enabled=True, tesla_controls=True)
  assert h.v_cruise_kph / MPH_TO_KPH == pytest.approx(55)
  h.update_v_cruise(cs, True, False, slc_enabled=False, tesla_controls=True)
  assert h.v_cruise_kph / MPH_TO_KPH == pytest.approx(25, abs=.001)
  cs.cruiseState.speed = 50 * .44704
  h.update_v_cruise(cs, True, False, slc_enabled=False, tesla_controls=True)
  assert h.v_cruise_kph / MPH_TO_KPH == pytest.approx(50, abs=.001)
  h.update_v_cruise(cs, True, False, slc_enabled=True, tesla_controls=True)
  h.apply_slc_target(55 * .44704, False)
  h.update_v_cruise(cs, True, False, slc_enabled=True, tesla_controls=True)
  assert h.v_cruise_kph / MPH_TO_KPH == pytest.approx(55, abs=.04)


@pytest.mark.parametrize('fault', ['off', 'display_only', 'stale', 'missing', 'owner_off'])
def test_mapping_cannot_bypass_live_slc_at_publisher_or_consumer(fault):
  cp = vehicle(); sm = State(NOW); slc(sm, NOW)
  h = VCruiseHelper(cp); h.v_cruise_kph = 80; h.slc_pcm = h.tesla_controls_pcm = True
  pub = Publisher(); sender = CruiseActionPublisher()
  assert sender.dispatch(True, sm, cp, pub, now_ns=NOW, step_units=1, tesla_controls=True)
  if fault == 'off': sm['slcState'].enabled = False
  if fault == 'display_only': sm['slcState'].displayOnly = True
  if fault == 'stale': sm.logMonoTime['slcState'] -= 100_000_001
  if fault == 'missing': del sm['slcState']
  if fault == 'owner_off': h.slc_pcm = False
  if fault != 'owner_off':
    assert not ui_authority(sm, cp, NOW, tesla_controls=True)
    assert not sender.dispatch(True, sm, cp, Publisher(), now_ns=NOW, tesla_controls=True)
  assert not CruiseActionConsumer().apply(pub.message, cp, state(), sm, h,
                                         now_ns=NOW, is_metric=True, enabled=True)
  assert h.v_cruise_kph == 80


@pytest.mark.parametrize('old_part', ['request', 'frame', 'publication'])
def test_queued_action_and_slc_evidence_cannot_cross_configuration_boundary(old_part):
  cp = vehicle(); sm = State(NOW); slc(sm, NOW)
  h = VCruiseHelper(cp); h.v_cruise_kph = 80; h.slc_pcm = h.tesla_controls_pcm = True
  pub = Publisher(); sender = CruiseActionPublisher(); consumer = CruiseActionConsumer()
  assert sender.dispatch(True, sm, cp, pub, now_ns=NOW, step_units=1)
  floor = NOW + 10_000_000; later = floor + 10_000_000
  if old_part == 'request':
    slc(sm, later)
  else:
    assert sender.dispatch(True, sm, cp, pub, now_ns=later, step_units=1)
    if old_part == 'frame': sm.logMonoTime['slcState'] = later
  assert not consumer.apply(pub.message, cp, state(), sm, h, now_ns=later,
                            is_metric=True, enabled=True, after_ns=floor)
  assert h.v_cruise_kph == 80
  slc(sm, later)
  assert sender.dispatch(True, sm, cp, pub, now_ns=later, step_units=1)
  assert consumer.apply(pub.message, cp, state(), sm, h, now_ns=later,
                        is_metric=True, enabled=True, after_ns=floor)
  assert h.v_cruise_kph == 81


def test_non_pcm_controller_still_works_without_slc():
  cp = mock_cp(); sm = State(NOW); h = VCruiseHelper(cp); h.v_cruise_kph = 80
  pub = Publisher()
  assert CruiseActionPublisher().dispatch(True, sm, cp, pub, now_ns=NOW)
  assert CruiseActionConsumer().apply(pub.message, cp, state(), sm, h, now_ns=NOW, is_metric=True, enabled=True)
  assert h.v_cruise_kph == 81


@pytest.mark.parametrize('slc_on', [False, True])
def test_actual_card_refreshes_owner_before_consuming_queued_action(tmp_path, slc_on):
  from openpilot.selfdrive.car import card
  cp = vehicle(); sm = State(NOW); slc(sm, NOW); sm['carControl'].longActive = True
  pub = Publisher()
  assert CruiseActionPublisher().dispatch(True, sm, cp, pub, now_ns=NOW, step_units=1, tesla_controls=True)
  cs = state()
  h = VCruiseHelper(cp); h.v_cruise_kph = 55 * MPH_TO_KPH; h.slc_pcm = h.tesla_controls_pcm = True
  instance = card.Car.__new__(card.Car)
  instance.CP = cp; vars(instance)['sm'] = sm
  instance.CC_prev = car.CarControl(enabled=True); instance.CS_prev = cs
  vars(instance)['CI'] = NS(CS=NS(), update=lambda packets: cs)
  vars(instance)['RI'] = NS(update=lambda packets: None)
  instance.car_gps_publisher = card.CarGpsPublisher()
  instance.params = Params(str(tmp_path))
  instance.can_sock = object(); instance.can_rcv_cum_timeout_counter = 0
  instance.ioniq6_long_prearmed = False
  instance.conditional_replay = instance.slc_replay = instance.curve_replay = False
  instance.slc_control_enabled = slc_on
  instance.tesla_controls_active = instance.tesla_controls_requested = True
  instance.switchback_button_tracker = instance.switchback_settings_owner = None
  instance.wheel_publisher = card.WheelPublisher(instance.params)
  instance.aol_card_intent = None; instance.is_metric = False; instance.experimental_mode = False
  instance.v_cruise_helper = h; instance.slc_command_sock = None
  instance.controller_cruise_sock = object(); instance.controller_cruise_consumer = CruiseActionConsumer()
  with (patch.object(instance, 'pm', NS(send=Mock()), create=True),
        patch.object(card.messaging, 'drain_sock_raw', return_value=[messaging.new_message('can', 1).to_bytes()]),
        patch.object(card.messaging, 'recv_one_or_none', side_effect=[pub.message, None]),
        patch.object(card.time, 'monotonic_ns', return_value=NOW)):
    selected, _ = instance.state_update()
  assert selected.vCruise / MPH_TO_KPH == pytest.approx(56 if slc_on else 25, abs=.04)
  assert instance.tesla_controls_active is slc_on


def test_assignment_admission_waits_for_disengagement_but_slc_off_revokes_immediately():
  from openpilot.selfdrive.car.card import Car
  c = Car.__new__(Car); c.CC_prev = NS(enabled=True)
  vars(c)['sm'] = {'carControl': NS(enabled=True)}
  c.tesla_controls_active = False; c.tesla_controls_requested = c.slc_control_enabled = True
  c.refresh_tesla_controls_owner(); assert not c.tesla_controls_active
  c.CC_prev.enabled = c.sm['carControl'].enabled = False
  c.refresh_tesla_controls_owner(); assert c.tesla_controls_active
  c.CC_prev.enabled = c.sm['carControl'].enabled = True
  c.slc_control_enabled = False
  c.refresh_tesla_controls_owner(); assert not c.tesla_controls_active
