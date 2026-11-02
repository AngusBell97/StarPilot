"""SP-031/SP-032: startup preferences keep exact speed-sync words and nothing else."""
import pytest
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import car
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.coop_preferences import apply_cooperative, supported
from opendbc.car.tesla.screen_button import apply_screen_button, screen_qualified, screen_tap_supported
from opendbc.car.tesla.values import CAR, CANBUS, DBC, TeslaFlags

# Mirrors tesla_sync_param_valid in safety/modes/tesla_speed_sync.h.
NATIVE_SYNC_WORDS = frozenset((2049, 2305, 2561, 2817, 3585, 3841))


def sync_params(candidate=CAR.TESLA_MODEL_3, long=True):
  fingerprint = gen_empty_fingerprint()
  fingerprint[CANBUS.vehicle][0x3DF] = 8
  fingerprint[CANBUS.vehicle][0x3C2] = 8
  fingerprint[CANBUS.autopilot_party][0x489] = 8
  return CarInterface.get_params(candidate, fingerprint, [], long, False, False)


def startup(cp, coop, screen=False, brake=False):
  apply_screen_button(cp, screen, brake)
  apply_cooperative(cp, coop)
  return cp


def expected_word(coop, screen, brake):
  return 2049 | (256 if coop else 0) | ((512 | (1024 if brake else 0)) if screen else 0)


def assert_rejected(cp):
  before = cp.to_bytes()
  assert not supported(cp)
  assert not screen_tap_supported(cp)
  assert not screen_qualified(cp)
  apply_cooperative(cp, True)
  apply_screen_button(cp, True, True)
  assert cp.to_bytes() == before


def test_installed_sync_word_precondition():
  cp = sync_params()
  assert cp.safetyConfigs[0].safetyParam == 2049
  assert cp.openpilotLongitudinalControl
  assert cp.flags & TeslaFlags.HAS_VEHICLE_BUS
  assert supported(cp) and screen_tap_supported(cp) and not screen_qualified(cp)


@pytest.mark.parametrize('coop', (False, True))
def test_cooperative_on_off_keeps_sync(coop):
  cp = startup(sync_params(), coop)
  assert cp.safetyConfigs[0].safetyParam == (2305 if coop else 2049)
  apply_cooperative(cp, not coop)
  assert cp.safetyConfigs[0].safetyParam == (2049 if coop else 2305)


@pytest.mark.parametrize('coop', (False, True))
@pytest.mark.parametrize('screen', (False, True))
@pytest.mark.parametrize('brake', (False, True))
def test_screen_brake_matrix_composes_native_words(coop, screen, brake):
  cp = startup(sync_params(), coop, screen, brake)
  expected = expected_word(coop, screen, brake)
  assert expected in NATIVE_SYNC_WORDS
  assert cp.safetyConfigs[0].safetyParam == expected
  assert supported(cp) and screen_tap_supported(cp)
  assert screen_qualified(cp) == screen
  assert bool(cp.flags & TeslaFlags.AOL_SCREEN_BUTTON) == screen
  apply_screen_button(cp, not screen, brake)
  assert cp.safetyConfigs[0].safetyParam == expected_word(coop, not screen, brake)
  assert screen_qualified(cp) == (not screen)


@pytest.mark.parametrize('coop_first', (False, True))
@pytest.mark.parametrize('brake', (False, True))
def test_repeated_startup_ordering_preserves_sync(coop_first, brake):
  cp = sync_params()
  expected = expected_word(True, True, brake)
  for _ in range(3):
    if coop_first:
      apply_cooperative(cp, True)
      apply_screen_button(cp, True, brake)
    else:
      apply_screen_button(cp, True, brake)
      apply_cooperative(cp, True)
    assert cp.safetyConfigs[0].safetyParam == expected
    assert screen_qualified(cp)
  apply_cooperative(cp, False)
  assert cp.safetyConfigs[0].safetyParam == expected & ~256
  apply_screen_button(cp, False, brake)
  assert cp.safetyConfigs[0].safetyParam == 2049
  apply_cooperative(cp, True)
  assert cp.safetyConfigs[0].safetyParam == 2305


def test_sync_model_y_rejected():
  cp = sync_params(CAR.TESLA_MODEL_Y)
  cp.safetyConfigs[0].safetyParam = 2049
  assert_rejected(cp)


def test_sync_longitudinal_off_rejected():
  for word in (2048, 2049, 2304):
    cp = sync_params(long=False)
    cp.safetyConfigs[0].safetyParam = word
    assert_rejected(cp)
  cp = sync_params()
  cp.openpilotLongitudinalControl = False
  assert_rejected(cp)


def test_sync_without_vehicle_bus_or_with_unknown_flag_rejected():
  cp = sync_params()
  cp.flags &= ~int(TeslaFlags.HAS_VEHICLE_BUS)
  assert_rejected(cp)
  cp = sync_params()
  cp.flags |= 64
  assert_rejected(cp)


@pytest.mark.parametrize('word', (2049 | 2, 2049 | 4, 2049 | 16, 2049 | 4096, 2049 | 1024, 2305 | 4096))
def test_sync_with_unknown_safety_bit_rejected(word):
  cp = sync_params()
  cp.safetyConfigs[0].safetyParam = word
  assert_rejected(cp)


def test_real_controller_and_carstate_cooperate_after_sync_startup():
  cp = startup(sync_params(), True)
  assert cp.safetyConfigs[0].safetyParam == 2305
  cs = CarState(cp)
  cs.hands_on_level = 0
  cs.das_control = {'DAS_controlCounter': 0}
  cs.out.vEgo = cs.out.vEgoRaw = 20.
  cs.out.steeringAngleDeg = 0.
  cs.out.gasPressed = False
  controller = CarController(DBC[CAR.TESLA_MODEL_3], cp)
  assert controller.coop_enabled
  cc = car.CarControl.new_message(latActive=True)
  for tick in range(10):
    cs.out.steeringTorque, cs.out.steeringDisengage = 1., False
    controller.update(cc.as_reader(), cs, 1_000_000_000 + tick * 10_000_000)
  assert controller.get_steering_limit_info()['valid']
  cs.out.steeringTorque, cs.out.steeringDisengage = 5.1, True
  controller.update(cc.as_reader(), cs, 1_100_000_000)
  assert not controller.get_steering_limit_info()['valid']
  assert cp.safetyConfigs[0].safetyParam == 2305

