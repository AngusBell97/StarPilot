"""SP-031: compare actual emitted commands to the pinned 7.0 controller."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest
from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import car
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.coop_preferences import apply_cooperative
from opendbc.car.tesla.screen_button import apply_screen_button, screen_qualified
from opendbc.car.tesla.values import CAR, DBC

spec = importlib.util.spec_from_file_location('pinned_v7_tesla', Path(__file__).parent/'fixtures/upstream_7_carcontroller.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def params(candidate=CAR.TESLA_MODEL_3, coop=False, long=True, screen=False, brake=False):
  fingerprint = gen_empty_fingerprint()
  fingerprint[2][0x489] = 8
  fingerprint[1][0x3DF] = 8
  cp = CarInterface.get_params(candidate, fingerprint, [], long, False, False)
  apply_screen_button(cp, screen, brake)
  apply_cooperative(cp, coop)
  return cp


def state(torque=0., disengage=False):
  return SimpleNamespace(out=SimpleNamespace(vEgo=20., vEgoRaw=20., steeringAngleDeg=0., steeringTorque=torque,
    steeringDisengage=disengage, gasPressed=False), hands_on_level=0, das_control={'DAS_controlCounter': 0})


@pytest.mark.parametrize('candidate', (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y))
@pytest.mark.parametrize('long', (False, True))
def test_flag_off_matches_pinned_7_commands_exactly(candidate, long):
  cp = params(candidate, long=long)
  before, after = base.CarController(DBC[candidate], cp), CarController(DBC[candidate], cp)
  for tick in range(160):
    cc = car.CarControl.new_message()
    cc.latActive = 10 <= tick < 100
    cc.longActive = 20 <= tick < 120
    cc.actuators.steeringAngleDeg = (tick % 25) - 12
    cc.actuators.accel = (tick % 49) / 10 - 3.4
    cc.cruiseControl.cancel = tick >= 140
    a, can_a = before.update(cc.as_reader(), state(), (tick+1)*10_000_000)
    b, can_b = after.update(cc.as_reader(), state(), (tick+1)*10_000_000)
    assert a.to_dict() == b.to_dict()
    assert_except_pedal_jerk(can_a, can_b)


def test_coop_does_not_change_7_longitudinal_after_pedal_release():
  cp = params(coop=True)
  before, after = base.CarController(DBC[CAR.TESLA_MODEL_3], cp), CarController(DBC[CAR.TESLA_MODEL_3], cp)
  for tick in range(200):
    cc = car.CarControl.new_message()
    cc.latActive = cc.longActive = True
    cc.actuators.accel = -1.5 if tick < 100 else .8
    cc.cruiseControl.cancel = tick > 180
    cs = state(torque=1.2)
    cs.out.gasPressed = 20 <= tick < 100
    _, a = before.update(cc.as_reader(), cs, (tick+1)*10_000_000)
    _, b = after.update(cc.as_reader(), cs, (tick+1)*10_000_000)
    assert_except_pedal_jerk(a, b, longitudinal_only=True)
  assert after.coop_enabled


@pytest.mark.parametrize('long', (False, True))
@pytest.mark.parametrize('brake', (False, True))
def test_screen_and_coop_survive_repeated_startup_preparation(long, brake):
  cp = params(coop=True, long=long, screen=True, brake=brake)
  expected = 256 | 512 | (1024 if brake else 0) | int(long)
  for _ in range(3):
    assert cp.safetyConfigs[0].safetyParam == expected
    assert screen_qualified(cp)
    apply_screen_button(cp, True, brake)
    apply_cooperative(cp, True)
  apply_cooperative(cp, False)
  assert cp.safetyConfigs[0].safetyParam == expected & ~256
  assert screen_qualified(cp)


def test_disengagement_invalidates_diagnostics_between_can_ticks():
  cp = params(coop=True)
  controller = CarController(DBC[CAR.TESLA_MODEL_3], cp)
  cc = car.CarControl.new_message(latActive=True)
  controller.update(cc.as_reader(), state(torque=1.), 1_000_000_000)
  assert controller.get_steering_limit_info()['valid']
  controller.update(cc.as_reader(), state(torque=5.1, disengage=True), 1_010_000_000)
  assert not controller.get_steering_limit_info()['valid']


def assert_except_pedal_jerk(before, after, longitudinal_only=False):
  a=[m for m in before if not longitudinal_only or m[0]==0x2B9]
  b=[m for m in after if not longitudinal_only or m[0]==0x2B9]
  assert len(a)==len(b)
  for old,new in zip(a,b):
    assert old[0]==new[0] and old[2]==new[2]
    if old[0]!=0x2B9:
      assert old==new
      continue
    values=[]
    for frame in (old,new):
      parser=CANParser('tesla_model3_party',[('DAS_control',0)],frame[2])
      parser.update([1_000_000_000,[frame]])
      values.append(dict(parser.vl['DAS_control']))
    for key in values[0]:
      if key not in ('DAS_jerkMin','DAS_jerkMax','DAS_controlChecksum'):
        assert values[0][key]==values[1][key],key
    # DBC jerk quantization is 0.034, including an offset at physical zero.
    assert -4.934<=values[1]['DAS_jerkMin']<=.034 and -.034<=values[1]['DAS_jerkMax']<=4.934
