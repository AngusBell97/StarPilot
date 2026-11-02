"""SP-011/SP-031: real CAN parser and formatter integration, no vehicle output."""
from types import SimpleNamespace
import pytest
from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CAR, DBC, CarControllerParams
from opendbc.car.dashboard_speed_limit import Status, Tracker
from opendbc.car.tesla.speed_limit import update_dashboard_limit
from openpilot.starpilot.speed_limits.runtime import dashboard_observation
from openpilot.starpilot.speed_limits.acceptance import ObservationKind

class Capture:
  def make_can_msg(self, name, bus, values):
    return values

@pytest.mark.parametrize('accel', [-3.48, -.1, 0., .8, 2.])
def test_exact_previous_ramp_with_unchanged_accel_and_speed(accel):
  codec=TeslaCAN(None,Capture())
  for frame in range(0,300,4):
    gas=40<=frame<80 or 120<=frame<124
    value=codec.create_longitudinal_command(4,accel,(frame//4)%8,20.,True,frame=frame,gas_pressed=gas)
    release=frame if gas else 120 if frame>=124 else 76 if frame>=80 else 0
    expected=4.9*min(1.,max(0.,(frame-release)/100.))
    assert value['DAS_jerkMax']==pytest.approx(expected)
    assert value['DAS_jerkMin']==pytest.approx(-expected)
    assert value['DAS_accelMin']==accel and value['DAS_accelMax']==max(accel,0)
    assert value['DAS_setSpeed']==pytest.approx((20+accel)*3.6)
    assert value['DAS_controlCounter']==(frame//4)%8
  assert codec.create_longitudinal_command(13,0,7,20,False,frame=304)['DAS_accState']==13
  assert TeslaCAN(None,Capture()).create_longitudinal_command(4,0,0,20,True,frame=0)['DAS_jerkMax']==0
  assert codec.create_longitudinal_command(4,0,0,20,True)['DAS_jerkMax']==4.9

@pytest.mark.parametrize('candidate',[CAR.TESLA_MODEL_3,CAR.TESLA_MODEL_Y])
@pytest.mark.parametrize('units,limit,expected',[(0,30,13.4112),(1,50,50/3.6),(0,155,0),(0,0,0)])
def test_actual_carstate_publishes_fused_limit(candidate,units,limit,expected):
  cp=CarInterface.get_non_essential_params(candidate);state=CarState(cp);parsers=state.get_can_parsers(cp)
  state.update(parsers)  # Register the production parser subscriptions.
  packers={bus:CANPacker(parser.dbc.name) for bus,parser in parsers.items()}
  for i in range(30):
    stamp=1_000_000_000+i*100_000_000
    for bus,parser in parsers.items():
      messages=[]
      for address in sorted(parser.addresses):
        msg=parser.dbc.addr_to_msg[address].name
        values={'DI_speedUnits':units} if msg=='DI_state' else {'DAS_fusedSpeedLimit':limit} if msg=='DAS_status' else {}
        messages.append(packers[bus].make_can_msg(msg,parser.bus,values))
      parser.update([stamp,messages])
    state.update(parsers)
  obs=state.dashboard_limit.observation
  assert obs.status==(Status.VALID if expected else Status.ABSENT if limit==155 else Status.UNKNOWN)
  assert obs.speed_mps==pytest.approx(expected)
  evidence=SimpleNamespace(status=obs.status.value,speedMps=obs.speed_mps,observedMonoTime=obs.observed_ns,
    validUntilMonoTime=obs.valid_until_ns,episode=obs.episode,producerSessionId='real-parser')
  result=dashboard_observation(evidence,car_fingerprint=str(candidate),session_id='drive',now_ns=stamp)
  assert result.kind==(ObservationKind.VALID if expected else ObservationKind.ABSENT if limit==155 else ObservationKind.UNKNOWN)
  if expected:
    assert dashboard_observation(evidence,car_fingerprint=str(candidate),session_id='drive',now_ns=obs.valid_until_ns+1).kind is ObservationKind.STALE
