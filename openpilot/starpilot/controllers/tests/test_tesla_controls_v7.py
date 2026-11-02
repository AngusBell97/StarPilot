import json
from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from openpilot.cereal import messaging
from opendbc.car import gen_empty_fingerprint
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR,TeslaFlags
from openpilot.selfdrive.car.cruise import VCruiseHelper
from opendbc.car.structs import car
from openpilot.starpilot.controllers.cruise_action import CruiseActionConsumer,CruiseActionPublisher
from openpilot.starpilot.controllers.tesla_receiver import TeslaReceiver
from openpilot.starpilot.controllers.tests.test_cruise_action import State,Publisher
from openpilot.starpilot.controllers.tests.test_tesla_decoder import wheel
from openpilot.starpilot.common.car_control_config import PARAM,SPEED_UP,SPEED_DOWN
from openpilot.starpilot.ui.tesla_controls_feature import TeslaControlsFeature
from openpilot.starpilot.tests.port_settings_fixture import Saved,request
from openpilot.starpilot.favorites.state import FavoriteAction

def vehicle():
  fp=gen_empty_fingerprint();fp[2][0x489]=8;fp[1][0x3df]=8
  cp=CarInterface.get_params(CAR.TESLA_MODEL_3,fp,[],True,False,False)
  cp.pcmCruise=True
  return cp

def slc(sm,now):
  value=messaging.new_message('slcState',valid=True)
  value.slcState.enabled=True;value.slcState.frameMonoTime=now
  sm['slcState']=value.slcState
  sm.valid['slcState']=sm.alive['slcState']=sm.seen['slcState']=True
  sm.logMonoTime['slcState']=now;sm.recv_time['slcState']=now/1e9

@pytest.mark.parametrize('metric,start,step,target',[(True,83.,1,84.),(True,83.,10,90.),(True,80.,-10,70.),
 (False,53./.621371,10,60./.621371),(False,50./.621371,-10,40./.621371)])
def test_pcm_tesla_commands_use_authoritative_target_and_snap_in_display_units(metric,start,step,target):
  cp=vehicle();now=5_000_000_000;sm=State(now);slc(sm,now)
  cs=car.CarState.new_message(canValid=True,cruiseState={'available':True})
  h=VCruiseHelper(cp);h.slc_pcm=True;h.v_cruise_kph=start
  pub=Publisher();sender=CruiseActionPublisher();consumer=CruiseActionConsumer()
  assert sender.dispatch(step>0,sm,cp,pub,now_ns=now,step_units=step)
  assert consumer.apply(pub.message,cp,cs,sm,h,now_ns=now,is_metric=metric,enabled=True)
  assert h.v_cruise_kph==pytest.approx(target,abs=.08)
  assert h.slc_cruise_change[2]==('accel' if step>0 else 'decel')
  assert not consumer.apply(pub.message,cp,cs,sm,h,now_ns=now+100_000_000,is_metric=metric,enabled=True)

@pytest.mark.parametrize('fault',['slc_off','stale','stock_long','other_pcm','owner_off'])
def test_pcm_commands_require_current_software_owner(fault):
  cp=vehicle();now=5_000_000_000;sm=State(now);slc(sm,now);pub=Publisher();sender=CruiseActionPublisher()
  assert sender.dispatch(True,sm,cp,pub,now_ns=now,step_units=1)
  cs=car.CarState.new_message(canValid=True,cruiseState={'available':True});h=VCruiseHelper(cp);h.slc_pcm=True;h.v_cruise_kph=80.
  if fault=='slc_off':sm['slcState'].enabled=False
  if fault=='stale':sm.logMonoTime['slcState']-=100_000_001
  if fault=='stock_long':cp.openpilotLongitudinalControl=False
  if fault=='other_pcm':cp.carFingerprint='OTHER'
  if fault=='owner_off':h.slc_pcm=False
  assert not CruiseActionConsumer().apply(pub.message,cp,cs,sm,h,now_ns=now,is_metric=True,enabled=True)

@pytest.fixture
def receiver(tmp_path):
  p=Saved(tmp_path);cp=vehicle();p.put('IsOnroad',b'1',block=True)
  document={'version':1,'vehicle':cp.carFingerprint,'bindings':{'right_wheel_up':{'key':SPEED_UP,'value':None,'label':'Up'}}}
  p.put(PARAM,json.dumps(document).encode(),block=True)
  calls=[];queue=[];clock=[1.]
  actions={SPEED_UP:FavoriteAction(SPEED_UP,'Up',available=True,invoke_value=lambda value:calls.append(value))}
  r=TeslaReceiver(p,lambda:actions,lambda:cp,clock=lambda:clock[0],socket_factory=lambda:object(),drain=lambda _:list(queue))
  def feed(data,delta=.1,address=0x3c2,bus=1,valid=True):
    clock[0]+=delta;event=messaging.new_message('can',1,valid=valid,logMonoTime=int(clock[0]*1e9))
    event.can[0]={'address':address,'dat':data,'src':bus};queue[:]=[event];r.poll()
  return p,cp,r,feed,calls,document

@pytest.mark.parametrize('ticks',[1,2,3])
def test_receive_only_decoder_delivers_one_action_on_release(receiver,ticks):
  p,cp,r,feed,calls,doc=receiver
  feed(wheel());feed(wheel(right_scroll=ticks));feed(wheel());feed(wheel())
  assert calls==[0 if ticks==1 else 1]

@pytest.mark.parametrize('fault',['changed','safe','bad','wrong_bus'])
def test_held_input_cannot_cross_configuration_or_validity_changes(receiver,fault):
  p,cp,r,feed,calls,doc=receiver
  feed(wheel());feed(wheel(right_scroll=2))
  if fault=='changed':doc['bindings']['right_wheel_up']['label']='Changed';p.put(PARAM,json.dumps(doc).encode(),block=True)
  if fault=='safe':p.put('SafeMode',b'1',block=True)
  feed(wheel(),valid=fault!='bad',bus=0 if fault=='wrong_bus' else 1)
  assert not calls

def test_shared_settings_exact_document_and_vehicle_guards(tmp_path):
  p=Saved(tmp_path);cp=vehicle();parked=[True]
  owner=NS(params=p,vehicle_params=lambda:cp,vehicle_fingerprint=lambda:cp.carFingerprint,authority=lambda _:parked[0])
  e=TeslaControlsFeature(owner);row=next(r for r in e.rows() if r.key=='tesla_input:right_wheel_up')
  assert e.apply(request(row,'Increase Comma Set Speed'))
  assert json.loads((p.path/PARAM).read_bytes())['bindings']['right_wheel_up']['key']==SPEED_UP
  assert not e.apply(request(row,'Decrease Comma Set Speed'))
  cp.flags=int(cp.flags)|int(TeslaFlags.AOL_SCREEN_BUTTON)
  assert not next(r for r in e.rows() if r.key=='tesla_input:screen_three_finger_tap').available
  current=next(r for r in e.rows() if r.key==row.key);parked[0]=False
  assert not e.apply(request(current,'Decrease Comma Set Speed'))
