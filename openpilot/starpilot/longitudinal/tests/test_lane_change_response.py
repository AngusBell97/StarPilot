"""SP-040 signal-hold acceptance, abrupt hazards and native planner interaction."""
from dataclasses import replace
import pytest
from openpilot.starpilot.longitudinal.lane_change_gap import LaneChangeGap, GapLead, response_has_room
from openpilot.starpilot.longitudinal.tests.test_overtake_preparation import frame, POLICY

LEADS=(GapLead(65.,-2.,0.,0.),)
def warmed(previous=-.6):
  g=LaneChangeGap()
  for i in range(15):
    g.step(POLICY,frame(i,state='off',signal_direction='none',direction='none',leads=LEADS),base_follow=1.5,long_active=True)
    g.limit_acceleration(previous)
  return g

def cycle(g,i,*,moving=False,native=-.6,previous=-.6,cruise=.8,blocked=False,experimental=False,**kw):
  f=frame(i,state='laneChangeStarting' if moving else 'preLaneChange',leads=kw.pop('leads',LEADS),**kw)
  g.step(POLICY,f,base_follow=1.5,long_active=True)
  result=g.comfort_target(native,cruise=cruise,previous=previous,positive_jerk=.6,blocked=blocked,experimental=experimental)
  return g.limit_acceleration(result)

def test_signal_releases_mild_braking_then_movement_accelerates():
  g=warmed();prev=-.6;out=[]
  for i in range(15,25):
    value=cycle(g,i,previous=prev);out.append(value);assert value<=0.;assert value-prev<=.100000001;prev=value
  assert out[5]==pytest.approx(0.,abs=1e-9)
  value=cycle(g,25,moving=True,previous=prev)
  assert 0.<value<=.030000001
  assert g.response_reason=='normal_acceleration'

@pytest.mark.parametrize('visible',[True,False])
def test_existing_native_acceleration_not_zeroed_by_signal(visible):
  g=warmed(.6)
  assert cycle(g,15,native=.6,previous=.6,adjacent_lane_visible=visible)==.6

def test_short_auto_delay_does_not_reset_braking_release():
  g=warmed();prev=-.6
  for i in range(15,25):
    value=cycle(g,i,moving=i>=19,previous=prev)
    if prev<-.10000001:assert value-prev<=.10000001
    elif prev>=0:assert value-prev<=.03000001
    prev=value
  assert prev>0.

@pytest.mark.parametrize('change',[
 {'left_blindspot':True},{'gas_pressed':True},{'brake_pressed':True},
 {'force_decel':True},{'model_should_stop':True},
 {'leads':(GapLead(12.,-8.,0.,0.),)},
 {'leads':(GapLead(65.,-2.,-1.,0.),)},
 {'leads':(GapLead(65.,-2.,0.,0.),GapLead(15.,-5.,0.,0.))},
 {'leads':()}, {'leads':(GapLead(25.,0.,0.,3.),)},
])
def test_hazard_or_new_lead_withdraws_without_positive_override(change):
  g=warmed();cycle(g,15,previous=0.)
  assert cycle(g,16,moving=True,native=-.7,previous=.3,**change)==-.7

@pytest.mark.parametrize('blocked,experimental,native',[(True,False,-.6),(False,True,-.6),(False,False,-1.),(False,False,-3.)])
def test_protective_context_and_strong_braking_unchanged(blocked,experimental,native):
  g=warmed();assert cycle(g,15,moving=True,native=native,previous=.2,blocked=blocked,experimental=experimental)==native

def test_cruise_limit_negative_cannot_be_bypassed():
  g=warmed()
  assert cycle(g,15,moving=True,native=-.6,previous=0.,cruise=-.8)==-.6
  # The planner still takes min(MPC response, cruise); this helper does not own cruise.
  assert min(-.8,cycle(g,16,moving=True,native=-.6,previous=0.,cruise=-.8))==-.8

def test_duplicate_does_not_release_again():
  g=warmed()
  a=cycle(g,15,previous=-.6)
  b=cycle(g,15,previous=a)
  assert b==-.6

def test_cancel_and_expiry_stop_extra_request():
  g=warmed();cycle(g,15,previous=-.6)
  g.step(POLICY,frame(16,state='off',signal_direction='none',direction='none',leads=LEADS),base_follow=1.5,long_active=True)
  assert g.comfort_target(-.6,cruise=.8,previous=0.,positive_jerk=.6,blocked=False,experimental=False)==-.6
  g=warmed()
  for i in range(15,110):cycle(g,i,previous=0.)
  assert g.expired and cycle(g,110,previous=0.)==-.6

def test_forecast_accounts_for_acceleration_and_both_leads():
  f=frame(20,leads=(GapLead(40.,-2.,0.,0.),))
  assert response_has_room(f,.75,0.)
  assert not response_has_room(f,.75,2.)
  assert not response_has_room(replace(f,leads=(*f.leads,GapLead(10.,-4.,0.,0.))),.75,0.)


def test_margin_withdrawal_cannot_rearm_during_same_signal():
  g=warmed();assert cycle(g,15,previous=-.6)>-.6
  assert cycle(g,16,leads=(GapLead(10.,-5.,0.,0.),),previous=.2)==-.6
  for i in range(17,40):assert cycle(g,i,moving=True,previous=0.)==-.6
  assert g.response_aborted

def native_sequence(speed,lead_speed,distance,context=None):
  from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
  from openpilot.starpilot.longitudinal.tests.test_lane_change_gap import NativeFrame, BASE_NS, FRAME_NS
  from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages
  from openpilot.starpilot.longitudinal.tests.test_overtake_preparation import fill_adjacent_geometry
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import CAR
  cp=CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3);cp.openpilotLongitudinalControl=True;cp.dashcamOnly=False;cp.passive=False
  clock=[BASE_NS+10_000_000]
  planner=LongitudinalPlanner(cp,init_v=speed,init_a=-.6,clock_ns=lambda:clock[0])
  sm=NativeFrame(messages(lead=True,e2e=False)[0])
  from openpilot.cereal import messaging
  sm["deviceState"]=messaging.new_message("deviceState").deviceState
  sm["deviceState"].started=True; sm["deviceState"].startedMonoTime=BASE_NS-1
  sm.logMonoTime["deviceState"]=BASE_NS; sm.valid["deviceState"]=True; sm.alive["deviceState"]=True
  sm.all_checks=lambda services:all(sm.valid.get(k,False) and sm.alive.get(k,False) for k in services)
  sm['carControl'].longActive=True;sm['carState'].canValid=True
  sm['carState'].vEgo=sm['carState'].vEgoCluster=speed;sm['carState'].aEgo=-.6
  sm['carState'].vCruise=(speed+5.)*3.6
  fill_adjacent_geometry(sm['modelV2'])
  for lead in (sm['radarState'].leadOne,sm['radarState'].leadTwo):
    lead.dRel=distance;lead.vLead=lead.vLeadK=lead_speed;lead.vRel=lead_speed-speed
    lead.aLeadK=0.;lead.yRel=0.;lead.radar=False
  from openpilot.starpilot.controllers.coast import CoastRuntime
  from openpilot.common.params import Params
  coast_runtime=CoastRuntime(); coast_params=Params()
  records=[]
  for i in range(50):
    stamp=BASE_NS+i*FRAME_NS;clock[0]=stamp+10_000_000
    sm.logMonoTime=dict.fromkeys(sm.logMonoTime,stamp)
    sm.recv_time=dict.fromkeys(sm.logMonoTime,stamp/1e9);sm.seen=dict.fromkeys(sm.logMonoTime,True)
    sm['carState'].leftBlinker=i>=20
    sm['modelV2'].meta.laneChangeState='off' if i<20 else 'preLaneChange' if i<24 else 'laneChangeStarting'
    sm['modelV2'].meta.laneChangeDirection='none' if i<20 else 'left'
    kwargs={"wheel_coast":(coast_runtime,coast_params)}
    if context is not None and i>=30:
      if context=='force_coast':
        from openpilot.starpilot.controllers.wheel_actions import cp_fingerprint
        coast_runtime.drive=BASE_NS-1;coast_runtime.cp_signature=cp_fingerprint(cp)
        coast_runtime.consumer.last_heartbeat=stamp
        coast_runtime.owner.force_coast=True;sm['carControl'].enabled=True
      elif context=='pulse_coast':
        from openpilot.starpilot.controllers.coast import CoastOwner
        owner=CoastOwner();owner.pulse_glide=True
        plan=owner.sample(admitted=True,active=True,gas=False,brake=False,speed_mps=speed,
                          target_mps=speed,delta_mps=2.,pitch=0.,lead_relevant=False,stop_context=False)
        assert plan.pulse_coasting
        coast_runtime.sample=lambda *args,**kwargs:plan
      elif context=='slc':
        from openpilot.starpilot.longitudinal.cruise_ceiling import CruiseCeiling
        from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import ACTIVE
        kwargs['cruise_ceiling']=CruiseCeiling(speed-2.,ACTIVE)
      elif context=='curve':
        from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
        kwargs['curve_ceiling']=CurveCeiling(speed-2.,stamp)
      elif context=='model':
        sm['selfdriveState'].experimentalMode=True;sm['modelV2'].action.desiredAcceleration=-.7
      elif context=='stop':sm['modelV2'].action.shouldStop=True
      elif context=='blindspot':sm['carState'].leftBlindspot=True
      elif context=='override':sm['carState'].gasPressed=True
      elif context=='invalid':sm.valid['radarState']=False
      elif context=='lead_brakes':
        sm['radarState'].leadOne.aLeadK=sm['radarState'].leadTwo.aLeadK=-3.
      elif context=='second_lead':
        sm['radarState'].leadTwo.dRel=12.;sm['radarState'].leadTwo.vRel=-5.
        sm['radarState'].leadTwo.vLead=sm['radarState'].leadTwo.vLeadK=speed-5.
    if i < 30:
      from openpilot.starpilot.longitudinal.lead_behavior import current_leads
      assert current_leads(sm,cp,clock[0],('sp040-cem',1,BASE_NS,'conditional_experimental')) is not None
    planner.update(sm,lane_change_policy=POLICY,conditional_handoff=('sp040-cem',1,BASE_NS,'conditional_experimental'),**kwargs)
    assert planner.mpc.solution_status==0
    if context in ('force_coast','pulse_coast') and i>=30:
      assert 'coast' in planner.lane_change_gap.response_vetoes
    records.append((i,float(planner.output_a_target),planner.lane_change_gap.response_reason,planner.lane_change_gap.response_raised))
  return records


@pytest.mark.parametrize('speed,lead_speed,distance',[(20.,18.,50.),(26.8,24.08,58.),(34.5,31.2,62.4)])
def test_native_planner_signal_hold_and_moving_response(speed,lead_speed,distance):
  records=native_sequence(speed,lead_speed,distance)
  prep=[x for x in records if 20<=x[0]<24];moving=[x for x in records if x[0]>=24]
  assert any(x[2]=='signal_hold' for x in prep),records
  assert any(x[1]>0.05 for x in moving[:10]),records
  assert max(x[1] for x in prep)<=0.000001,prep

@pytest.mark.parametrize('context',['slc','curve','model','stop','blindspot','override','invalid','lead_brakes','second_lead','force_coast','pulse_coast'])
def test_native_protection_revokes_an_already_active_response(context):
  records=native_sequence(20.,18.,50.,context)
  assert any(r[3] for r in records[20:30]),records
  assert not any(r[3] for r in records[30:]),records
  if context=='model':assert all(r[1]<=-.7+1e-6 for r in records[30:])
  if context=='second_lead':assert min(r[1] for r in records[30:])<0.  # magnitude belongs to native MPC


def test_cem_inside_gap_cap_cannot_undo_guarded_moving_response(monkeypatch):
  import openpilot.selfdrive.controls.lib.longitudinal_planner as planner_module
  original_adjust=planner_module.adjust_lead_behavior
  caps=[]
  def observe(**kwargs):
    result=original_adjust(**kwargs)
    if result < kwargs['target']-1e-9:caps.append((kwargs['target'],result))
    return result
  monkeypatch.setattr(planner_module,'adjust_lead_behavior',observe)
  records=native_sequence(34.5,31.2,62.4)
  assert caps, 'Fixture must exercise real CEM inside-gap braking'
  assert max(r[1] for r in records[24:34])>.05
  # Counterfactual: restore the previous downstream-cap ownership.
  original_response=LaneChangeGap.comfort_target
  def prior_ownership(self,*args,**kwargs):
    result=original_response(self,*args,**kwargs)
    self.response_raised=False
    return result
  monkeypatch.setattr(LaneChangeGap,'comfort_target',prior_ownership)
  previous=native_sequence(34.5,31.2,62.4)
  assert max(r[1] for r in previous[24:34])<0.


def test_benign_new_lead_requalifies_only_after_fresh_half_second():
  g=warmed();assert cycle(g,15,moving=True,previous=0.)>0.
  changed=(GapLead(85.,-2.,0.,0.),)
  for i in range(16,26):
    assert cycle(g,i,moving=True,leads=changed,previous=-.6)==-.6
    assert not g.response_aborted
  assert cycle(g,26,moving=True,leads=changed,previous=-.6)>-.6
  assert g.response_reason=='normal_acceleration'

@pytest.mark.parametrize('native,experimental,blocked',[
  (-1.1,False,False),(-.6,True,False),(-.6,False,True)])
def test_hazard_during_history_reset_latches(native,experimental,blocked):
  g=warmed();cycle(g,15,moving=True,previous=0.)
  assert cycle(g,16,moving=True,leads=(GapLead(85.,-2.,0.,0.),),
               native=native,experimental=experimental,blocked=blocked)==native
  for i in range(17,40):assert cycle(g,i,moving=True,previous=0.)==-.6
  assert g.response_aborted

def test_signal_can_release_during_admitted_movement_but_cancel_still_withdraws():
  g=warmed();assert cycle(g,15,moving=True,previous=0.)>0.
  assert cycle(g,16,moving=True,signal_direction='none',previous=0.)>0.
  f=frame(17,state='off',signal_direction='none',direction='none',leads=LEADS)
  g.step(POLICY,f,base_follow=1.5,long_active=True)
  assert g.comfort_target(-.6,cruise=.8,previous=0.,positive_jerk=.6,blocked=False,experimental=False)==-.6

def test_diagnostics_record_policy_and_real_vetoes():
  g=warmed();cycle(g,15,moving=True)
  g.comfort_target(-.6,cruise=.8,previous=0.,positive_jerk=.6,blocked=True,
                   experimental=False,block_reasons=('coast','slc'))
  d=g.diagnostic()
  assert d['revision']==5 and d['response_vetoes']==('coast','slc')
  assert d['policy_close_gap'] and d['policy_gap_s']==.75
  assert d['policy_fast_pace']==POLICY.close_gap_fast_pace
