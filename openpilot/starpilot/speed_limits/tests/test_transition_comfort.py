from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from unittest.mock import patch
from openpilot.starpilot.speed_limits.transition_comfort import Adjustment,TransitionComfort,ceiling
from openpilot.starpilot.speed_limits.custom_settings import CustomSettings
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages,V_EGO
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR


def comfort_input(limit, *, now=1_000_000_000, pedal=False):
  candidate=NS(source='map',speed_mps=limit)
  result=NS(state=NS(acceptance=NS(accepted=NS(candidate=candidate)),
                    override=NS(persistent_selected_mps=None)),
            selection=NS(observation=NS(candidate=candidate)),
            domain=NS(context=NS(effective_cluster_mps=limit)),errors=())
  frame=NS(now_ns=now,authority=NS(can_control=True),pedal_pressed=pedal,host=NS(force_decel=False))
  return result,frame


@pytest.mark.parametrize('direction', [-1,1])
def test_pedal_pauses_and_resumes_same_unfinished_transition(direction):
  c=TransitionComfort();settings=CustomSettings(transition_offsets=True)
  old,target=(10.,20.) if direction>0 else (30.,20.)
  args=dict(settings=settings,limits={"acceleration":.7,"braking":.8},selected=target,speed=target-direction*5,
            acknowledged=True,selected_acknowledged=True,manual=False,reengage=False)
  c.step(*comfort_input(old),**dict(args,selected=old))
  assert c.step(*comfort_input(target,now=1_050_000_000),**args).limit==(.7 if direction>0 else .8)
  deadline=c.deadline
  for now in (1_100_000_000,2_100_000_000):
    assert c.step(*comfort_input(target,now=now,pedal=True),**args) is None
    assert c.target==target and c.deadline==deadline
  assert c.step(*comfort_input(target,now=2_150_000_000),**args).limit==(.7 if direction>0 else .8)


@pytest.mark.parametrize('veto', ['reengage','lost_authority','force','expired','complete'])
def test_pedal_pause_never_restores_replaced_or_unqualified_target(veto):
  c=TransitionComfort();settings=CustomSettings(transition_offsets=True)
  args=dict(settings=settings,limits={"acceleration":.7,"braking":.8},selected=20.,speed=15.,acknowledged=True,selected_acknowledged=True,manual=False,reengage=False)
  c.step(*comfort_input(10.),**dict(args,selected=10.))
  assert c.step(*comfort_input(20.,now=1_050_000_000),**args) is not None
  result,frame=comfort_input(20.,now=1_100_000_000,pedal=True)
  if veto in ('manual','reengage'):args[veto]=True
  if veto=='changed_target':args['selected']=21.
  if veto=='lost_authority':frame.authority.can_control=False
  if veto=='force':frame.host.force_decel=True
  if veto=='expired':frame.now_ns=c.deadline+1
  if veto=='complete':args['speed']=20.;frame.pedal_pressed=False
  if veto=='disabled':args['limits']=None
  if veto=='persistent':result.state.override.persistent_selected_mps=21.
  assert c.step(result,frame,**args) is None
  # A selected target replacement before its normal one-second ack fence is
  # pending rather than canceled; it must never shape the replacement speed.
  if veto=='changed_target':
    frame.now_ns=2_200_000_000
    assert c.step(result,frame,**args) is None
  assert c.target is None


def test_new_limit_adopted_with_pedal_held_is_shaped_only_after_release():
  c=TransitionComfort();settings=CustomSettings(transition_offsets=True)
  args=dict(settings=settings,limits={"acceleration":.7,"braking":.8},selected=10.,speed=8.,acknowledged=True,manual=False,reengage=False)
  assert c.step(*comfort_input(10.,pedal=True),**args) is None
  args.update(selected=20.,speed=12.)
  assert c.step(*comfort_input(20.,now=1_050_000_000,pedal=True),**args) is None
  assert c.step(*comfort_input(20.,now=1_100_000_000),**args).limit==.7


@pytest.mark.parametrize('limit', [.4,.7,1.5])
@pytest.mark.parametrize('tesla', [False,True])
def test_curve_target_retains_slc_acceleration_bound_and_braking_authority(limit,tesla):
  from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
  from openpilot.selfdrive.controls.lib.longitudinal_planner import get_max_accel
  cp=CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  if tesla:
    from opendbc.car.tesla.interface import CarInterface as TeslaInterface
    from opendbc.car.tesla.values import CAR as TeslaCar
    cp=TeslaInterface.get_non_essential_params(TeslaCar.TESLA_MODEL_3)
    cp.openpilotLongitudinalControl=True
  class Checked(dict):
    def all_checks(self,services):return True
  def run(curve_speed,request_limit,relevant=False):
    planner=LongitudinalPlanner(cp,init_v=V_EGO)
    for i in range(120):
      now=1_000_000_000+i*50_000_000
      sm=Checked(messages()[0]);sm.logMonoTime={'modelV2':now}
      sm['carState'].vCruise=108.;sm['carState'].vEgoCluster=V_EGO
      sm['carControl'].longActive=True
      with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.braking_lead_relevant',return_value=relevant):
        planner.update(sm,now_ns=now,curve_ceiling=CurveCeiling(curve_speed,now) if curve_speed else None,
          slc_comfort=Adjustment(now,30.,1,request_limit))
    return planner
  cap=min(get_max_accel(V_EGO),limit)
  assert run(25.,limit).a_cruise==pytest.approx(cap)
  # Tesla's jerk-limited transition approaches the ceiling asymptotically.
  # A 0.001 m/s^2 tolerance bounds numerical convergence after six seconds.
  assert run(None,limit).a_cruise==pytest.approx(cap,abs=.001)
  assert run(None,limit,relevant=True).output_a_target==pytest.approx(run(None,3.5,relevant=True).output_a_target)
  assert run(15.,limit).output_a_target==pytest.approx(run(15.,3.5).output_a_target)

@pytest.mark.parametrize('changed', ['stale','future','manual','settled','blocked','bad_limit'])
def test_vetoes(changed):
  r=Adjustment(1_000_000_000,20.,-1,.8)
  args=dict(now_ns=r.now_ns,selected=20.,speed=25.,blocked=False)
  if changed=='stale':args['now_ns']+=100_000_001
  if changed=='future':args['now_ns']-=1
  if changed=='manual':args['selected']=21.
  if changed=='settled':args['speed']=20.
  if changed=='blocked':args['blocked']=True
  if changed=='bad_limit':r=replace(r,limit=0.)
  assert ceiling(r,**args) is None

def test_native_cruise_shaping_does_not_relax_lead_stop_or_curve_braking():
  cp=CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  def run(request,lead=False,stop=False,force=False,coast=None,force_stop=None,throttle=True,relevant=None,legacy=False):
    planner=LongitudinalPlanner(cp,init_v=V_EGO)
    from openpilot.starpilot.longitudinal.profile_runtime import ProfileTuning
    tuning=ProfileTuning('standard',1.5,1.,1.,1.,1.,1.,acceleration_max=2.,cruise_brake_magnitude=1.2) if legacy else None
    for _ in range(65):
      sm,_=messages(lead=lead,force=force)
      class Checked(dict):
        def all_checks(self,services):return True
      sm=Checked(sm)
      sm.logMonoTime={'modelV2':1_000_000_000}
      sm['carState'].vCruise=50.
      sm['carState'].vEgoCluster=V_EGO
      sm['carControl'].longActive=True
      sm['modelV2'].action.shouldStop=stop
      sm['modelV2'].meta.disengagePredictions.gasPressProbs=[1.,1. if throttle else .1]
      from contextlib import nullcontext
      gate=patch('openpilot.selfdrive.controls.lib.longitudinal_planner.braking_lead_relevant',return_value=relevant) if relevant is not None else nullcontext()
      with gate:
        planner.update(sm,now_ns=1_000_000_000,slc_comfort=request,global_braking_response='standard',profile_tuning=tuning,
          wheel_coast=(NS(sample=lambda *a,**kw:coast),None) if coast is not None else None,
          force_stop_provider=(lambda _:replace(force_stop,model_ns=1_000_000_000)) if force_stop is not None else None)
    if force_stop is not None:
      assert planner.force_stop_plan==replace(force_stop,model_ns=1_000_000_000)
    return planner
  request=Adjustment(1_000_000_000,50/3.6,-1,.8)
  normal,soft=run(None),run(request)
  assert soft.a_cruise==pytest.approx(-.8) and normal.a_cruise==pytest.approx(-1.2)
  # The value/menu conversion preserves the pre-existing lead/throttle vetoes.
  assert run(request,relevant=True).a_cruise==run(None,relevant=True).a_cruise
  assert run(request,relevant=True,legacy=True).a_cruise==run(None,relevant=True,legacy=True).a_cruise
  assert run(request,throttle=False).a_cruise==run(None,throttle=False).a_cruise
  lead_base,lead_capped=run(None,lead=True),run(request,lead=True)
  assert lead_base.output_a_target < -.8
  assert (lead_base.output_a_target,lead_base.output_should_stop)==(lead_capped.output_a_target,lead_capped.output_should_stop)
  from openpilot.starpilot.controllers.coast import CoastPlan
  from openpilot.starpilot.longitudinal.force_stop import StopPlan
  for kwargs in ({'stop':True},{'force':True},{'coast':CoastPlan(brake_floor=-1.4)}):
    a,b=run(None,**kwargs),run(request,**kwargs)
    assert (a.output_a_target,a.output_should_stop,a.a_cruise)==(b.output_a_target,b.output_should_stop,b.a_cruise)


def test_manual_confirmation_retains_old_limit_baseline_until_ack():
  from openpilot.starpilot.speed_limits import acceptance as acc
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  from openpilot.starpilot.speed_limits.runtime import Runtime,Action
  from openpilot.starpilot.speed_limits.tests.test_runtime_replay import RuntimeReplayTests
  from openpilot.starpilot.speed_limits.tests.test_set_speed import tick as original_tick,command_receipt
  def tick(runtime,*a,**kw):
    runtime.comfort_limits={"acceleration":.7,"braking":.8}
    return original_tick(runtime,*a,**kw)
  f=RuntimeReplayTests();f.setUp();sm,cp=f.sm,f.cp
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=90.;sm['carState'].vEgoCluster=25.
  r=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard','SLCTransitionOffsets':True,'SLCDecelOffsetPct':-50.}))
  for i in range(62):tick(r,sm,cp,i*.05)
  assert r.comfort.baseline==('dashboard',25.)
  r.settings=replace(r.settings,acceptance=replace(r.settings.acceptance,confirm_lower=True))
  sm['slcDashboardObservation'].speedMps=20.
  pending=tick(r,sm,cp,3.1).message.slcState
  assert pending.hasPending and r.comfort.baseline==('dashboard',25.)
  out=tick(r,sm,cp,3.2,request=Action(r.session_id,1,pending.decisionId,pending.presentationId,'accept'))
  cmd=out.command.slcCruiseCommand
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=72.
  tick(r,sm,cp,3.25,cruise_event=command_receipt(r,cmd,3.25,90.,72.))
  assert r.comfort_request is not None and r.comfort_request.limit==.8
  deadline=r.comfort.deadline
  from openpilot.starpilot.speed_limits.runtime import CruiseEvent
  from openpilot.starpilot.speed_limits.tests.test_set_speed import START
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=75.6
  at=START+3_300_000_000
  event=CruiseEvent(2,at,20.,21.,'accel','card-producer',at)
  tick(r,sm,cp,3.3,cruise_event=event)
  assert r.comfort_request is not None and r.comfort_request.target==pytest.approx(21.)
  assert r.comfort_request.limit==.8 and r.comfort.deadline==deadline
  tick(r,sm,cp,3.35)
  assert r.comfort_request is not None and r.comfort_request.limit==.8
