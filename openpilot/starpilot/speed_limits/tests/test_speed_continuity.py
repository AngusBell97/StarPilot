from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from openpilot.starpilot.speed_limits.manual_targets import PerLimitTargets
from openpilot.starpilot.speed_limits.runtime import Runtime,CruiseEvent
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.speed_limits.tests.test_set_speed import drive,tick,command_receipt,START
from openpilot.starpilot.longitudinal.set_speed_comfort import SetSpeedComfort,braking_lower_bound,owns_selected_target
from openpilot.starpilot.longitudinal.slc_coast import EasingRequest
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel

MPH=.44704

def observe(c,t,limit,fresh=True,enabled=True):
  c.observe(round(t*1e9),limit,enabled=enabled,fresh=fresh)

def test_separate_targets_return_before_thirty_seconds():
  c=PerLimitTargets();observe(c,0,30*MPH);c.remember(33*MPH)
  for i in range(1,201):observe(c,i*.05,40*MPH)
  c.remember(42*MPH)
  for i in range(201,401):observe(c,i*.05,40*MPH)
  observe(c,20.05,30*MPH)
  assert c.target(30*MPH)==33*MPH and c.target(40*MPH)==42*MPH
  assert c.entries[c.key(30*MPH)].away_ns==0

def test_thirty_seconds_aggregate_other_limits_and_disengagement():
  c=PerLimitTargets();observe(c,0,30*MPH);c.remember(33*MPH)
  for i in range(1,602):
    observe(c,i*.05,(40 if i<300 else 50)*MPH)
  assert c.target(30*MPH) is None

def test_unknown_time_pauses_and_stalls_do_not_age():
  c=PerLimitTargets();observe(c,0,30*MPH);c.remember(33*MPH)
  for i in range(1,401):observe(c,i*.05,40*MPH)
  for i in range(401,2401):observe(c,i*.05,None)
  observe(c,180,40*MPH)
  assert c.target(30*MPH)==33*MPH
  for i in range(1,203):observe(c,180+i*.05,50*MPH)
  assert c.target(30*MPH) is None

def test_disable_restart_and_backwards_clock_clear_memory():
  for change in ('disable','clock','restart'):
    c=PerLimitTargets();observe(c,1,30*MPH);c.remember(33*MPH)
    if change=='disable':observe(c,1.05,30*MPH,enabled=False)
    elif change=='clock':observe(c,.5,30*MPH)
    else:c=PerLimitTargets()
    assert c.target(30*MPH) is None

def test_native_accel_for_slc_not_extra_point_six_ceiling():
  cp=NS(steerRatio=15.,wheelbase=2.8)
  args=(False,25.,15.,1.3,0.,cp,.05,0.,True)
  normal=get_cruise_accel(*args,selected_acceleration_max=1.3,set_speed_comfort=True)
  owned=get_cruise_accel(*args,selected_acceleration_max=1.3,set_speed_comfort=True,profile_owned=True)
  half=get_cruise_accel(*args,selected_acceleration_max=.65,set_speed_comfort=True,profile_owned=True)
  assert normal==pytest.approx(1.3) and owned==pytest.approx(1.3)
  # A reduced ceiling is approached at bounded native jerk, not an instant step.
  a=1.3
  for _ in range(40):a=get_cruise_accel(False,25.,15.,a,0.,cp,.05,0.,True,selected_acceleration_max=.65,set_speed_comfort=True,profile_owned=True)
  assert a==pytest.approx(.65)

def test_positive_release_native_then_gentle_braking_and_ownership_reset():
  assert braking_lower_bound(1.,.05,1.,.4)==pytest.approx(.95)
  assert braking_lower_bound(.02,.05,1.,.4)==pytest.approx(-.012)
  assert braking_lower_bound(-.2,.05,1.,.4)==pytest.approx(-.22)
  c=SetSpeedComfort();cp=NS(carFingerprint='TESLA_MODEL_3',openpilotLongitudinalControl=True)
  c.update(cp,selected=20.,speed=15.,enabled=True)
  c.update(cp,selected=25.,speed=15.,enabled=True,profile_owned=True)
  assert c.profile_owned
  c.update(cp,selected=25.,speed=16.,enabled=True)
  assert c.profile_owned # source dropout cannot reintroduce the extra cap
  c.update(cp,selected=26.,speed=16.,enabled=True)
  assert not c.profile_owned
  c.update(cp,selected=27.,speed=16.,enabled=True,profile_owned=True)
  c.update(cp,selected=27.,speed=16.,enabled=False)
  assert not c.profile_owned
  assert owns_selected_target(EasingRequest(100,20.,False),100,20.)
  assert not owns_selected_target(EasingRequest(100,20.,False),100_000_101,20.)

def test_runtime_separate_targets_reengage_and_temporary_zone(drive):
  sm,cp=drive
  r=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard','SLCReengageSpeed':True,'SLCRememberManualOffset':True}))
  sm['carState'].vEgo=sm['carState'].vEgoCluster=30*MPH
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=30*MPH*3.6
  sm['slcDashboardObservation'].speedMps=30*MPH
  time=0.;eid=0;commands=[]
  def frame(manual=None,engaged=True,zone=None):
    nonlocal time,eid
    time=round(time+.05,2)
    sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=engaged
    sm['controlsState'].longControlState='pid' if engaged else 'off'
    if zone is not None:
      sm['slcDashboardObservation'].speedMps=zone*MPH
    event=None
    if manual is not None:
      before=sm['carState'].vCruise/3.6;after=manual*MPH
      sm['carState'].vCruise=sm['carState'].vCruiseCluster=after*3.6
      eid+=1;at=START+round(time*1e9)
      event=CruiseEvent(eid,at,before,after,'accel' if after>before else 'decel','card-producer',at)
    out=tick(r,sm,cp,time,cruise_event=event)
    assert not out.result.errors
    if out.command is not None:
      cmd=out.command.slcCruiseCommand;commands.append(cmd.targetMps/MPH)
      before=sm['carState'].vCruise;after=cmd.targetMps*3.6
      sm['carState'].vCruise=sm['carState'].vCruiseCluster=after
      time=round(time+.05,2);eid+=1
      ack=tick(r,sm,cp,time,cruise_event=command_receipt(r,cmd,time,before,after,event_id=eid))
      assert ack.message.slcState.commandStatus=='applied'
    return out
  for _ in range(70):frame()
  frame(manual=33)
  for _ in range(10):frame(engaged=False)
  frame()
  assert sm['carState'].vCruise/3.6/MPH==pytest.approx(33)
  for _ in range(80):frame(zone=40)
  assert sm['carState'].vCruise/3.6/MPH==pytest.approx(40,abs=.01)
  frame(manual=42)
  for _ in range(80):frame(zone=30)
  assert sm['carState'].vCruise/3.6/MPH==pytest.approx(33,abs=.01)
  for _ in range(80):frame(zone=40)
  assert sm['carState'].vCruise/3.6/MPH==pytest.approx(42,abs=.01)


def test_target_shaping_survives_gap_only_for_acknowledged_selected_target():
  from openpilot.starpilot.speed_limits.transition_comfort import TransitionComfort
  from openpilot.starpilot.speed_limits.custom_settings import CustomSettings
  c=TransitionComfort();c.target=25.;c.direction=1;c.deadline=10_000_000_000;c.started=0
  frame=NS(now_ns=1_000_000_000,authority=NS(can_control=True),pedal_pressed=False,host=NS(force_decel=False))
  result=NS(state=NS(acceptance=NS(accepted=None),override=NS(persistent_selected_mps=None)),selection=None,domain=None,errors=())
  settings=CustomSettings(transition_offsets=True)
  args=dict(settings=settings,limits={"acceleration":.7,"braking":.8},selected=25.,speed=20.,acknowledged=True,manual=False,reengage=False)
  assert c.step(result,frame,**args).limit==.7
  assert c.step(result,frame,**dict(args,selected=26.)).limit==.7
  assert c.target == 26.

def test_remembered_lower_target_changes_braking_forecast():
  from openpilot.starpilot.speed_limits.tests.test_mapped_timing import BASE,OBS,NOW
  from openpilot.starpilot.speed_limits.mapped_timing import MappedTiming
  settings=parse({'SpeedLimitController':True,'SLCPriority1':'Map Data'})
  args=dict(now_ns=NOW,now_boot_ns=NOW,speed=30.,acceleration=1.,cluster_delta=0.,settings=settings,enabled=True)
  normal=MappedTiming();normal.project(OBS,BASE,**args)
  recalled=MappedTiming();recalled.project(OBS,BASE,**args,target_override=23.)
  assert recalled.trigger<normal.trigger

def test_memory_ages_during_disengagement_without_issuing_commands(drive):
  sm,cp=drive
  r=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard','SLCReengageSpeed':True,'SLCRememberManualOffset':True}))
  sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=False
  sm['controlsState'].longControlState='off'
  sm['slcDashboardObservation'].speedMps=30*MPH
  tick(r,sm,cp,0);r.manual_targets.remember(33*MPH)
  for i in range(1,602):
    sm['slcDashboardObservation'].speedMps=(40 if i<200 else 50)*MPH
    assert tick(r,sm,cp,i*.05).command is None
  assert r.manual_targets.target(30*MPH) is None


def test_reengage_during_missing_source_recalls_when_current_evidence_returns(drive):
  sm,cp=drive
  r=Runtime(parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard','SLCReengageSpeed':True,'SLCRememberManualOffset':True}))
  sm['carState'].vEgo=sm['carState'].vEgoCluster=25.
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=90.
  for i in range(65):tick(r,sm,cp,i*.05)
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=100.
  at=START+3_300_000_000
  tick(r,sm,cp,3.3,cruise_event=CruiseEvent(1,at,25.,100/3.6,'accel','card-producer',at))
  sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=False
  sm['controlsState'].longControlState='off'
  sm.valid['slcDashboardObservation']=False
  tick(r,sm,cp,3.35)
  sm['carState'].vCruise=sm['carState'].vCruiseCluster=90.
  sm['selfdriveState'].enabled=sm['carControl'].enabled=sm['carControl'].longActive=True
  sm['controlsState'].longControlState='pid'
  for i in range(68,190):
    assert tick(r,sm,cp,i*.05).command is None
  sm.valid['slcDashboardObservation']=True
  out=tick(r,sm,cp,9.5)
  assert out.command is not None and out.command.slcCruiseCommand.targetMps==pytest.approx(100/3.6)
