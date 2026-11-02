"""SP-011 recorded early triggers and production cruise arrival regression."""
from dataclasses import replace
from types import SimpleNamespace as NS
import math
import pytest
from openpilot.starpilot.speed_limits.mapped_timing import BrakingProfile, MappedTiming, braking_distance, profile_from_host
from openpilot.starpilot.speed_limits.tests.test_mapped_timing import BASE, OBS, NOW, project
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.longitudinal.slc_coast import shaped_slc_floor
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
from openpilot.starpilot.longitudinal.set_speed_comfort import CruiseTransition

CP=NS(carFingerprint='TESLA_MODEL_3',steerRatio=15.,wheelbase=2.8)
class Host:
  disabled=False
  def sample(self,*a,**kw):return NS(cruise_brake_magnitude=1.,slc_braking_style='standard')
  def sample_selected(self,*a,**kw):return NS(cruise_brake_magnitude=1.,braking_style='standard')

def test_unqualified_control_frame_never_becomes_weak_profile():
  normal=profile_from_host(Host(),NOW,1,CP,enabled=True)
  assert normal.floor(18.)==-1.
  assert profile_from_host(Host(),NOW,1,CP,enabled=True,qualified=False) is None
  assert profile_from_host(Host(),NOW,1,CP,enabled=True,traffic=None) is None
  assert profile_from_host(None,NOW,1,CP,enabled=False).floor(18.)==-1.2

@pytest.mark.parametrize('distance',[0.,.001,100.,400.])
def test_profile_gap_cannot_start_lower_even_at_boundary_preview(distance):
  h=MappedTiming();h.profile=None
  result=project(h,distance)
  assert result.candidate.speed_mps==30. and not h.approach.active
  assert h.trigger==0. and math.isinf(h.eta)

def test_profile_recovery_and_latched_approach_preserve_authority():
  h=MappedTiming();h.profile=None
  project(h,5.)
  h.profile=BrakingProfile(floors=(-1.,-1.))
  assert project(h,5.,now=NOW+50_000_000).candidate.speed_mps==20.
  h.profile=None
  assert project(h,4.,now=NOW+100_000_000).candidate.speed_mps==20.
  assert h.approach.active
  # New source generation has no permission to inherit an approach.
  sample=replace(BASE,generation=BASE.generation+1,next_distance_m=4.)
  result=h.project(OBS,sample,now_ns=NOW+150_000_000,now_boot_ns=NOW+150_000_000,
    speed=30.,acceleration=0.,cluster_delta=0.,settings=parse({}),enabled=True)
  assert not h.approach.active and result.candidate.speed_mps==30.

def test_higher_distance_is_independent_of_lower_forecast_qualification():
  h=MappedTiming();h.profile=None
  assert project(h,111.,current=20.,upcoming=30.).candidate.speed_mps==20.
  assert project(h,110.,current=20.,upcoming=30.,now=NOW+50_000_000).candidate.speed_mps==30.

@pytest.mark.parametrize('speed,a,target,distance',[
  (17.316686630249023,1.1103944778442383,40.2/3.6,400.3137512207031),
  (17.91229820251465,-.004307325463742018,45.1/3.6,240.35684204101562),
])
def test_recorded_fallback_frames_do_not_activate(speed,a,target,distance):
  h=MappedTiming();settings=parse({'SpeedLimitController':True,'SLCPriority1':'Map Data',
    'SLCTransitionOffsets':True,'SLCDecelOffsetPct':-20,'SLCLowerLimitTiming':-2})
  sample=replace(BASE,speed_mps=40*.44704,next_speed_mps=20*.44704,next_distance_m=distance)
  obs=replace(OBS,candidate=replace(OBS.candidate,speed_mps=40*.44704))
  for i,qualified in enumerate((True,False,True,False,True)):
    h.profile=profile_from_host(Host(),NOW+i*50_000_000,1,CP,enabled=True,qualified=qualified)
    result=h.project(obs,sample,now_ns=NOW+i*50_000_000,now_boot_ns=NOW+i*50_000_000,
      speed=speed,acceleration=a,cluster_delta=0.,settings=settings,enabled=True,target_override=target)
    assert not h.approach.active and result.candidate.speed_mps==40*.44704

@pytest.mark.parametrize('percent',[-50,-20,0,50])
def test_no_acceleration_threshold_discontinuity_or_percentage_delay(percent):
  p=BrakingProfile(floors=(-1.,-1.),percent=percent,jerk=.4)
  assert abs(braking_distance(18.,11.,-.0499,p)-braking_distance(18.,11.,-.0501,p))<.1
  assert shaped_slc_floor(11.3,11.,-.8,approach=True)==-.8
  assert shaped_slc_floor(11.3,11.,-.8,approach=False)>-.8
  assert BrakingProfile(floors=(-.2,-.2),percent=-50).floor(18.)==-.175

@pytest.mark.parametrize('mph,target_mph',[(70,50),(50,30),(42,25),(40,28),(30,20)])
@pytest.mark.parametrize('percent',[-50,-20,0])
@pytest.mark.parametrize('initial_a',[-.3,0.,1.1])
@pytest.mark.parametrize('initial_rate',[-.4,0.,.35])
def test_forecast_tracks_production_cruise_with_command_handover(mph,target_mph,percent,initial_a,initial_rate):
  speed=mph*.44704;target=target_mph*.44704;p=BrakingProfile(floors=(-1.,-1.),percent=percent,jerk=.4)
  predicted=braking_distance(speed,target,initial_a,p)
  a=initial_a;distance=0.;t=0.;transition=CruiseTransition();started=False
  # Same selected profile/offset through the real native cruise implementation.
  # Vehicle response is an ideal integration, not a road-comfort claim.
  while speed>target+.05 and t<120:
    dt=.05
    if t>=.6-1e-9:
      if not started:
        transition.time_ns=int((t+.95)*1e9);transition.output=a;transition.rate=initial_rate;started=True
      magnitude=max(.175,1.*(1.+percent/100.))
      a=get_cruise_accel(False,target,speed,a,0.,CP,dt,0.,True,
        profile=NS(cruise_brake_magnitude=magnitude,acceleration_max=2.),selected_acceleration_max=2.,
        a_target_prev=a,set_speed_comfort=False,set_speed_slowing=True,profile_owned=True,
        transition=transition,transition_now_ns=int((t+1)*1e9),transition_enabled=True)
    v=max(0.,speed+a*dt);distance+=(speed+v)*.5*dt;speed=v;t+=dt
  assert t<120
  if initial_rate>=0:
    assert abs(predicted-distance)<max(2.,mph*.44704*.1)
  else:
    # An already-deepening brake request can arrive sooner; zero trend is the
    # conservative forecast. It must not underestimate the required distance.
    assert distance-2. <= predicted <= distance+mph*.44704*1.5

class MissingHost(Host):
  def sample(self,*a,**kw):return None
  def sample_selected(self,*a,**kw):return None

def test_missing_document_does_not_invent_soft_braking():
  assert profile_from_host(MissingHost(),NOW,1,CP,enabled=True) is None
  h=MissingHost();h.disabled=True
  assert profile_from_host(h,NOW,1,CP,enabled=True).floor(18.)==-1.2


def test_real_profile_freshness_gate_remains_twenty_ms():
  from openpilot.selfdrive.controls.plannerd import _profile_frame_valid
  class SM(dict):pass
  sm=SM(carState=NS(canValid=True,canTimeout=False))
  required=('carState','carControl','controlsState','selfdriveState')
  sm.valid={k:True for k in required};sm.alive=sm.valid.copy();sm.logMonoTime={k:NOW-10_000_000 for k in required}
  assert _profile_frame_valid(sm,NOW)
  sm.logMonoTime['carControl']=NOW-21_400_000
  assert not _profile_frame_valid(sm,NOW)
  assert profile_from_host(Host(),NOW,1,CP,enabled=True,qualified=_profile_frame_valid(sm,NOW)) is None

@pytest.mark.parametrize('direction',[-1,1])
@pytest.mark.parametrize('abort',[None,'unacknowledged','manual','pedal','force','authority','reengage','override','target','deadline','clock','disabled','error'])
def test_accepted_target_percentage_survives_requalified_source_but_not_driver_or_authority_abort(direction,abort):
  from openpilot.starpilot.speed_limits.transition_comfort import TransitionComfort
  from openpilot.starpilot.speed_limits.custom_settings import CustomSettings
  c=TransitionComfort();c.target=20.;c.direction=direction;c.started=NOW-500_000_000;c.deadline=NOW+10_000_000_000
  frame=NS(now_ns=NOW,authority=NS(can_control=True),pedal_pressed=False,host=NS(force_decel=False))
  # Acceptance changed identity/numeric current limit during the preview gap;
  # card still holds the previously acknowledged target.
  accepted=NS(candidate=NS(speed_mps=30.,source='map'))
  result=NS(state=NS(acceptance=NS(accepted=accepted),override=NS(persistent_selected_mps=None)),
    selection=NS(observation=NS(candidate=accepted.candidate)),domain=NS(context=NS(effective_cluster_mps=30.)),errors=())
  settings=CustomSettings(transition_offsets=True,accel_percent=-50.,decel_percent=-20.)
  args=dict(settings=settings,selected=20.,speed=20.-direction*3.,acknowledged=False,selected_acknowledged=True,manual=False,reengage=False)
  if abort=='unacknowledged':args['selected_acknowledged']=False
  elif abort=='manual':args['manual']=True
  elif abort=='pedal':frame.pedal_pressed=True
  elif abort=='force':frame.host.force_decel=True
  elif abort=='authority':frame.authority.can_control=False
  elif abort=='reengage':args['reengage']=True
  elif abort=='override':result.state.override.persistent_selected_mps=20.
  elif abort=='target':args['selected']=21.
  elif abort=='deadline':frame.now_ns=c.deadline+1
  elif abort=='clock':frame.now_ns=c.started-1
  elif abort=='disabled':args['settings']=replace(settings,transition_offsets=False)
  elif abort=='error':result.errors=('error',)
  value=c.step(result,frame,**args)
  if abort is None:
    assert value.target==20. and value.percent==(-50. if direction>0 else -20.)
  else:assert value is None


def test_recalled_manual_target_forecast_uses_ordinary_profile_not_transition_percentage():
  h=MappedTiming();h.profile=BrakingProfile(floors=(-1.,-1.),jerk=.4)
  settings=parse({'SpeedLimitController':True,'SLCPriority1':'Map Data','SLCTransitionOffsets':True,'SLCDecelOffsetPct':-50})
  h.project(OBS,BASE,now_ns=NOW,now_boot_ns=NOW,speed=30.,acceleration=0.,cluster_delta=0.,
    settings=settings,enabled=True,target_override=23.)
  # Imperial selected speed is rounded to0.1kph by card.
  from openpilot.selfdrive.car.cruise import slc_target_kph
  expected=braking_distance(30.,slc_target_kph(23.,False)/3.6,0.,h.profile)
  assert h.trigger==pytest.approx(expected)
