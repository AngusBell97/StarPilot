from dataclasses import replace
import math
import pytest
from openpilot.starpilot.speed_limits.mapped_timing import MappedTiming, BrakingProfile, braking_distance
from openpilot.starpilot.speed_limits.map_source import MapFrame
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.speed_limits import acceptance as a

NOW=10_000_000_000
OBS=a.Observation(a.ObservationKind.VALID,a.Candidate('map',a.ObservationIdentity(a.IdentityKind.GEOGRAPHIC,value='road'),30.))
BASE=MapFrame(True,NOW,2,'matchedLimit','external',NOW,NOW,1,41,30.,True,12,'current',20.,500.,'road')

def project(host, distance, *, current=30.,upcoming=20.,speed=30.,timing=0.,higher=110.,now=NOW,**changes):
  settings=parse({'SpeedLimitController':True,'SLCPriority1':'Maps','SLCLowerLimitTiming':timing,'SLCHigherLimitDistance':higher,**changes})
  sample=replace(BASE,event_mono_ns=now,computed_mono_ns=now,next_distance_m=distance,next_speed_mps=upcoming,speed_mps=current)
  obs=replace(OBS,candidate=replace(OBS.candidate,speed_mps=current))
  return host.project(obs,sample,now_ns=now,now_boot_ns=now,speed=speed,acceleration=0.,cluster_delta=0.,settings=settings,enabled=True)

@pytest.mark.parametrize('speed,target',[(30.,20.),(31.2928,13.4112),(22.352,20.1168),(10.,9.95)])
def test_forecast_uses_braking_strength_and_returns_finite_distance(speed,target):
  weak=braking_distance(speed,target,0.,BrakingProfile(floors=(-.25,-.25)))
  normal=braking_distance(speed,target,0.,BrakingProfile())
  assert math.isfinite(weak) and weak>=normal>=0

def test_positive_acceleration_and_weaker_transition_offset_start_earlier():
  normal=braking_distance(30.,20.,0.,BrakingProfile())
  assert braking_distance(30.,20.,1.,BrakingProfile())>normal
  assert braking_distance(30.,20.,0.,BrakingProfile(percent=-50))>normal
  assert braking_distance(30.,20.,0.,BrakingProfile(percent=50))==normal

def test_lower_timing_is_metres_from_current_speed_not_target_speed():
  h=MappedTiming();project(h,1000.);normal=h.trigger
  h=MappedTiming();project(h,1000.,timing=-4.)
  assert h.trigger==pytest.approx(max(0.,normal-30.*4.))
  assert project(h,h.trigger+.1,timing=-4.).candidate.speed_mps==30.
  assert project(h,h.trigger-.1,timing=-4.,now=NOW+50_000_000).candidate.speed_mps==20.

def test_higher_limit_adoption_starts_at_requested_distance():
  h=MappedTiming()
  assert project(h,111.,current=20.,upcoming=30.,speed=20.).candidate.speed_mps==20.
  assert project(h,110.,current=20.,upcoming=30.,speed=20.,now=NOW+50_000_000).candidate.speed_mps==30.

def test_confirmed_boundary_commits_without_restarting_numeric_change():
  h=MappedTiming();assert project(h,5.).candidate.speed_mps==20.
  result=project(h,500.,current=20.,upcoming=30.,now=NOW+50_000_000)
  assert result.candidate.speed_mps==20. and h.approach.phase=='commit'

@pytest.mark.parametrize('kind',[a.ObservationKind.UNKNOWN,a.ObservationKind.STALE,a.ObservationKind.ABSENT])
def test_missing_source_cannot_create_forecast_or_extend_authority(kind):
  h=MappedTiming();project(h,5.)
  obs=a.Observation(kind)
  assert h.project(obs,BASE,now_ns=NOW,now_boot_ns=NOW,speed=30.,acceleration=0.,cluster_delta=0.,settings=parse({}),enabled=True)==obs
  assert not h.approach.active

def test_zero_delay_adoption_still_requires_current_authority_and_confirmation_policy():
  authority=a.Authority(a.Mode.COMBINED,a.LongitudinalOwner.SYSTEM,True,True,False,False)
  immediate=a.step(a.new_session('drive'),OBS,authority,a.Policy(False,False,automatic_adoption_ns=0),now_ns=NOW)
  assert immediate.state.accepted and immediate.basis=='accepted'
  waiting=a.step(a.new_session('drive'),OBS,authority,a.Policy(True,True,automatic_adoption_ns=0),now_ns=NOW)
  assert waiting.state.pending and not waiting.state.accepted
  invalid=a.step(a.new_session('drive'),OBS,authority,a.Policy(False,False,automatic_adoption_ns=-1),now_ns=NOW)
  assert invalid.errors

def test_profile_changes_and_source_generation_invalidate_forecast_cache():
  h=MappedTiming();project(h,1000.);old=h.trigger
  h.profile=BrakingProfile(floors=(-.25,-.25))
  project(h,1000.,now=NOW+50_000_000)
  assert h.trigger>old

def test_warning_once_before_target_change_without_replaying_after_busy_or_gap():
  h=MappedTiming();settings=parse({'SLCChangeWarning':True})
  h.identity=h.warning_identity='boundary';h.eta=5.;h.remaining=120.
  h.warn(NOW,eligible=True,target_change=True,speed=20.,settings=settings)
  assert h.warning_event is None
  h.eta=1.9;h.warn(NOW+50_000_000,eligible=True,target_change=True,speed=20.,settings=settings)
  assert h.warning_event['id']=='boundary' and h.warning_pin['warningPinned']
  h.warn(NOW+400_000_000,eligible=True,target_change=True,speed=20.,settings=settings)
  assert h.warning_event is None


def test_selected_braking_uses_real_custom_knots_without_missing_a_trough():
  from types import SimpleNamespace as NS
  from openpilot.starpilot.speed_limits.mapped_timing import profile_from_host
  knot=20*.44704
  class Host:
    def sample(self,*args,**kwargs):return None
    def sample_selected(self,now,personality,speed,cp,**kwargs):
      return NS(cruise_brake_magnitude=min(2.,.5+abs(speed-knot)))
  profile=profile_from_host(Host(),1,1,None,enabled=True)
  assert knot in profile.speeds and profile.floor(knot)==-.5
  assert len(profile.speeds)<25
