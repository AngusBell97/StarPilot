"""SLC-only caps: migration, driver intervention, timing and native settings."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace as NS
import json
import pytest
from openpilot.starpilot.longitudinal.profile_document import (profile_document, default_personality_profiles,
  migrate_profile_document, strict_profile_document, serialize_personality_profiles)
from openpilot.starpilot.speed_limits.transition_comfort import TransitionComfort, ceiling
from openpilot.starpilot.speed_limits.custom_settings import CustomSettings
from openpilot.starpilot.speed_limits.mapped_timing import BrakingProfile, braking_distance
from openpilot.starpilot.speed_limits.tests.test_transition_comfort import comfort_input
from openpilot.common.params import Params
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest

LIMITS={'acceleration':.7,'braking':.8}


def test_v5_migrates_without_changing_profiles_or_saved_bytes():
  old=profile_document(default_personality_profiles(False),enabled=True)
  old.pop('slcLimits');old['schemaVersion']=5
  raw=json.dumps(old);new=migrate_profile_document(raw)
  assert new['schemaVersion']==6 and new['profiles']==old['profiles']
  assert all(v==LIMITS for v in new['slcLimits'].values())
  assert json.loads(raw)==old


@pytest.mark.parametrize('bad',[None,True,0.,float('nan'),float('inf'),-1.,3.6,'0.7'])
def test_invalid_limit_cannot_activate(bad):
  doc=profile_document(default_personality_profiles(False),enabled=True)
  doc['slcLimits']['relaxed']['acceleration']=bad
  assert strict_profile_document(doc) is None


@pytest.mark.parametrize('direction',[-1,1])
@pytest.mark.parametrize('kind',['wheel','bluetooth','pedal'])
def test_active_limit_survives_new_manual_target_and_pedal(direction,kind):
  c=TransitionComfort();args=dict(settings=CustomSettings(transition_offsets=True),limits=LIMITS,selected=20.,speed=20.-direction*5,
    acknowledged=True,selected_acknowledged=True,manual=False,reengage=False)
  c.step(*comfort_input(10. if direction>0 else 30.),**args)
  assert c.step(*comfort_input(20.,now=1_050_000_000),**args) is not None
  deadline=c.deadline
  new=22. if direction>0 else 18.
  if kind=='pedal':
    assert c.step(*comfort_input(20.,now=1_100_000_000,pedal=True),**dict(args,speed=25.)) is None
    assert c.target==20.
  else:
    assert c.step(*comfort_input(20.,now=1_100_000_000),**dict(args,manual=True)) is not None
  request=c.step(*comfort_input(20.,now=1_150_000_000),**dict(args,selected=new,acknowledged=False,selected_acknowledged=False))
  assert request.target==new and request.limit==LIMITS['acceleration' if direction>0 else 'braking']
  assert c.deadline==deadline
  assert ceiling(request,now_ns=request.now_ns,selected=new,speed=args['speed'],blocked=False)==request.limit
  assert c.step(*comfort_input(20.,now=deadline+1),**dict(args,selected=new)) is None


def test_manual_speed_change_alone_never_starts_slc_limits():
  c=TransitionComfort();args=dict(settings=CustomSettings(transition_offsets=True),limits=LIMITS,speed=15.,acknowledged=True,manual=True,reengage=False)
  for n in range(10):
    assert c.step(*comfort_input(20.,now=1_000_000_000+n*50_000_000),**dict(args,selected=20.+n)) is None


def test_manual_change_before_slc_acknowledgement_cannot_start_caps():
  c=TransitionComfort();args=dict(settings=CustomSettings(transition_offsets=True),limits=LIMITS,selected=10.,speed=15.,
    acknowledged=False,selected_acknowledged=False,manual=False,reengage=False)
  c.step(*comfort_input(10.),**args)
  assert c.step(*comfort_input(20.,now=1_050_000_000),**dict(args,selected=18.,manual=True)) is None
  assert not c.armed
  assert c.step(*comfort_input(20.,now=2_100_000_000),**dict(args,selected=18.)) is None
  assert c.target is None


def test_profile_gap_pauses_without_forgetting_or_extending_active_transition():
  c=TransitionComfort();args=dict(settings=CustomSettings(transition_offsets=True),limits=LIMITS,selected=20.,speed=15.,
    acknowledged=True,manual=False,reengage=False)
  c.step(*comfort_input(10.),**args)
  assert c.step(*comfort_input(20.,now=1_050_000_000),**args).limit==.7
  deadline=c.deadline
  for now in (1_100_000_000,2_100_000_000):
    assert c.step(*comfort_input(20.,now=now),**dict(args,limits=None)) is None
    assert c.target==20. and c.deadline==deadline
  assert c.step(*comfort_input(20.,now=2_150_000_000),**args).limit==.7


def test_lower_braking_cap_increases_forecast_and_gentler_profile_is_preserved():
  base=BrakingProfile(floors=(-1.2,-1.2),limit=.8,easing=False)
  softer=replace(base,limit=.4)
  assert braking_distance(30.,15.,0.,softer)>braking_distance(30.,15.,0.,base)
  assert replace(base,floors=(-.5,-.5)).floor(20.)==-.5


def test_native_editor_round_trip_preserves_caps_and_other_profile_choices(tmp_path):
  params=Params(str(tmp_path));cp=NS(carFingerprint='TEST MODEL',openpilotLongitudinalControl=True,pcmCruise=False,
    passive=False,notCar=False,dashcamOnly=False,carVin='VIN',transmissionType=0)
  owner=FeatureSettingsOwner(params,lambda _:True,vehicle_fingerprint=lambda:cp.carFingerprint,vehicle_params=lambda:cp)
  def row(page,key):return next(r for r in owner.snapshot(page,parked=True,system_long=True,lateral_context=True,metric=False).rows if r.key==key)
  def apply(page,key,value):
    r=row(page,key)
    assert r.available
    assert owner.apply(FeatureSettingsRequest(r.key,r.source,value,vehicle_fingerprint=r.vehicle_fingerprint,
      capability=r.capability,dependencies=r.dependencies))
  for name in ('relaxed','standard','aggressive'):
    assert row(name,f'profile:{name}:acceleration:slc_limit').value=='0.7'
    assert row(name,f'profile:{name}:braking:slc_limit').value=='0.8'
    assert row(name,f'profile:{name}:acceleration:slc_limit').unit=='m/s\u00b2'
  apply('relaxed','profile:relaxed:acceleration:slc_limit','0.5')
  apply('relaxed','profile:relaxed:braking:slc_limit','0.9')
  apply('relaxed','profile:relaxed:acceleration','Comfort')
  doc=migrate_profile_document(params.get('LongitudinalPersonalityProfiles'))
  assert doc['slcLimits']['relaxed']=={'acceleration':.5,'braking':.9}
  assert doc['slcLimits']['standard']==LIMITS
  text=owner._edit_document('profile:global_acceleration','Sport')
  assert migrate_profile_document(text)['slcLimits']==doc['slcLimits']
  slc=owner.snapshot('slc',parked=True,system_long=True,lateral_context=True,metric=False)
  assert not any(r.page=='slc/comfort' or r.key in ('SLCAccelOffsetPct','SLCDecelOffsetPct','SLCTransitionOffsets') for r in slc.rows)
  assert row('profiles','SLCTransitionOffsets').value=='Off'
  apply('profiles','SLCTransitionOffsets','On')
  assert params.get_bool('SLCTransitionOffsets')
  assert row('profiles','SLCTransitionOffsets').value=='On'


def test_existing_enable_switch_still_controls_caps_and_forecast():
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  from openpilot.starpilot.speed_limits.tests.test_mapped_timing import project
  from openpilot.starpilot.speed_limits.mapped_timing import MappedTiming
  disabled=CustomSettings(transition_offsets=False)
  assert parse({}).custom==disabled
  c=TransitionComfort();args=dict(settings=disabled,limits=LIMITS,selected=20.,speed=15.,
    acknowledged=True,manual=False,reengage=False)
  assert c.step(*comfort_input(10.),**args) is None
  assert c.step(*comfort_input(20.,now=1_050_000_000),**args) is None
  assert c.target is None
  off=MappedTiming();off.profile=BrakingProfile(floors=(-1.2,-1.2),limit=.3)
  on=MappedTiming();on.profile=off.profile
  project(off,1000.,SLCTransitionOffsets=False)
  project(on,1000.,SLCTransitionOffsets=True)
  assert on.trigger>off.trigger


def test_disabled_profile_forecast_keeps_existing_base_with_new_cap():
  from openpilot.starpilot.speed_limits.mapped_timing import profile_from_host
  cp=NS(carFingerprint='TESLA_MODEL_3')
  assert profile_from_host(None,0,1,cp,enabled=False,slc_limit=.8).floor(20.)==-.8
  assert profile_from_host(None,0,1,cp,enabled=False,slc_limit=.3).floor(20.)==-.3
  assert profile_from_host(None,0,1,cp,enabled=False,slc_limit=None) is None


def test_preexisting_manual_target_keeps_old_entry_policy_but_active_manual_change_persists():
  c=TransitionComfort();args=dict(settings=CustomSettings(transition_offsets=True),limits=LIMITS,selected=20.,speed=15.,
    acknowledged=True,manual=False,reengage=False)
  c.step(*comfort_input(10.),**args)
  result,frame=comfort_input(20.,now=1_050_000_000)
  result.state.override.persistent_selected_mps=20.
  assert c.step(result,frame,**args) is None
  c=TransitionComfort();c.step(*comfort_input(10.),**args)
  assert c.step(*comfort_input(20.,now=1_050_000_000),**args) is not None
  frame.now_ns=1_100_000_000
  assert c.step(result,frame,**dict(args,selected=21.,manual=True)).target==21.


def test_obsolete_percentage_bytes_cannot_disable_slc_or_scale_limits():
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  current=parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard'})
  old=parse({'SpeedLimitController':True,'SLCPriority1':'Dashboard',
    'SLCAccelOffsetPct':'corrupt','SLCDecelOffsetPct':-50,'SLCTransitionOffsets':False})
  assert old==current


def test_each_personality_has_independent_caps_including_traffic(tmp_path):
  from openpilot.starpilot.longitudinal.profile_runtime import ProfileHost
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import CAR
  params=Params(str(tmp_path));cp=CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  doc=profile_document(default_personality_profiles(False),enabled=False)
  for n,name in enumerate(('aggressive','standard','relaxed','traffic')):
    doc['slcLimits'][name]={'acceleration':.4+n*.1,'braking':.5+n*.1}
  params.put('LongitudinalPersonalityProfiles',doc,block=True)
  host=ProfileHost(params)
  for n,name in enumerate(('aggressive','standard','relaxed','traffic')):
    assert host.sample_slc_limits(1_000_000_000,n if n<3 else 1,cp,traffic_mode=n==3)==doc['slcLimits'][name]


def test_missing_profile_uses_defaults_but_invalid_profile_has_no_cap(tmp_path):
  from openpilot.starpilot.longitudinal.profile_runtime import ProfileHost
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import CAR
  params=Params(str(tmp_path));cp=CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  host=ProfileHost(params)
  assert host.sample_slc_limits(1_000_000_000,1,cp)==LIMITS
  params.put('LongitudinalPersonalityProfiles',{'schemaVersion':999},block=True)
  assert host.sample_slc_limits(2_000_000_000,1,cp) is None


def test_pedal_crossing_original_target_waits_for_capture_and_direction_can_reverse():
  c=TransitionComfort();args=dict(settings=CustomSettings(transition_offsets=True),limits=LIMITS,selected=20.,speed=15.,
    acknowledged=True,manual=False,reengage=False)
  c.step(*comfort_input(10.),**args)
  assert c.step(*comfort_input(20.,now=1_050_000_000),**args).limit==.7
  deadline=c.deadline
  assert c.step(*comfort_input(20.,now=1_100_000_000,pedal=True),**dict(args,speed=23.)) is None
  assert c.step(*comfort_input(20.,now=1_150_000_000),**dict(args,speed=23.)) is None
  assert c.target==20.
  # Card publishes the captured target after release. The same episode survives.
  request=c.step(*comfort_input(20.,now=1_200_000_000),**dict(args,selected=24.,speed=23.,manual=True))
  assert request.limit==.7 and request.target==24.
  request=c.step(*comfort_input(20.,now=1_250_000_000),**dict(args,selected=18.,speed=23.,manual=True))
  assert request.limit==.8 and request.direction==-1 and c.deadline==deadline
