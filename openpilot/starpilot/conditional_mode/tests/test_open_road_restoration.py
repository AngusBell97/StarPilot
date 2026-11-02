from dataclasses import replace
from types import SimpleNamespace as NS
import pytest
from openpilot.starpilot.conditional_mode.policy import *
from openpilot.starpilot.conditional_mode.open_road import OpenRoad, MPH, following_gap
from openpilot.starpilot.conditional_mode.tests.test_policy import scene, AUTHORITY, NO_LEAD

SETTINGS = replace(ModeSettings(), cem_open_road=True)

class Trace:
  def __init__(self):
    self.p = ConditionalModePolicy()
    self.now = 10.
  def step(self, **kwargs):
    self.now = round(self.now + .05, 6)
    s = scene(self.now, speed=20., set_speed_mps=20., tesla_open_road=True,
              open_road_observed_s=self.now, open_road_model_s=self.now,
              open_road_model_accel=.1, open_road_model_stop=False,
              open_road_headway_s=1.5, open_road_lateral_m=0.)
    if 'speed' in kwargs: kwargs['speed_mps'] = kwargs.pop('speed')
    auth = kwargs.pop('authority', AUTHORITY)
    manual = kwargs.pop('manual', ManualIntent.NONE)
    settings = kwargs.pop('settings', SETTINGS)
    return self.p.step(self.now, ModeChoice.CEM, manual, auth, replace(s, **kwargs), settings)
  def enter(self, **kwargs):
    for i in range(41):
      result = self.step(**kwargs)
      assert result.requested_experimental is (i == 40)
    return result

LEAD = LeadEvidence(True, True, False, 40., 20., 0., .95)

def test_entry_and_retry_require_two_seconds_and_model_not_braking():
  t=Trace();t.enter()
  for i in range(42):
    r=t.step(speed=19.)
    assert r.requested_experimental is (i <= 40)
  for _ in range(50): assert not t.step(open_road_model_accel=-.01).requested_experimental
  t.enter()

@pytest.mark.parametrize('kwargs', [dict(open_road_model_stop=True),dict(open_road_model_accel=None),
  dict(open_road_model_accel=float('nan')),dict(open_road_model_accel=-1.),
  dict(open_road_model_s=0.),dict(open_road_observed_s=None),dict(lead=None),dict(following_lead=None),
  dict(lead=LEAD),dict(set_speed_mps=255.),dict(set_speed_mps=None),dict(speed=20.+MPH+.0001)])
def test_entry_denials(kwargs):
  t=Trace()
  for _ in range(61): assert not t.step(**kwargs).requested_experimental

def test_distant_untracked_detection_does_not_block_entry_or_force_handoff():
  t=Trace();distant=replace(LEAD,tracked=False,distance_m=100.)
  t.enter(lead=distant)
  for _ in range(100): assert t.step(lead=distant,speed=18.).requested_experimental

def test_small_deficit_is_retained_and_exact_one_mph_does_not_recover():
  t=Trace();t.enter()
  for _ in range(100): assert t.step(speed=20.-MPH).requested_experimental

def test_stable_follow_handoff_has_no_extra_generic_hold():
  t=Trace();t.enter()
  for i in range(11):
    r=t.step(lead=LEAD,following_lead=True)
    assert r.requested_experimental is (i<10)

@pytest.mark.parametrize('kwargs',[dict(lead=replace(LEAD,accel_mps2=-.201)),
  dict(lead=replace(LEAD,model_probability=.89)),dict(lead=replace(LEAD,distance_m=35.)),
  dict(lead=replace(LEAD,speed_mps=17.)),dict(open_road_lateral_m=1.801),
  dict(open_road_lateral_m=None),dict(open_road_headway_s=None),dict(open_road_headway_s=3.)])
def test_unsafe_or_unknown_following_keeps_experimental(kwargs):
  t=Trace();t.enter()
  args=dict(lead=LEAD,following_lead=True);args.update(kwargs)
  for _ in range(61): assert t.step(**args).requested_experimental

def test_replacement_or_renewed_braking_restarts_stability():
  for replacement in (replace(LEAD,distance_m=50.),replace(LEAD,accel_mps2=-1.)):
    t=Trace();t.enter()
    for _ in range(9): assert t.step(lead=LEAD,following_lead=True).requested_experimental
    assert t.step(lead=replacement,following_lead=True).requested_experimental
    for _ in range(9): assert t.step(lead=LEAD,following_lead=True).requested_experimental

def test_lead_arrives_before_recovery_and_loss_dwell_starts_new_timer():
  t=Trace();t.enter()
  for _ in range(40): assert t.step(speed=18.).requested_experimental
  assert t.step(speed=18.,lead=replace(LEAD,speed_mps=10.)).requested_experimental
  for i in range(52):
    assert t.step(speed=18.).requested_experimental is (i<=50)

@pytest.mark.parametrize('change',[dict(manual=ManualIntent.FORCE_CHILL),dict(authority=replace(AUTHORITY,safe_mode=True)),
  dict(authority=replace(AUTHORITY,long_active=False)),dict(settings=ModeSettings()),dict(standstill=True,standstill_stop_hold=False)])
def test_lifecycle_resets_open_road(change):
  t=Trace();t.enter();t.step(**change)
  assert not t.p.open_road.active
  # Stop-release suppression may extend entry; it must not make entry instant.
  assert not t.step().requested_experimental

def test_set_speed_change_and_duplicate_samples_restart_entry():
  t=Trace()
  for _ in range(30): t.step()
  t.step(open_road_observed_s=t.now)
  for _ in range(40): assert not t.step().requested_experimental
  assert t.step().requested_experimental
  assert not t.step(set_speed_mps=21.).requested_experimental

def test_model_gap_cannot_count_as_confirmation():
  t=Trace()
  for _ in range(39): t.step()
  t.now += .3
  assert not t.step().requested_experimental
  for _ in range(39): assert not t.step().requested_experimental
  assert t.step().requested_experimental

@pytest.mark.parametrize('condition,reason',[('stop_light_detected',Reason.CEM_STOP),('slow_lead_detected',Reason.CEM_LEAD),
  ('curve_detected',Reason.CEM_CURVE)])
def test_other_conditions_take_priority_during_entry_and_recovery(condition,reason):
  t=Trace(); settings=replace(SETTINGS,cem_curves=True)
  observed={condition:True}
  if condition=='curve_detected':observed['curve_raw']=True
  r=t.step(settings=settings,**observed)
  assert r.reason is reason and r.requested_experimental
  # Independent hold cannot be consumed by an Open Road release.
  for _ in range(8): assert t.step(settings=settings).requested_experimental

def test_independent_hold_survives_open_road_recovery_release():
  t=Trace();t.enter()
  for _ in range(39): t.step(speed=18.)
  assert t.step(speed=18.,stop_light_detected=True).reason is Reason.CEM_STOP
  for _ in range(8): assert t.step(speed=18.).requested_experimental
  assert not t.p.open_road.active

def test_gap_matches_current_mpc_semantics():
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_safe_obstacle_distance,get_stopped_equivalence_factor
  for speed in (0.,5.,20.,35.):
    for lead in (0.,10.,25.):
      for headway in (.5,1.5,3.):
        assert following_gap(speed,lead,headway) == pytest.approx(max(6.,speed*headway,get_safe_obstacle_distance(speed,headway)-get_stopped_equivalence_factor(lead)))

# Exercise real serialized current messages, clock conversion and installed
# lead tracker before the policy: no fabricated scene freshness in this test.
def test_serialized_projector_enters_tesla_only_and_rejects_stale_model():
  from openpilot.cereal import messaging
  from openpilot.starpilot.conditional_mode.projection import SceneProjector
  from openpilot.starpilot.conditional_mode.tests.test_projection import FakeSubMaster,serialized_scene,owner_context,MONO,BOOT
  for fingerprint in ('TESLA_MODEL_3','TESLA_MODEL_Y','HYUNDAI_IONIQ_6'):
    cp=NS(openpilotLongitudinalControl=True,pcmCruise=False,carFingerprint=fingerprint)
    projector=SceneProjector();policy=ConditionalModePolicy()
    sm=FakeSubMaster(serialized_scene(speed=20.,lead_present=False),MONO)
    projector.project(sm,cp,now_mono_ns=MONO,now_boot_ns=BOOT,sample_skew_ns=1000)
    saw=False
    for tick in range(1,65):
      stamp=MONO+tick*50_000_000;sm.stamp(stamp)
      sm.payloads.update(serialized_scene(stamp=BOOT+tick*50_000_000,speed=20.,lead_present=False))
      message=messaging.new_message('modelV2',valid=True);message.modelV2=sm.payloads['modelV2']
      message.modelV2.action.shouldStop=False;message.modelV2.action.desiredAcceleration=.1
      sm.payloads['modelV2']=messaging.log_from_bytes(message.to_bytes()).modelV2
      projection=projector.project(sm,cp,now_mono_ns=stamp+1_000_000,now_boot_ns=BOOT+tick*50_000_000+1_000_000,
        sample_skew_ns=1000,safe_mode=False,owner_context=owner_context(stamp),selected_t_follow_s=1.5,selected_t_follow_observed_mono_ns=stamp)
      assert projection.scene.tesla_open_road is fingerprint.startswith('TESLA')
      result=policy.step((stamp+1_000_000)/1e9,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,projection.scene,SETTINGS)
      saw |= result.reason is Reason.CEM_OPEN_ROAD
      if fingerprint.startswith('TESLA') and tick<40: assert result.reason is not Reason.CEM_OPEN_ROAD
    assert saw
    sm.valid['modelV2']=False
    projection=projector.project(sm,cp,now_mono_ns=stamp+2_000_000,now_boot_ns=BOOT+tick*50_000_000+2_000_000,
      sample_skew_ns=1000,safe_mode=False,owner_context=owner_context(stamp),selected_t_follow_s=1.5,selected_t_follow_observed_mono_ns=stamp)
    assert projection.scene.open_road_model_s is None
    assert projection.scene.open_road_model_accel is None


def test_latest_pre7_state_machine_oracle_on_8000_fresh_frames():
  import ast,math,random
  from pathlib import Path
  source=Path(__file__).with_name('fixtures')/'pre7_open_road.txt'
  tree=ast.parse(source.read_text())
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ConditionalExperimentalMode')
  methods={'reset_open_road','update_open_road','open_road_model_ready','open_road_stable_following'}
  cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in methods or
            isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id.startswith('OPEN_ROAD_') for t in n.targets)]
  now=100.
  env=dict(time=NS(monotonic=lambda:now),np=NS(isfinite=math.isfinite),CV=NS(MPH_TO_MS=MPH),V_CRUISE_UNSET=255.)
  exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(source),'exec'),env)
  legacy=env['ConditionalExperimentalMode']();legacy.reset_open_road()
  lead=NS(status=False,radar=False,modelProb=.95,dRel=40.,vLead=20.,aLeadK=0.,yRel=0.)
  follow=NS(following_lead=False,t_follow=1.5,desired_follow_distance=36.)
  legacy.starpilot_planner=NS(lead_one=lead,tracking_lead=False,starpilot_following=follow,lead_path_y=0.)
  class Messages(dict):
    def all_checks(self,service_list=None):return True
  sm=Messages(carState=NS(vCruise=72.),modelV2=NS(action=NS(desiredAcceleration=.1,shouldStop=False)))
  sm.logMonoTime={};toggle=NS(conditional_open_road=True);current=OpenRoad();rng=random.Random(21)
  phases=[(20.,False,False,20.,0.,.1)]*2
  choices=[(20.,False,False,20.,0.,.1),(19.,False,False,20.,0.,.1),
           (19.8,False,False,20.,0.,.1),(20.,True,False,15.,-.3,.1),
           (20.,True,True,20.,0.,.1),(20.,True,True,18.,-.3,-.3),
           (20.,False,False,20.,0.,-.3)]
  phases += [rng.choice(choices) for _ in range(98)]
  entries=releases=0
  for speed,present,tracked,lead_speed,brake,accel in phases:
    for _ in range(80):
      now=round(now+.05,6);sm.logMonoTime['modelV2']=int(now*1e9)
      lead.status=present;lead.vLead=lead_speed;lead.aLeadK=brake
      legacy.starpilot_planner.tracking_lead=tracked;follow.following_lead=tracked
      follow.desired_follow_distance=following_gap(speed,lead_speed,1.5)
      sm['modelV2'].action.desiredAcceleration=accel
      expected=legacy.update_open_road(speed,sm,toggle,20.,False)
      frame=scene(now,speed=speed,set_speed_mps=20.,lead=LeadEvidence(present,tracked,False,40.,lead_speed,brake,.95),
                  following_lead=tracked,open_road_observed_s=now,open_road_model_s=now,
                  open_road_model_accel=accel,open_road_model_stop=False,open_road_headway_s=1.5,open_road_lateral_m=0.)
      before=current.active
      assert current.step(now,frame) is expected,(now,frame)
      assert current.approach is legacy.open_road_approach
      entries+=not before and current.active;releases+=before and not current.active
  assert entries>=5 and releases>=5
