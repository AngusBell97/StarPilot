"""SP-021 settings consistency and discontinuity regressions."""
from dataclasses import replace, fields
from types import SimpleNamespace
import json
import pytest
from openpilot.cereal import messaging
from openpilot.starpilot.conditional_mode.policy import *
from openpilot.starpilot.conditional_mode.speed_lead import SpeedLeadSelector
from openpilot.starpilot.conditional_mode.preferences import CEMOptions, CCMOptions, SavedPreferences, encode_preferences
from openpilot.starpilot.conditional_mode.tests.test_policy import scene, AUTHORITY
from openpilot.starpilot.conditional_mode.tests.test_projection import MONO, BOOT, CP, serialized_scene, FakeSubMaster, owner_context
from openpilot.starpilot.conditional_mode.projection import SceneProjector
from openpilot.starpilot.conditional_mode.tests.test_manual_saved_join import ManualSavedJoinTests
from openpilot.starpilot.longitudinal.mode_recovery import ModeRecovery, RecoveryFrame
from openpilot.starpilot.ui.conditional_feature import OPTION_HELP, SIGNAL_LANE_HELP

MPH = 0.44704

@pytest.mark.parametrize('main,lead,present,speed,expected',[
 (0,20,True,10,True),(0,20,False,10,False),
 (20,10,True,15,False),(20,10,False,15,True),
 (10,20,True,15,True),(10,20,False,15,False),
 (20,10,None,5,True),(20,10,None,15,False),
 (0,20,None,5,False),(20,0,True,5,False),(0,0,False,0,False),
 (20,20,True,20,False),(20,20,False,19.99,True),
])
def test_separate_thresholds_are_never_ored(main,lead,present,speed,expected):
 p=ConditionalModePolicy();s=ModeSettings(cem_speed_mps=main*MPH,cem_speed_with_lead_mps=lead*MPH)
 result=p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,
   scene(10.,speed=speed*MPH,speed_lead_present=present,following_lead=None),s)
 assert result.requested_experimental is expected

@pytest.mark.parametrize('present',[False,True])
def test_speed_rule_survives_standstill_crawl_and_stop_release(present):
 p=ConditionalModePolicy();s=ModeSettings(cem_speed_mps=20*MPH,cem_speed_with_lead_mps=20*MPH)
 for n in range(160):
  now=10+n*.05;speed=(0.,.1,.5,1.,1.5)[n//8%5]
  frame=scene(now,speed=speed,standstill=speed==0,standstill_stop_hold=n<8,
              speed_lead_present=present,following_lead=None)
  result=p.step(now,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,frame,s)
  assert result.requested_experimental


def test_speed_release_band_and_setting_disable():
 p=ConditionalModePolicy();s=ModeSettings(cem_speed_mps=20*MPH)
 for n in range(100):
  now=10+n*.05
  speed=(19.9 if n%2==0 else 20.5)*MPH
  assert p.step(now,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(now,speed=speed),s).requested_experimental
 assert not p.step(15.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(15.,speed=5.),replace(s,cem_speed_mps=0.)).requested_experimental


def test_curve_chatter_never_reenters_chill_before_continuous_clear():
 p=ConditionalModePolicy();outputs=[]
 for n in range(160):
  now=10+n*.05
  # Enter Chill, then a bend containing repeated 0.4s apparent clear dips.
  bend=20<=n<100 and n%20<12
  result=p.step(now,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,
                scene(now,speed=25.,set_speed_mps=30.,curve_detected=bend))
  outputs.append(result.requested_experimental)
 assert outputs[10] is False
 assert all(outputs[20:139])
 assert outputs[-1] is False
 assert sum(a!=b for a,b in zip(outputs,outputs[1:]))==3
 # A fresh hazard bypasses every hold and re-entry timer immediately.
 now=18.
 assert p.step(now,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,scene(now,speed=25.,slow_lead_detected=True)).requested_experimental


def test_curve_unknown_and_raw_curve_restart_clearance():
 p=ConditionalModePolicy()
 for n in range(110):
  now=10+n*.05;detected=True if n==0 else None if n==30 else False
  raw=True if n==60 else detected
  result=p.step(now,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,
                scene(now,speed=25.,set_speed_mps=30.,curve_detected=detected,curve_raw=raw))
  if n<107:assert result.requested_experimental
 assert not result.requested_experimental


def test_curve_enabled_with_lead_does_not_require_following_geometry():
 p=ConditionalModePolicy()
 result=p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,
   scene(10.,curve_detected=True,following_lead=None,speed_lead_present=None),ModeSettings(cem_curves=True,cem_curves_with_lead=True))
 assert result.reason is Reason.CEM_CURVE


def test_launch_option_missing_launch_only_data_does_not_block_highway_chill():
 p=ConditionalModePolicy();s=ModeSettings(ccm_launch=True)
 for n in range(30):
  now=10+n*.05
  result=p.step(now,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,
   scene(now,speed=25.,set_speed_mps=30.,launch_candidate=None,launch_forced_exit=None,launch_lead=None),s)
 assert not result.requested_experimental


def test_disabling_launch_clears_chill_dwell():
 p=ConditionalModePolicy();s=ModeSettings(ccm_launch=True)
 assert not p.step(10.,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,scene(10.,speed=.5,launch_candidate=True),s).requested_experimental
 assert p.step(10.05,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,scene(10.05,speed=.5,launch_candidate=True),replace(s,ccm_launch=False)).requested_experimental


def test_disabled_stop_option_does_not_request_standstill_model_hold():
 p=ConditionalModePolicy()
 r=p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.,speed=0.,standstill=True,standstill_stop_hold=True),ModeSettings(cem_stop=False))
 assert not r.requested_experimental
 r=p.step(10.05,ModeChoice.CEM,ManualIntent.FORCE_CHILL,AUTHORITY,scene(10.05,speed=0.,standstill=True,committed_stop=True),ModeSettings(cem_stop=False))
 assert r.requested_experimental and r.reason is Reason.CEM_STOP


def lead(**changes):
 return SimpleNamespace(**(dict(present=True,radar=False,dRel=10.,vLead=1.,yRel=0.,modelProb=.999)|changes))


def test_primary_lead_absence_confirmation_and_unknown_freshness():
 c=SpeedLeadSelector()
 def sample(n,raw=None,**kw):
  return c.step(raw or lead(),observed_ns=10_000_000_000+n*50_000_000,now_ns=10_001_000_000+n*50_000_000,fresh=kw.get('fresh',True))
 assert sample(0) is True
 assert sample(1,lead(modelProb=.71)) is True
 for n in range(2,12):assert sample(n,lead(present=False)) is True
 assert sample(12,lead(present=False)) is False
 assert sample(13) is True
 assert sample(14,fresh=False) is None
 assert sample(15,lead(modelProb=.71)) is None
 assert sample(16,lead(dRel=float('nan'))) is None
 assert sample(17) is True
 assert sample(17) is True

@pytest.mark.parametrize('fault',['backstep','far_lateral','crawl'])
def test_real_projector_keeps_speed_lead_when_safety_tracker_cannot_follow(fault):
 projector=SceneProjector();sm=FakeSubMaster(serialized_scene(),MONO-5_000_000)
 projector.project(sm,CP,now_mono_ns=MONO,now_boot_ns=BOOT,sample_skew_ns=1000)
 policy=ConditionalModePolicy()
 for n in range(1,25):
  stamp=MONO+n*50_000_000;sm.stamp(stamp)
  sm.payloads.update(serialized_scene(stamp=BOOT+n*50_000_000,speed=1. if fault=='crawl' else 3.,lead_distance=10.,lead_speed=1.))
  event=messaging.new_message('modelV2',valid=True);event.modelV2=sm['modelV2']
  if fault=='backstep':event.modelV2.position.x[24]=float(event.modelV2.position.x[23])-.035
  if fault=='far_lateral':event.modelV2.position.y[32]=-50.09
  sm.payloads['modelV2']=messaging.log_from_bytes(event.to_bytes()).modelV2
  r=projector.project(sm,CP,now_mono_ns=stamp+1_000_000,now_boot_ns=BOOT+n*50_000_000+1_000_000,
     sample_skew_ns=1000,safe_mode=False,selected_t_follow_s=1.45,selected_t_follow_observed_mono_ns=stamp,
     owner_context=owner_context(stamp))
  assert r.scene.speed_lead_present is True
  assert r.scene.following_lead is not True
  assert policy.step((stamp+1_000_000)/1e9,ModeChoice.CEM,ManualIntent.NONE,r.authority,r.scene,
    ModeSettings(cem_speed_with_lead_mps=20*MPH,cem_stop=False)).requested_experimental


def test_all_saved_option_help_is_explicit():
 for kind,cls in [('cem',CEMOptions),('ccm',CCMOptions)]:
  assert {f.name for f in fields(cls)}-{'persist_manual'} <= set(OPTION_HELP[kind])|set(SIGNAL_LANE_HELP)


def test_both_switch_directions_and_protective_frame_keep_upward_bound():
 for first in (False,True):
  g=ModeRecovery()
  def step(n,mode,previous,target,protective=False):
   f=RecoveryFrame(10_000_000_000+n*50_000_000,9_999_000_000+n*50_000_000,1,mode)
   return g.step(f,previous_target=previous,target=target,positive_jerk=1.,protective=protective)
  step(0,first,0.,0.)
  assert step(1,not first,0.,1.) == pytest.approx(.05)
  assert step(2,not first,.05,-2.,True)==-2.
  assert g.armed
  assert step(3,not first,-2.,1.) == pytest.approx(-1.8)
  assert step(4,first,-1.8,-3.)==-3.


class TestManualSettingsEdit(ManualSavedJoinTests):
 def test_volatile_manual_choice_survives_threshold_edit(self):
  def save(speed,persist=False):
   self.params.put('ConditionalModeConfig',json.loads(encode_preferences(SavedPreferences(mode=ModeChoice.CEM,
     cem=CEMOptions(speed_mps=speed,persist_manual=persist)))),block=True)
  save(20.)
  self.sample(MONO)
  stamp=MONO+50_000_000
  self.sample(stamp,self.receipt(stamp,1))
  assert self.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL
  save(10.)
  self.owner.settings.refresh(stamp+50_000_000,force=True)
  self.sample(stamp+50_000_000)
  assert self.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL
  assert self.owner.manual.code == 2 and self.owner.manual_live_code == 2
  self.sample(stamp+100_000_000,self.receipt(stamp+100_000_000,2))
  assert self.owner.manual_state.intent is ManualIntent.NONE
  assert self.owner.manual.code == 0
  save(10.,True)
  self.owner.settings.refresh(stamp+150_000_000,force=True)
  self.sample(stamp+150_000_000)
  assert self.owner.manual_state.intent is ManualIntent.NONE
def test_ccm_missing_proposal_does_not_authorize_chill_with_fresh_sources():
 from openpilot.starpilot.conditional_mode.consumer import ModeConsumer
 c=ModeConsumer();now=100_000_000_000
 kwargs=dict(now_ns=now,now_boot_ns=now+2_000_000_000,sample_skew_ns=1000,message_ns=now-1_000_000,
   receipt_ns=now-1_000_000,drive_id=90_000_000_000,model_ns=now-1_000_000,car_state_ns=now-1_000_000,
   authority=True,stock_experimental=False,choice=ModeChoice.CCM,settings_fingerprint='a'*64)
 c.sample(None,**kwargs)
 for k in ('now_ns','now_boot_ns','message_ns','receipt_ns','model_ns','car_state_ns'):kwargs[k]+=50_000_000
 r=c.sample(None,**kwargs)
 assert not r.accepted and r.experimental
 r=c.sample(None,**(kwargs|{'authority':False}))
 assert not r.accepted and not r.experimental


def test_ccm_detector_options_are_independent_of_saved_cem_options(tmp_path):
 from unittest.mock import patch
 from openpilot.common.params import Params
 from openpilot.starpilot.conditional_mode.host import ConditionalModeHost
 from openpilot.starpilot.conditional_mode.runtime_settings import ConditionalSettingsOwner
 from openpilot.starpilot.conditional_mode.preferences import manual_for_drive
 params=Params(str(tmp_path/'params'))
 saved=SavedPreferences(mode=ModeChoice.CCM,cem=CEMOptions(lead=False,slower_lead=False,stopped_lead=False,stop_lights=False,model_stop_s=0.))
 params.put('ConditionalModeConfig',json.loads(encode_preferences(saved)),block=True)
 settings=ConditionalSettingsOwner(params);host=ConditionalModeHost()
 sm=FakeSubMaster(serialized_scene(),MONO-5_000_000)
 with patch.object(SceneProjector,'project',autospec=True,side_effect=SceneProjector.project) as project:
  host.sample(sm,CP,now_mono_ns=MONO,now_boot_ns=BOOT,sample_skew_ns=1000,drive_id=MONO,
    settings_owner=settings,manual=manual_for_drive(ManualIntent.NONE,MONO,MONO))
  options=project.call_args.kwargs['signal_options']
 assert options.lead and options.slower_lead and options.stopped_lead and options.stop_lights
 assert options.model_stop_s==7.7


def test_zero_lane_width_does_not_disable_signal_trigger():
 from openpilot.starpilot.conditional_mode.signal_lane import SignalLaneTracker
 from openpilot.starpilot.conditional_mode.tests.test_signal_lane import frame
 r=SignalLaneTracker().update(replace(frame(speed=4.9),lane_detection_width_m=0.))
 assert r.signal_scene is True and r.lane_available is False


def test_protective_upward_step_still_bounded():
 g=ModeRecovery()
 for n in range(3):
  r=g.step(RecoveryFrame(10_000_000_000+n*50_000_000,10_000_000_000+n*50_000_000,1,n>0),
    previous_target=-1.,target=1.,positive_jerk=1.,protective=True)
  if n: assert r == pytest.approx(-.8) and g.armed


def test_absence_after_unknown_and_cold_start_requires_confirmation():
 for initial in (False,True):
  c=SpeedLeadSelector()
  if initial:
   assert c.step(lead(),observed_ns=9_950_000_000,now_ns=9_950_000_000,fresh=True) is True
   assert c.step(lead(dRel=float('nan')),observed_ns=10_000_000_000,now_ns=10_000_000_000,fresh=True) is None
  for n in range(11):
   t=10_050_000_000+n*50_000_000
   r=c.step(lead(present=False),observed_ns=t,now_ns=t,fresh=True)
   assert r is (False if n==10 else None)


def test_standstill_unknown_speed_hold_is_bounded_not_renewed():
 p=ConditionalModePolicy();s=ModeSettings(cem_speed_with_lead_mps=20*MPH,cem_stop=False)
 for n in range(15):
  t=10+n*.05
  r=p.step(t,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(t,speed=0.,standstill=True,
    speed_lead_present=True if n==0 else None),s)
  assert r.requested_experimental is (n<10)


def test_stale_scene_cannot_complete_curve_clearance():
 p=ConditionalModePolicy()
 for n in range(100):
  t=10+n*.05
  f=scene(t,speed=25.,set_speed_mps=30.,curve_detected=n==0,curve_raw=n==0)
  if 11<=n<41:f=replace(f,observed_mono_s=t-1.)
  r=p.step(t,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,f)
  if n<87:assert r.requested_experimental
 assert not r.requested_experimental


def test_manual_curve_history_survives_return_to_automatic():
 p=ConditionalModePolicy()
 for n in range(70):
  t=10+n*.05
  r=p.step(t,ModeChoice.CCM,ManualIntent.FORCE_EXPERIMENTAL if n<10 else ManualIntent.NONE,AUTHORITY,
    scene(t,speed=25.,set_speed_mps=30.,curve_detected=n==0,curve_raw=n<20))
  if n<67:assert r.requested_experimental
 assert not r.requested_experimental


def test_inactive_mode_edit_keeps_cem_history():
 p=ConditionalModePolicy();s=ModeSettings(cem_speed_mps=20*MPH)
 p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.,speed=5.),s)
 road=p.open_road
 r=p.step(10.05,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.05,speed=15.),replace(s,ccm_speed_mps=30.))
 assert p.open_road is road and r.requested_experimental

@pytest.fixture
def joined_owner():
 f=ManualSavedJoinTests();f.setUp()
 try:yield f
 finally:f.doCleanups()


def test_settings_edit_retention_survives_busy_lease(joined_owner):
 from unittest.mock import patch
 from openpilot.starpilot.conditional_mode import manual_saved
 f=joined_owner
 def save(speed):
  f.params.put('ConditionalModeConfig',json.loads(encode_preferences(SavedPreferences(mode=ModeChoice.CEM,cem=CEMOptions(speed_mps=speed)))),block=True)
 save(20.);f.sample(MONO);f.sample(MONO+50_000_000,f.receipt(MONO+50_000_000,1))
 assert f.owner.manual.code==2
 save(10.);f.owner.settings.refresh(MONO+100_000_000,force=True)
 with patch.object(manual_saved,'_lease',return_value=None):f.sample(MONO+100_000_000)
 assert f.owner.retained_manual is not None
 assert f.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL
 f.sample(MONO+150_000_000)
 assert f.owner.manual.code==2 and f.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL
 f.sample(MONO+200_000_000,f.receipt(MONO+200_000_000,2))
 assert f.owner.manual_state.intent is ManualIntent.NONE


def test_ccm_threshold_edit_does_not_reset_scene_detectors(joined_owner):
 from unittest.mock import patch
 f=joined_owner
 def save(speed):
  f.params.put('ConditionalModeConfig',json.loads(encode_preferences(SavedPreferences(mode=ModeChoice.CCM,ccm=CCMOptions(speed_mps=speed)))),block=True)
 save(20.)
 for n in range(25):f.sample(MONO+n*50_000_000)
 projector=f.owner.mode.projector
 with patch.object(projector.stop_detector,'reset',wraps=projector.stop_detector.reset) as stop, patch.object(projector.slower_lead_detector,'reset',wraps=projector.slower_lead_detector.reset) as slow:
  save(10.);f.owner.settings.refresh(MONO+1_250_000_000,force=True);f.sample(MONO+1_250_000_000)
  assert stop.call_count==0 and slow.call_count==0


def test_ui_tap_after_threshold_edit_returns_to_automatic():
 from openpilot.starpilot.conditional_mode.tests.test_ui_action import TestUiAction
 f=TestUiAction();f.setUp()
 try:
  t=MONO+50_000_000
  f.sample(t,card_event=f.card(t,1));assert f.owner.manual.code==2
  saved=SavedPreferences(mode=ModeChoice.CEM,cem=CEMOptions(speed_mps=10.))
  f.params.put('ConditionalModeConfig',json.loads(encode_preferences(saved)),block=True)
  f.owner.settings.refresh(t+50_000_000,force=True);f.ui.settings.last_refresh_ns=None
  f.sample(t+50_000_000,effective=True);assert f.owner.manual.code==2
  event=f.request(t+60_000_000,effective=True,code=2)
  f.sample(t+70_000_000,ui_event=event,effective=True)
  assert f.owner.manual.code==0 and f.owner.manual_state.intent is ManualIntent.NONE
 finally:f.doCleanups()


def test_committed_stop_does_not_skip_bend_observations():
 p=ConditionalModePolicy()
 for n in range(130):
  now=10+n*.05
  bend=n==0 or 11<=n<71
  r=p.step(now,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,scene(now,speed=25.,set_speed_mps=30.,
    curve_detected=bend,curve_raw=bend,committed_stop=11<=n<71))
  if n<117:assert r.requested_experimental
 assert not r.requested_experimental


@pytest.mark.parametrize('choice',[ModeChoice.CEM,ModeChoice.CCM])
def test_old_bend_only_withholds_ccm_chill_after_interruption(choice):
 p=ConditionalModePolicy();s=ModeSettings(cem_curves=True,cem_curves_with_lead=True)
 p.step(10.,choice,ManualIntent.NONE,AUTHORITY,scene(10.,speed=25.,curve_detected=True,curve_raw=True),s)
 r=p.step(20.,choice,ManualIntent.NONE,AUTHORITY,scene(20.,speed=25.,curve_detected=False,curve_raw=False),s)
 assert r.requested_experimental is (choice is ModeChoice.CCM)
 assert r.reason is not Reason.CEM_CURVE


@pytest.mark.parametrize('committed',[False,True])
def test_speed_hold_survives_stop_release_with_unknown_lead(committed):
 p=ConditionalModePolicy();s=ModeSettings(cem_speed_with_lead_mps=20*MPH)
 p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.,speed=0.,standstill=True,
   standstill_stop_hold=True,committed_stop=committed,speed_lead_present=True),s)
 r=p.step(10.05,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.05,speed=0.,standstill=True,
   standstill_stop_hold=False,committed_stop=False,speed_lead_present=None),s)
 assert r.requested_experimental


@pytest.mark.parametrize('persist',[False,True])
def test_repeated_threshold_edits_during_busy_owner_keep_applied_intent(joined_owner,persist):
 from unittest.mock import patch
 from openpilot.starpilot.conditional_mode import manual_saved
 f=joined_owner
 def save(speed):
  f.params.put('ConditionalModeConfig',json.loads(encode_preferences(SavedPreferences(mode=ModeChoice.CEM,
    cem=CEMOptions(speed_mps=speed,persist_manual=persist)))),block=True)
 save(20.);f.sample(MONO);f.sample(MONO+50_000_000,f.receipt(MONO+50_000_000,1))
 if persist:f.settle(MONO+75_000_000)
 assert f.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL
 with patch.object(manual_saved,'_lease',return_value=None):
  for n in (2,3,4):
   stamp=MONO+n*50_000_000;save(20.-n);f.owner.settings.refresh(stamp,force=True)
   proposal,_=f.sample(stamp)
   assert f.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL
   assert proposal.decision.reason is Reason.MANUAL_EXPERIMENTAL
   if persist:assert not f.owner.manual.ready
 f.sample(MONO+250_000_000)
 assert f.owner.manual_state.intent is ManualIntent.FORCE_EXPERIMENTAL and f.owner.manual.code==2
 f.sample(MONO+300_000_000,f.receipt(MONO+300_000_000,2))
 if persist:f.settle(MONO+325_000_000)
 assert f.owner.manual_state.intent is ManualIntent.NONE


def test_active_settings_edit_preserves_fresh_bend_latch():
 p=ConditionalModePolicy();s=ModeSettings(cem_curves=True,cem_curves_with_lead=True)
 p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.,speed=25.,curve_detected=True,curve_raw=True),s)
 r=p.step(10.05,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.05,speed=25.,curve_detected=False,curve_raw=True),
   replace(s,cem_speed_mps=10.))
 assert r.requested_experimental and r.reason is Reason.CEM_CURVE
