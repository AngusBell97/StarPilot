"""SP-021 R2: optional CCM Chill permission in a bend, not an acceleration override."""
from dataclasses import replace
import json
import pytest
from openpilot.starpilot.conditional_mode.policy import ConditionalModePolicy, ModeChoice, ModeSettings, ManualIntent, Reason, LeadEvidence
from openpilot.starpilot.conditional_mode.preferences import SavedPreferences, CCMOptions, PreferenceError, encode_preferences, decode_preferences
from openpilot.starpilot.conditional_mode.actions import edit
from openpilot.starpilot.conditional_mode.tests.test_policy import scene, AUTHORITY


def run(policy, n, settings, **kw):
 now=10+n*.05
 return policy.step(now,ModeChoice.CCM,ManualIntent.NONE,AUTHORITY,scene(now,speed=25.,set_speed_mps=30.,**kw),settings)


@pytest.mark.parametrize('enabled',[False,True])
def test_known_bend_permission_and_chatter(enabled):
 p=ConditionalModePolicy();s=ModeSettings(ccm_allow_chill_in_bends=enabled)
 out=[run(p,n,s,curve_detected=n%20<12,curve_raw=True).requested_experimental for n in range(100)]
 assert all(out) if not enabled else not any(out[8:])
 assert sum(a!=b for a,b in zip(out,out[1:]))==(1 if enabled else 0)


@pytest.mark.parametrize('flag',['left_blinker','right_blinker','traffic_mode','slow_lead_detected',
 'stop_light_detected','stop_sign_confirmed','forcing_stop','adjacent_lead_ambiguous','low_speed_stop_scene','standstill','committed_stop'])
def test_bend_permission_never_overrides_protective_conditions(flag):
 p=ConditionalModePolicy();s=ModeSettings(ccm_allow_chill_in_bends=True)
 for n in range(12):r=run(p,n,s,curve_detected=True)
 assert not r.requested_experimental
 assert run(p,12,s,curve_detected=True,**{flag:True}).requested_experimental


@pytest.mark.parametrize('flag',['curve_detected','left_blinker','slow_lead_detected','stop_light_detected','traffic_mode'])
def test_bend_permission_never_turns_unknown_into_clear(flag):
 p=ConditionalModePolicy();s=ModeSettings(ccm_allow_chill_in_bends=True)
 for n in range(12):run(p,n,s,curve_detected=True)
 assert run(p,12,s,**{flag:None}).requested_experimental


def test_permission_does_not_create_candidate_and_off_reinstates_curve_veto():
 p=ConditionalModePolicy();s=ModeSettings(ccm_allow_chill_in_bends=True,ccm_speed_mps=30.)
 for n in range(20):assert run(p,n,s,curve_detected=True).requested_experimental
 s=replace(s,ccm_speed_mps=0.)
 for n in range(20,32):r=run(p,n,s,curve_detected=True)
 assert not r.requested_experimental
 assert run(p,32,replace(s,ccm_allow_chill_in_bends=False),curve_detected=False,curve_raw=True).requested_experimental


def test_stable_lead_candidate_can_use_bend_permission():
 p=ConditionalModePolicy();s=ModeSettings(ccm_allow_chill_in_bends=True)
 lead=LeadEvidence(True,True,False,30.,25.,0.,.99)
 for n in range(30):r=run(p,n,s,curve_detected=True,lead=lead,following_lead=True)
 assert not r.requested_experimental and r.reason is Reason.CCM_LEAD


def test_cem_is_unchanged_by_ccm_permission():
 for enabled in (False,True):
  p=ConditionalModePolicy();s=ModeSettings(cem_curves=True,cem_curves_with_lead=True,ccm_allow_chill_in_bends=enabled)
  assert p.step(10.,ModeChoice.CEM,ManualIntent.NONE,AUTHORITY,scene(10.,curve_detected=True),s).reason is Reason.CEM_CURVE


def test_old_documents_roundtrip_without_new_field_and_on_off_edits():
 old=encode_preferences(SavedPreferences(mode=ModeChoice.CCM))
 assert b'allow_chill_in_bends' not in old
 assert not decode_preferences(old).ccm.allow_chill_in_bends
 on=edit(decode_preferences(old),'conditional:ccm:allow_chill_in_bends','On',metric=False)
 assert decode_preferences(on).mode_settings().ccm_allow_chill_in_bends
 off=edit(decode_preferences(on),'conditional:ccm:allow_chill_in_bends','Off',metric=False)
 assert off==old
 explicit=json.loads(old);explicit['ccm']['allow_chill_in_bends']=False
 assert encode_preferences(decode_preferences(json.dumps(explicit)))==old


@pytest.mark.parametrize('bad',[None,0,1,'true',[],{}])
def test_extension_is_strict_boolean(bad):
 d=json.loads(encode_preferences(SavedPreferences()));d['ccm']['allow_chill_in_bends']=bad
 with pytest.raises(PreferenceError):decode_preferences(json.dumps(d))


def test_shared_native_galaxy_row_and_guarded_commit():
 from pathlib import Path
 from openpilot.starpilot.ui.tests.test_conditional_feature import ConditionalFeatureTests, required_change
 from openpilot.starpilot.ui.feature_settings_state import FeaturePage
 f=ConditionalFeatureTests();f.setUp()
 try:
  key='conditional:ccm:allow_chill_in_bends';page=FeaturePage.CONDITIONAL_CCM
  row=f.row(key,page);assert row.label=='Allow Chill in bends' and row.value=='Off' and row.choices==('Off','On')
  assert 'CSC' in row.reason
  assert f.owner.apply(required_change(row))
  assert f.row(key,page).value=='On'
  assert decode_preferences(Path(f.params.get_param_path('ConditionalModeConfig')).read_bytes()).ccm.allow_chill_in_bends
  request=required_change(f.row(key,page))
  f.parked=False
  assert not f.owner.apply(request)
 finally:f.doCleanups()


@pytest.mark.parametrize('ceiling',[17.,24.])
def test_chill_bend_keeps_actual_csc_ceiling_and_native_planner(ceiling):
 from openpilot.starpilot.curve_speed.tests.test_planner import Inputs
 from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages
 from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
 from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
 from opendbc.car.honda.interface import CarInterface
 from opendbc.car.honda.values import CAR
 p=ConditionalModePolicy();s=ModeSettings(ccm_allow_chill_in_bends=True)
 for n in range(12):r=run(p,n,s,curve_detected=True)
 assert not r.requested_experimental
 cp=CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
 actual=LongitudinalPlanner(cp,init_v=20.)
 for n in range(40):
  stamp=1_000_000_000+n*50_000_000;data,_=messages(e2e=r.requested_experimental)
  data['carControl'].longActive=True;data['modelV2'].action.desiredAcceleration=-1.
  sm=Inputs(data,stamp)
  actual.update(sm,curve_ceiling=CurveCeiling(ceiling,stamp))
  assert actual.mpc.solution_status==0
  assert actual.last_curve_ceiling_status=='selected'
 assert actual.last_curve_ceiling_applied
 if ceiling>20.:assert actual.output_a_target>0.
 else:assert actual.output_a_target<0.
