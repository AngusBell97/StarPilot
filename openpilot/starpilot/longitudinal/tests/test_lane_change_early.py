"""SP-040 r3: observed startup failures and native-authority regression boundaries."""
import json
from dataclasses import replace
import pytest
from openpilot.starpilot.longitudinal.tests.test_overtake_preparation import frame, POLICY
from openpilot.starpilot.longitudinal.lane_change_gap import LaneChangeGap, LaneChangePace, GapLead, continuous_leads


def warm(owner, policy=POLICY, leads=(GapLead(40., 0., 0.),)):
  for i in range(15):
    f = frame(i, state='off', signal_direction='none', direction='none', leads=leads)
    if isinstance(owner, LaneChangeGap):
      assert owner.step(policy, f, base_follow=1.45, long_active=True) is None
      assert owner.limit_acceleration(.7) == .7
    else:
      assert owner.duration(policy, f, long_active=True) == policy.duration_s


@pytest.mark.parametrize('pre_frames', [0, 1, 4])
def test_short_and_zero_delay_have_prequalified_history(pre_frames):
  g = LaneChangeGap(); warm(g)
  for i in range(15, 15 + pre_frames):
    assert g.step(POLICY, frame(i, leads=(GapLead(40., 0., 0.),)), base_follow=1.45, long_active=True) == .75
    assert g.limit_acceleration(.8) == .7  # retain the pre-signal native acceleration
  assert g.step(POLICY, frame(15 + pre_frames, state='laneChangeStarting'), base_follow=1.45, long_active=True) is None
  assert g.limit_acceleration(.8) > 0.
  assert g.reason == 'normal_lead_protection'


def test_indicator_model_mismatch_recovers_without_expiry():
  g = LaneChangeGap(); warm(g)
  assert g.step(POLICY, frame(15, state='off', direction='none', leads=(GapLead(40., 0., 0.),)), base_follow=1.45, long_active=True) is None
  assert not g.expired
  assert g.step(POLICY, frame(16, leads=(GapLead(40., 0., 0.),)), base_follow=1.45, long_active=True) == .75


@pytest.mark.parametrize('signal', ['left', 'none'])
def test_no_new_moving_wait_or_signal_off_acceleration_step(signal):
  g = LaneChangeGap(); warm(g)
  g.step(POLICY, frame(15), base_follow=1.45, long_active=True); g.limit_acceleration(1.)
  prev = .7
  for i in range(16, 40):
    g.step(POLICY, frame(i, state='laneChangeStarting', signal_direction=signal if i > 16 else 'left'), base_follow=1.45, long_active=True)
    out = g.limit_acceleration(1.)
    assert out >= prev
    assert out - prev <= .030000001
    prev = out


@pytest.mark.parametrize('leads', [(), (GapLead(12., -8., -2.),), (GapLead(60., 0., 0.), GapLead(25., -2., 0.))])
def test_native_braking_and_normal_following_protect_changed_destination(leads):
  g = LaneChangeGap(); warm(g)
  g.step(POLICY, frame(15), base_follow=1.45, long_active=True); g.limit_acceleration(0.)
  assert g.step(POLICY, frame(16, state='laneChangeStarting', leads=leads), base_follow=1.45, long_active=True) is None
  # This feature cannot replace or weaken the native negative request.
  assert g.limit_acceleration(-3.) == -3.
  assert g.ramped_follow is None


def test_slot_swaps_and_duplicate_hypotheses_preserve_history():
  a, b = GapLead(40., 0., 0., -.2), GapLead(60., 0., 0., 3.)
  assert continuous_leads((a,b), (b,a), .05)
  assert continuous_leads((a,a), (a,), .05)
  assert not continuous_leads((a,b), (a,), .05)
  assert not continuous_leads((a,), (replace(a,lateral_m=3.),), .05)


@pytest.mark.parametrize('pre_frames', [0, 4])
def test_speed10_uses_pre_signal_history(pre_frames):
  policy = replace(POLICY, close_gap_fast_pace=True)
  p = LaneChangePace(); leads = (GapLead(33.,0.,0.),); warm(p,policy,leads)
  for i in range(15,15+pre_frames):
    assert p.duration(policy,frame(i,leads=leads),long_active=True)==policy.duration_s
  assert p.duration(policy,frame(15+pre_frames,state='laneChangeStarting',leads=leads),long_active=True)==3.
  assert policy.duration_s != 3.


def test_speed10_no_late_selection_after_unqualified_entry():
  policy = replace(POLICY,close_gap_fast_pace=True);p=LaneChangePace()
  for i in range(40):
    assert p.duration(policy,frame(i,state='laneChangeStarting',leads=(GapLead(33.,0.,0.),)),long_active=True)==policy.duration_s


@pytest.mark.parametrize('change',[dict(left_blindspot=True),dict(gas_pressed=True),dict(brake_pressed=True),dict(force_decel=True),dict(model_should_stop=True)])
def test_inhibitors_revoke_preparation_and_pace(change):
  g=LaneChangeGap();warm(g);p=LaneChangePace();policy=replace(POLICY,close_gap_fast_pace=True);warm(p,policy,(GapLead(33.,0.,0.),))
  assert g.step(POLICY,frame(15,**change),base_follow=1.45,long_active=True) is None
  assert p.duration(policy,frame(15,state='laneChangeStarting',**change),long_active=True)==policy.duration_s


def test_timeout_cannot_rearm_but_cap_release_is_bounded():
  g=LaneChangeGap();warm(g)
  previous=0.
  for i in range(15,120):
    follow=g.step(POLICY,frame(i,leads=(GapLead(40.,0.,0.),)),base_follow=1.45,long_active=True)
    out=g.limit_acceleration(1.)
    if i>=95:
      assert follow is None and g.expired
      assert out-previous<=.030000001
    previous=out


def test_diagnostic_is_json_safe_and_bounded():
  g=LaneChangeGap();warm(g);messages=[]
  for i in range(15,35):
    g.step(POLICY,frame(i,leads=(GapLead(40.,0.,0.),)),base_follow=1.45,long_active=True)
    g.limit_acceleration(1.)
    event=g.diagnostic(before=1.,after=0.)
    if event is not None: messages.append(event);json.dumps(event,allow_nan=False)
  assert 3 <= len(messages) <= 5
  assert all(x['reason']=='qualified' for x in messages)


def test_disabled_is_bitwise_neutral_even_after_preparation():
  g=LaneChangeGap();warm(g);g.step(POLICY,frame(15),base_follow=1.45,long_active=True)
  g.step(replace(POLICY,close_gap=False),frame(16),base_follow=1.45,long_active=True)
  assert g.limit_acceleration(1.2345)==1.2345


def test_duplicate_frames_never_build_history():
  g=LaneChangeGap()
  for _ in range(30):
    assert g.step(POLICY,frame(0),base_follow=1.45,long_active=True) is None
  assert g.stable_seconds == 0.


def test_selected_pace_survives_non_threatening_handover_but_not_danger():
  policy=replace(POLICY,close_gap_fast_pace=True);p=LaneChangePace();leads=(GapLead(33.,0.,0.),)
  warm(p,policy,leads)
  assert p.duration(policy,frame(15,state='laneChangeStarting',leads=leads),long_active=True)==3.
  assert p.duration(policy,frame(16,state='laneChangeStarting',leads=(GapLead(80.,0.,0.),)),long_active=True)==3.
  assert p.duration(policy,frame(17,state='laneChangeStarting',leads=(GapLead(10.,-6.,0.),)),long_active=True)==policy.duration_s
  assert p.duration(policy,frame(30,state='laneChangeStarting',leads=leads),long_active=True)==policy.duration_s


def test_invalid_input_diagnostic_retains_denial_reason():
  g=LaneChangeGap();warm(g);g.step(POLICY,frame(15),base_follow=1.45,long_active=True)
  g.step(POLICY,None,base_follow=1.45,long_active=True)
  event=g.diagnostic(now_ns=20_000_000_000)
  assert event['reason']=='invalid_or_inhibited' and event['model_ns'] is None


def test_cancel_zero_request_does_not_discard_release_in_one_frame():
  g=LaneChangeGap();warm(g);g.step(POLICY,frame(15),base_follow=1.45,long_active=True);g.limit_acceleration(1.)
  g.step(POLICY,frame(16,state='off',signal_direction='none',direction='none'),base_follow=1.45,long_active=True)
  assert g.limit_acceleration(0.)==0. and g.acceleration_ceiling is not None
  g.step(POLICY,frame(17,state='off',signal_direction='none',direction='none'),base_follow=1.45,long_active=True)
  assert 0.<g.limit_acceleration(1.)<=.030000001


def test_selected_pace_revokes_on_clock_discontinuity():
  policy=replace(POLICY,close_gap_fast_pace=True);p=LaneChangePace();leads=(GapLead(33.,0.,0.),)
  warm(p,policy,leads)
  assert p.duration(policy,frame(15,state='laneChangeStarting',leads=leads),long_active=True)==3.
  assert p.duration(policy,frame(30,state='laneChangeStarting',leads=leads),long_active=True)==policy.duration_s
