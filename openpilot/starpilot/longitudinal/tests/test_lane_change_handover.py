"""SP-040 hand-over: a completed manoeuvre with the indicator still on never clamps the request to zero, and a
lead-margin withdrawal of a raised response ramps down instead of stepping (3 Nov 2026 drives)."""
import pytest
from openpilot.starpilot.longitudinal.lane_change_gap import GapLead
from openpilot.starpilot.longitudinal.tests.test_lane_change_response import warmed, cycle, frame, POLICY, LEADS

DT = 0.05
FAR_MARGIN = (GapLead(28., -2.5, 0., 0.),)   # room for 1 s at zero acceleration, none at 2.5 s (20 m/s, 0.75 s gap)
IMMINENT = (GapLead(12., -8., 0., 0.),)


def raised(g, cycles=20):
  """Move for a second with a raised response so the release envelope has climbed to about +0.6."""
  prev = 0.
  for i in range(15, 15 + cycles):
    prev = cycle(g, i, moving=True, previous=prev)
  assert g.response_raised and g.phase == 'moving' and prev > .5
  return prev


def test_completed_manoeuvre_with_lingering_indicator_releases_from_last_output():
  g = warmed()
  prev = raised(g)
  # The model has finished (state off) but the stalk is still on for this cycle: before the fix this was
  # phase pending with a zero ceiling, and limit_acceleration() cut a +0.6 request to 0.
  g.step(POLICY, frame(35, state='off', signal_direction='left', direction='none', leads=LEADS), base_follow=1.5, long_active=True)
  assert g.phase == 'pending' and g.reason == 'manoeuvre_complete'
  value = g.limit_acceleration(.9)
  assert value == pytest.approx(prev + .6 * DT, abs=1e-6)
  # The indicator goes off: normal settling from where we are, no restart from zero.
  g.step(POLICY, frame(36, state='off', signal_direction='none', direction='none', leads=LEADS), base_follow=1.5, long_active=True)
  assert g.phase == 'settling' and g.reason == 'signal_released'
  assert g.limit_acceleration(.9) == pytest.approx(value + .6 * DT, abs=1e-6)


def test_lingering_indicator_holds_for_several_cycles_without_dropping():
  g = warmed()
  prev = raised(g)
  for i in range(35, 41):
    g.step(POLICY, frame(i, state='off', signal_direction='left', direction='none', leads=LEADS), base_follow=1.5, long_active=True)
    value = g.limit_acceleration(.9)
    assert value > prev - 1e-9 and g.reason == 'manoeuvre_complete'
    prev = value
  assert prev > .6


def test_native_below_the_envelope_passes_through_at_the_hand_over():
  g = warmed()
  raised(g)
  g.step(POLICY, frame(35, state='off', signal_direction='left', direction='none', leads=LEADS), base_follow=1.5, long_active=True)
  assert g.limit_acceleration(.2) == .2
  assert g.limit_acceleration(-.4) == -.4


def test_lost_intent_while_preparing_still_clamps_to_zero():
  # Indicator on, model direction lost before anything started: unchanged (no permission while uncertain).
  g = warmed()
  cycle(g, 15, previous=-.6)
  assert g.phase == 'preparing'
  g.step(POLICY, frame(16, state='preLaneChange', direction='none', leads=LEADS), base_follow=1.5, long_active=True)
  assert g.phase == 'pending' and not g.maneuver_started
  assert g.limit_acceleration(.5) == 0.


def test_blindspot_during_the_manoeuvre_still_clamps_to_zero():
  g = warmed()
  raised(g)
  g.step(POLICY, frame(35, state='laneChangeStarting', left_blindspot=True, leads=LEADS), base_follow=1.5, long_active=True)
  assert g.phase == 'pending'
  assert g.limit_acceleration(.5) == 0.


def test_lead_margin_loss_ramps_a_raised_request_down_to_native():
  g = warmed()
  prev = raised(g)
  value = cycle(g, 35, moving=True, native=-.3, previous=prev, leads=FAR_MARGIN)
  assert value == pytest.approx(prev - 2. * DT, abs=1e-6)
  assert g.response_reason == 'lead_margin_release' and g.response_raised and not g.response_aborted
  prev = value
  for i in range(36, 60):
    value = cycle(g, i, moving=True, native=-.3, previous=prev, leads=FAR_MARGIN)
    if value > -.3 + 1e-9:
      assert value == pytest.approx(prev - 2. * DT, abs=1e-6) and g.response_reason == 'lead_margin_release'
    else:
      assert value == -.3 and g.response_reason == 'lead_margin' and not g.response_raised
      break
    prev = value
  else:
    pytest.fail('ramp never reached native')
  # Once at native the withdrawal latches as before: no late re-raise in this manoeuvre.
  assert cycle(g, 61, moving=True, native=-.3, previous=-.3, leads=FAR_MARGIN) == -.3
  assert g.response_reason == 'blocked'


def test_imminent_lead_still_withdraws_at_once():
  g = warmed()
  prev = raised(g)
  assert cycle(g, 35, moving=True, native=-.7, previous=prev, leads=IMMINENT) == -.7
  assert g.response_reason == 'lead_margin' and not g.response_raised


def test_braking_lead_and_no_lead_still_withdraw_at_once():
  for leads in ((GapLead(28., -2.5, -1., 0.),), ()):
    g = warmed()
    prev = raised(g)
    assert cycle(g, 35, moving=True, native=-.7, previous=prev, leads=leads) == -.7
    assert not g.response_raised


def test_margin_ramp_needs_a_raised_request_above_native():
  g = warmed()
  raised(g)
  assert cycle(g, 35, moving=True, native=-.3, previous=-.3, leads=FAR_MARGIN) == -.3
  assert g.response_reason == 'lead_margin'


def test_margin_ramp_does_not_apply_while_preparing():
  g = warmed()
  cycle(g, 15, previous=-.6)
  assert cycle(g, 16, native=-.3, previous=.4, leads=FAR_MARGIN) == -.3
  assert g.response_reason == 'lead_margin'
