"""SP-040 moving response: a changed lead observation mid-manoeuvre holds the raised request instead of withdrawing it."""
import pytest
from openpilot.starpilot.longitudinal.lane_change_gap import GapLead
from openpilot.starpilot.longitudinal.tests.test_lane_change_response import warmed, cycle

CHANGED = (GapLead(85., -2., 0., 0.),)


def test_changed_lead_during_movement_holds_then_resumes():
  g = warmed()
  prev = cycle(g, 15, moving=True, previous=0.)
  assert prev > 0. and g.response_reason == 'normal_acceleration'
  for i in range(16, 26):
    value = cycle(g, i, moving=True, leads=CHANGED, previous=prev)
    assert value == pytest.approx(prev, abs=1e-9)  # held: no withdrawal to native, no rise either
    assert g.response_reason == 'lead_history_hold' and g.response_raised and not g.response_aborted
    prev = value
  value = cycle(g, 26, moving=True, leads=CHANGED, previous=prev)
  assert value > prev and g.response_reason == 'normal_acceleration'


def test_hold_keeps_a_released_request_that_is_still_negative():
  g = warmed()
  cycle(g, 15, moving=True, previous=-.4)
  value = cycle(g, 16, moving=True, leads=CHANGED, native=-.6, previous=-.3)
  assert value == pytest.approx(-.3, abs=1e-9) and g.response_reason == 'lead_history_hold'


@pytest.mark.parametrize('change', [
  {'leads': (GapLead(85., -2., 0., 0.), GapLead(15., -5., 0., 0.))},  # a close new lead: no room
  {'leads': (GapLead(85., -2., -1., 0.),)},                            # braking lead
  {'leads': ()},                                                        # no lead at all
  {'left_blindspot': True}, {'gas_pressed': True}, {'brake_pressed': True},
])
def test_hazards_still_withdraw_during_a_hold(change):
  g = warmed()
  cycle(g, 15, moving=True, previous=0.)
  assert cycle(g, 16, moving=True, native=-.7, previous=.3, **change) == -.7
  assert not g.response_raised


def test_strong_braking_and_model_braking_still_win_over_a_hold():
  g = warmed()
  cycle(g, 15, moving=True, previous=0.)
  assert cycle(g, 16, moving=True, leads=CHANGED, native=-1.1, previous=.3) == -1.1
  assert g.response_reason == 'strong_braking'
  g = warmed()
  cycle(g, 15, moving=True, previous=0.)
  assert cycle(g, 16, moving=True, leads=CHANGED, native=-.6, previous=.3, experimental=True) == -.6
  assert g.response_reason == 'model_braking'


def test_preparing_phase_still_withdraws_on_a_changed_lead():
  g = warmed()
  assert cycle(g, 15, previous=-.6) > -.6 and g.response_reason == 'signal_hold'
  assert cycle(g, 16, leads=CHANGED, previous=-.5) == -.6
  assert g.response_reason == 'lead_history'


def test_same_car_stepping_in_the_radar_holds():
  # Drive 28: the car being passed steps 3-6 m a frame and slides sideways; both tracks shift together.
  g = warmed()
  prev = cycle(g, 15, moving=True, previous=0.)
  steps = [(GapLead(68.5, -2., 0., .6), GapLead(69.5, -2., 0., .6)), (GapLead(72.5, -1.5, 0., 1.1), GapLead(73.5, -1.5, 0., 1.1)),
           (GapLead(70., -1.5, 0., 1.4),), (GapLead(75., -1., 0., 1.9),)]
  for i, leads in enumerate(steps, start=16):
    value = cycle(g, i, moving=True, leads=leads, previous=prev)
    assert value == pytest.approx(prev, abs=1e-9) and g.response_reason == 'lead_history_hold'
    prev = value


def test_new_closer_car_during_a_hold_withdraws():
  g = warmed()
  cycle(g, 15, moving=True, previous=0.)
  assert cycle(g, 16, moving=True, leads=(GapLead(40., -2., 0., 0.),), native=-.6, previous=.3) == -.6
  assert g.response_reason == 'lead_history'


def test_new_lead_without_room_for_the_held_value_withdraws():
  # A changed observation is held only while every current lead has room for the held value.
  g = warmed()
  cycle(g, 15, moving=True, previous=0.)
  tight = (GapLead(25., 0., 0., 3.),)
  assert cycle(g, 16, moving=True, leads=tight, native=-.7, previous=.3) == -.7
  assert g.response_reason in ('lead_history', 'lead_margin')
