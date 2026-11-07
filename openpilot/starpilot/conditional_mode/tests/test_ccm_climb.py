"""Conditional Chill climb mode: Experimental by default, Chill only to climb to the set speed (7 Nov 2026)."""

from dataclasses import replace
import unittest

from openpilot.starpilot.conditional_mode import (
  Authority, ConditionalModePolicy, LeadEvidence, ManualIntent, ModeChoice, ModeSettings, Reason, SceneEvidence,
)
from openpilot.starpilot.conditional_mode.policy import (
  CLIMB_CONFIRM_S, CLIMB_HELD_S, CLIMB_LEAD_AWAY_S, CLIMB_REENTRY_S, CURVE_CLEAR_S, MPH_TO_MPS,
)

AUTHORITY = Authority(True, True, False, True, True, True)
NO_LEAD = LeadEvidence(False, False, False)
CLIMB = replace(ModeSettings(), ccm_climb=True, ccm_lead=False, ccm_speed_mps=20 * MPH_TO_MPS,
                ccm_set_speed_margin_mps=2 * MPH_TO_MPS)
DT = 0.05


def scene(now, speed=40 * MPH_TO_MPS, target=50 * MPH_TO_MPS, accel=0.0, plan3=None, plan8=None, **changes):
  base = SceneEvidence(
    observed_mono_s=now, speed_mps=speed, set_speed_mps=target, climb_target_mps=target, lead=NO_LEAD,
    following_lead=False, speed_lead_present=False, left_blinker=False, right_blinker=False, lane_available=True,
    curve_detected=False, curve_raw=False, slow_lead_detected=False, stop_light_detected=False, standstill=False,
    traffic_mode=False, stop_sign_confirmed=False, forcing_stop=False, adjacent_lead_ambiguous=False,
    low_speed_stop_scene=False, launch_candidate=False, launch_forced_exit=False, launch_lead=False,
    model_accel_mps2=accel, model_speed_3s_mps=speed if plan3 is None else plan3,
    model_speed_8s_mps=speed if plan8 is None else plan8,
  )
  return replace(base, **changes)


class Driver:
  """Steps one policy at 20 Hz; returns the last decision."""

  def __init__(self, settings=CLIMB):
    self.policy = ConditionalModePolicy()
    self.settings = settings
    self.now = 100.0

  def run(self, seconds, authority=AUTHORITY, **kw):
    decision = None
    for _ in range(max(1, round(seconds / DT))):
      self.now = round(self.now + DT, 6)
      decision = self.policy.step(self.now, ModeChoice.CCM, ManualIntent.NONE, authority, scene(self.now, **kw), self.settings)
    return decision


def chill(d):
  return not d.requested_experimental


class ClimbTests(unittest.TestCase):
  def test_clear_road_climb_starts_after_confirmation(self):
    d = Driver()
    self.assertFalse(chill(d.run(CLIMB_CONFIRM_S - 0.2)))
    started = d.run(0.3)
    self.assertTrue(chill(started))
    self.assertEqual((started.reason, started.status_code), (Reason.CCM_SPEED, 6))

  def test_no_climb_while_the_model_is_slowing(self):
    for kw in (dict(accel=-0.2), dict(plan3=40 * MPH_TO_MPS - 1.0), dict(plan8=40 * MPH_TO_MPS - 1.0)):
      with self.subTest(kw=kw):
        self.assertFalse(chill(Driver().run(3.0, **kw)))

  def test_no_climb_within_the_margin_below_the_climb_speed_or_with_unknown_model(self):
    self.assertFalse(chill(Driver().run(3.0, speed=48.5 * MPH_TO_MPS)))
    self.assertFalse(chill(Driver().run(3.0, speed=15 * MPH_TO_MPS, target=30 * MPH_TO_MPS)))
    self.assertFalse(chill(Driver().run(3.0, accel=None)))

  def test_climb_hands_back_when_the_model_slows_or_a_lead_appears(self):
    d = Driver()
    self.assertTrue(chill(d.run(1.5)))
    self.assertFalse(chill(d.run(0.05, accel=-0.35)))
    self.assertEqual(d.policy.climb_end_reason, 'model slowing')
    d = Driver()
    d.run(1.5)
    lead = LeadEvidence(True, True, True, distance_m=40.0, speed_mps=17.0, accel_mps2=0.0)  # 2.2 s ahead
    self.assertFalse(chill(d.run(0.05, lead=lead)))
    self.assertEqual(d.policy.climb_end_reason, 'lead')

  def test_done_at_target_then_a_model_slowdown_is_left_alone(self):
    d = Driver()
    d.run(1.5)
    target = 50 * MPH_TO_MPS
    done = d.run(0.05, speed=target - 0.1 * MPH_TO_MPS)
    self.assertFalse(chill(done))
    self.assertEqual(d.policy.climb_end_reason, 'done')
    d.run(CLIMB_HELD_S + 0.5, speed=target)
    # Experimental then eases 4 mph below on its own and holds there, calm: no climb.
    self.assertFalse(chill(d.run(CLIMB_REENTRY_S + 3.0, speed=target - 4 * MPH_TO_MPS)))

  def test_a_lead_or_set_speed_rise_makes_a_new_climb_possible(self):
    target = 50 * MPH_TO_MPS
    d = Driver()
    d.run(CLIMB_HELD_S + 0.5, speed=target)  # holding speed, never climbed
    self.assertFalse(chill(d.run(CLIMB_REENTRY_S + 2.0, speed=target - 4 * MPH_TO_MPS)))
    lead = LeadEvidence(True, True, True, distance_m=40.0, speed_mps=17.0, accel_mps2=0.0)
    d.run(2.0, speed=target - 6 * MPH_TO_MPS, lead=lead)
    self.assertTrue(chill(d.run(3.0, speed=target - 6 * MPH_TO_MPS)))  # the lead left: recover
    d = Driver()
    d.run(CLIMB_HELD_S + 0.5, speed=target)
    self.assertTrue(chill(d.run(2.0, speed=target, target=target + 5 * MPH_TO_MPS)))  # set speed raised

  def test_driver_accelerator_counts_as_recovery(self):
    target = 50 * MPH_TO_MPS
    d = Driver()
    d.run(CLIMB_HELD_S + 0.5, speed=target)
    d.run(1.0, speed=target - 6 * MPH_TO_MPS)
    d.run(0.5, speed=target - 6 * MPH_TO_MPS, authority=Authority(True, True, False, True, False, True))
    self.assertTrue(chill(d.run(2.0, speed=target - 5 * MPH_TO_MPS)))

  def test_bends_default_waits_after_the_bend_exit_option_does_not(self):
    for exit_option, expect_early in ((False, False), (True, True)):
      with self.subTest(exit_option=exit_option):
        d = Driver(replace(CLIMB, ccm_climb_on_bend_exit=exit_option))
        self.assertEqual(d.run(2.0, curve_detected=True, curve_raw=True).reason, Reason.CCM_VETO)
        early = d.run(CURVE_CLEAR_S - 0.5)
        self.assertEqual(chill(early), expect_early)
        self.assertTrue(chill(d.run(2.0)))

  def test_indicator_is_a_turn_below_35_mph_and_only_pauses_climbs_above(self):
    d = Driver()
    self.assertEqual(d.run(0.5, speed=30 * MPH_TO_MPS, target=40 * MPH_TO_MPS, left_blinker=True).reason, Reason.CCM_VETO)
    d = Driver()
    d.run(1.5)
    self.assertTrue(chill(d.run(1.0, left_blinker=True)))  # a lane change at speed does not flip the mode
    d = Driver()
    self.assertFalse(chill(d.run(3.0, left_blinker=True)))  # but no new climb starts while signalling

  def test_reentry_waits_after_a_climb_ends(self):
    d = Driver()
    d.run(1.5)
    d.run(0.05, accel=-0.4)
    self.assertFalse(chill(d.run(CLIMB_REENTRY_S - 0.5)))
    self.assertTrue(chill(d.run(2.0)))

  def test_lead_pulling_away_option(self):
    speed = 40 * MPH_TO_MPS
    away = LeadEvidence(True, True, True, distance_m=4.0 * speed, speed_mps=speed + 2.0, accel_mps2=0.3)
    self.assertFalse(chill(Driver().run(5.0, lead=away)))  # option off
    d = Driver(replace(CLIMB, ccm_climb_lead=True))
    self.assertFalse(chill(d.run(CLIMB_LEAD_AWAY_S + CLIMB_CONFIRM_S - 0.3, lead=away)))
    self.assertTrue(chill(d.run(0.5, lead=away)))
    self.assertEqual(d.policy.climb_kind, 'lead')
    closing = replace(away, distance_m=2.0 * speed)
    self.assertFalse(chill(d.run(0.05, lead=closing)))
    near = replace(away, distance_m=2.0 * speed)
    self.assertFalse(chill(Driver(replace(CLIMB, ccm_climb_lead=True)).run(5.0, lead=near)))  # never starts within 3 s
    slow = replace(away, speed_mps=speed + 0.5)
    self.assertFalse(chill(Driver(replace(CLIMB, ccm_climb_lead=True)).run(5.0, lead=slow)))  # not really pulling away

  def test_far_lead_that_is_not_slower_lets_a_climb_carry_on(self):
    speed = 40 * MPH_TO_MPS
    d = Driver()
    self.assertTrue(chill(d.run(1.5)))
    far = LeadEvidence(True, False, True, distance_m=5.0 * speed, speed_mps=speed, accel_mps2=0.0)
    self.assertTrue(chill(d.run(2.0, lead=far)))  # 5 s ahead, same speed: carry on
    self.assertFalse(chill(d.run(0.05, lead=replace(far, distance_m=3.0 * speed))))  # within 4 s: hand back
    self.assertEqual(d.policy.climb_end_reason, 'lead')
    d = Driver()
    d.run(1.5)
    slower = LeadEvidence(True, False, True, distance_m=5.0 * speed, speed_mps=speed - 2 * MPH_TO_MPS, accel_mps2=0.0)
    self.assertFalse(chill(d.run(0.05, lead=slower)))  # far but slower: hand back
    d = Driver()
    d.run(1.5)
    braking = replace(far, accel_mps2=-1.0)
    self.assertFalse(chill(d.run(0.05, lead=braking)))  # far but braking: hand back
    self.assertFalse(chill(Driver().run(3.0, lead=far)))  # a far lead still stops a new climb starting

  def test_steady_easing_ends_a_climb_but_noise_does_not(self):
    speed = 40 * MPH_TO_MPS
    d = Driver()
    d.run(1.5)
    self.assertTrue(chill(d.run(0.4, accel=-0.15)))  # brief gentle easing: carry on
    self.assertTrue(chill(d.run(1.0)))
    self.assertFalse(chill(d.run(1.5, accel=-0.15)))  # sustained (about 1 s to confirm): hand back
    self.assertEqual(d.policy.climb_end_reason, 'model easing')
    d = Driver()
    d.run(1.5)
    self.assertTrue(chill(d.run(0.05, plan8=speed - 4 * MPH_TO_MPS)))  # one noisy plan dip: carry on
    self.assertTrue(chill(d.run(1.0)))
    self.assertFalse(chill(d.run(2.0, plan8=speed - 3 * MPH_TO_MPS)))  # a sustained falling plan: hand back
    self.assertEqual(d.policy.climb_end_reason, 'model easing')
    d = Driver()
    d.run(1.5)
    self.assertFalse(chill(d.run(0.05, accel=-0.35)))  # real braking still ends it at once
    self.assertEqual(d.policy.climb_end_reason, 'model slowing')

  def test_climb_mode_off_keeps_the_frozen_speed_rule(self):
    d = Driver(replace(CLIMB, ccm_climb=False))
    entered = d.run(0.5, accel=-0.4)  # the frozen rule ignores the model
    self.assertEqual((entered.reason, chill(entered)), (Reason.CCM_SPEED, True))


if __name__ == '__main__':
  unittest.main()
