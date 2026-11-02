"""SP-040 behavioural boundaries, not a road-safety qualification."""
import unittest
from dataclasses import replace
from openpilot.starpilot.lateral.lane_change_preferences import LaneChangePolicy, decode, to_value
from openpilot.starpilot.longitudinal.lane_change_gap import (
  GapFrame, GapLead, LaneChangeGap, LaneChangePace, approach_has_room, close_approach,
)
import json

POLICY = LaneChangePolicy(close_gap=True)
NS = 10_000_000_000


def frame(i, *, state='preLaneChange', leads=(GapLead(50., -1., 0.),), **kwargs):
  stamp = NS + i * 50_000_000
  return replace(GapFrame(stamp + 10_000_000, stamp, stamp, stamp, (stamp,) * 3, 20., state, 'left',
    False, False, False, bool(leads), min((l.acceleration_mps2 for l in leads), default=0.),
    False, False, False, False, leads, 'left', True), **kwargs)


def prime(gap, policy=POLICY, *, count=30, leads=(GapLead(50., -1., 0.),)):
  value = None
  for i in range(count):
    value = gap.step(policy, frame(i, leads=leads), base_follow=1.45, long_active=True)
  return value


class OvertakePreparationTests(unittest.TestCase):
  def test_preserves_momentum_without_positive_acceleration(self):
    gap = LaneChangeGap()
    self.assertLess(prime(gap), 1.45)
    self.assertEqual(gap.limit_acceleration(1.2), 0.)
    self.assertEqual(gap.limit_acceleration(-1.5), -1.5)

  def test_slow_steady_lead_and_braking_lead_do_not_get_reduction(self):
    for lead in (GapLead(35., -8., 0.), GapLead(50., 0., -.8), GapLead(15., 0., 0.)):
      with self.subTest(lead=lead):
        gap = LaneChangeGap()
        self.assertIsNone(prime(gap, leads=(lead,)))
        self.assertIsNone(gap.ramped_follow)

  def test_second_lead_is_also_checked(self):
    gap = LaneChangeGap()
    self.assertIsNone(prime(gap, leads=(GapLead(60., 0., 0.), GapLead(25., -4., 0.))))

  def test_room_depends_on_closing_speed_and_lead_braking(self):
    self.assertTrue(approach_has_room(frame(0, leads=(GapLead(40., 0., 0.),)), .75))
    self.assertFalse(approach_has_room(frame(0, leads=(GapLead(40., -8., 0.),)), .75))
    self.assertFalse(approach_has_room(frame(0, leads=(GapLead(40., 0., -1.),)), .75))

  def test_starts_without_accelerating_even_when_no_lead(self):
    gap = LaneChangeGap()
    self.assertIsNone(prime(gap, leads=()))
    self.assertEqual(gap.limit_acceleration(1.), 0.)

  def test_no_reduced_gap_once_steering_starts(self):
    gap = LaneChangeGap()
    self.assertLess(prime(gap), 1.45)
    self.assertIsNone(gap.step(POLICY, frame(30, state='laneChangeStarting'), base_follow=1.45, long_active=True))
    self.assertIsNone(gap.ramped_follow)
    self.assertAlmostEqual(gap.limit_acceleration(2.), .03)
    previous = .03
    for i in range(31,45):
      self.assertIsNone(gap.step(POLICY, frame(i,state='laneChangeStarting'),base_follow=1.45,long_active=True))
      value=gap.limit_acceleration(2.)
      self.assertLessEqual(value-previous,.030000001)
      previous=value
    self.assertGreater(previous,.3)

  def test_new_destination_lead_gets_full_follow_and_unrestricted_native_braking(self):
    gap = LaneChangeGap()
    prime(gap)
    for i in range(30,55):
      gap.step(POLICY,frame(i,state='laneChangeStarting'),base_follow=1.45,long_active=True)
      gap.limit_acceleration(.8)
    new=frame(55,state='laneChangeStarting',leads=(GapLead(22.,-5.,0.),))
    self.assertIsNone(gap.step(POLICY,new,base_follow=1.45,long_active=True))
    self.assertIsNone(gap.ramped_follow)
    self.assertEqual(gap.limit_acceleration(-2.),-2.)
    self.assertEqual(gap.reason,'normal_lead_protection')

  def test_lead_disappearance_settles_without_claiming_clear_lane(self):
    gap=LaneChangeGap();prime(gap)
    for i in range(30,42):
      self.assertIsNone(gap.step(POLICY,frame(i,state='laneChangeStarting',leads=()),base_follow=1.45,long_active=True))
      self.assertGreater(gap.limit_acceleration(.8),0.)
      self.assertEqual(gap.reason,'normal_lead_protection')

  def test_changed_lead_needs_fresh_history_within_original_budget(self):
    gap=LaneChangeGap();prime(gap)
    for i in range(30,60):
      result=gap.step(POLICY,frame(i,leads=(GapLead(80.,0.,0.),)),base_follow=1.45,long_active=True)
      if i<40:self.assertIsNone(result)
      else:self.assertLess(result,1.45)

  def test_timeout_releases_cap_and_does_not_rearm_on_held_signal(self):
    gap=LaneChangeGap();prime(gap)
    previous=0.
    for i in range(30,100):
      value=gap.step(POLICY,frame(i),base_follow=1.45,long_active=True)
      output=gap.limit_acceleration(1.)
      if i>=80:
        self.assertIsNone(value)
        self.assertLessEqual(output-previous,.030000001)
      previous=output
    gap.step(POLICY,frame(100,state='off',signal_direction='none'),base_follow=1.45,long_active=True)
    for i in range(101,130):value=gap.step(POLICY,frame(i),base_follow=1.45,long_active=True)
    self.assertLess(value,1.45)

  def test_aborted_maneuver_does_not_restart_preparation_on_same_signal(self):
    gap = LaneChangeGap()
    prime(gap)
    gap.step(POLICY, frame(30, state='laneChangeStarting'), base_follow=1.45, long_active=True)
    self.assertIsNone(gap.step(POLICY, frame(31), base_follow=1.45, long_active=True))

  def test_vetoes_stale_sources_and_driver_inputs(self):
    changes = [dict(left_blindspot=True), dict(force_decel=True), dict(gas_pressed=True),
               dict(brake_pressed=True), dict(model_should_stop=True), dict(standstill=True),
               dict(signal_direction='right'), dict(speed_mps=4.),
               dict(car_ns=NS), dict(radar_ns=NS), dict(authority_stamps=(NS,)),
               dict(leads=(GapLead(float('nan'), 0., 0.),))]
    for change in changes:
      with self.subTest(change=change):
        gap = LaneChangeGap()
        prime(gap)
        self.assertIsNone(gap.step(POLICY, frame(30, **change), base_follow=1.45, long_active=True))
        self.assertIsNone(gap.ramped_follow)

  def test_source_discontinuity_and_duplicates_remove_privilege(self):
    for index in (29, 10, 100):
      gap = LaneChangeGap()
      prime(gap)
      self.assertIsNone(gap.step(POLICY, frame(index), base_follow=1.45, long_active=True))

  def test_uncertain_sources_revoke_gap_but_keep_no_boost_ceiling(self):
    gap = LaneChangeGap()
    prime(gap)
    self.assertIsNone(gap.step(POLICY, None, base_follow=1.45, long_active=True))
    self.assertEqual(gap.limit_acceleration(1.), 0.)
    self.assertEqual(gap.limit_acceleration(-2.), -2.)

  def test_no_visible_adjacent_lane_never_shortens_gap(self):
    gap = LaneChangeGap()
    for i in range(30):
      self.assertIsNone(gap.step(POLICY, frame(i, adjacent_lane_visible=False), base_follow=1.45, long_active=True))

  def test_disabled_is_neutral(self):
    gap = LaneChangeGap()
    self.assertIsNone(prime(gap, replace(POLICY, close_gap=False)))
    self.assertEqual(gap.limit_acceleration(1.5), 1.5)


class FastPaceTests(unittest.TestCase):
  def prepared(self, policy=replace(POLICY, close_gap_fast_pace=True), leads=(GapLead(33., -1., 0.),)):
    pace = LaneChangePace()
    for i in range(14):
      self.assertEqual(pace.duration(policy, frame(i, leads=leads), long_active=True), policy.duration_s)
    return pace, policy, leads

  def test_default_off_and_saved_pace_unchanged(self):
    pace, policy, leads = self.prepared(POLICY)
    self.assertEqual(pace.duration(policy, frame(14, state='laneChangeStarting', leads=leads), long_active=True), policy.duration_s)
    self.assertFalse(decode(json.dumps(to_value(POLICY)).encode()).close_gap_fast_pace)

  def test_close_qualified_maneuver_uses_ten_then_restores(self):
    pace, policy, leads = self.prepared()
    self.assertEqual(pace.duration(policy, frame(14, state='laneChangeStarting', leads=leads), long_active=True), 3.)
    self.assertEqual(pace.duration(policy, frame(15, state='laneChangeFinishing', leads=leads), long_active=True), 3.)
    self.assertEqual(pace.duration(policy, frame(16, state='off', leads=leads), long_active=True), policy.duration_s)
    self.assertEqual(policy.duration_s, LaneChangePolicy().duration_s)

  def test_gap_and_closing_speed_both_affect_close_threshold(self):
    self.assertFalse(close_approach(frame(0, leads=(GapLead(40., 0., 0.),))))
    self.assertTrue(close_approach(frame(0, leads=(GapLead(40., -3., 0.),))))

  def test_dangerously_close_or_fast_closing_does_not_hasten_steering(self):
    for leads in ((GapLead(8., -4., 0.),), (GapLead(40., -12., 0.),), ()):
      pace, policy, _ = self.prepared(leads=leads)
      self.assertEqual(pace.duration(policy, frame(14, state='laneChangeStarting', leads=leads), long_active=True), policy.duration_s)

  def test_missing_preparation_and_blindspot_do_not_hasten(self):
    pace = LaneChangePace()
    policy = replace(POLICY, close_gap_fast_pace=True)
    self.assertEqual(pace.duration(policy, frame(0, state='laneChangeStarting'), long_active=True), policy.duration_s)
    pace, policy, leads = self.prepared()
    self.assertEqual(pace.duration(policy, frame(14, state='laneChangeStarting', leads=leads, left_blindspot=True),
                                   long_active=True), policy.duration_s)

  def test_close_gap_off_and_long_inactive_are_neutral(self):
    for policy, active in ((replace(POLICY, close_gap=False, close_gap_fast_pace=True), True),
                           (replace(POLICY, close_gap_fast_pace=True), False)):
      pace = LaneChangePace()
      for i in range(20):
        self.assertEqual(pace.duration(policy, frame(i, state='laneChangeStarting'), long_active=active), policy.duration_s)

  def test_duplicate_control_ticks_do_not_accumulate_evidence(self):
    pace, policy, leads = self.prepared()
    self.assertEqual(pace.duration(policy, frame(14, state='laneChangeStarting', leads=leads), long_active=True), 3.)
    for _ in range(10):
      self.assertEqual(pace.duration(policy, frame(14, state='laneChangeStarting', leads=leads), long_active=True), 3.)

  def test_old_saved_preferences_migrate_without_losing_fields(self):
    value = to_value(replace(POLICY, duration_s=4., auto_lane_change=True))
    value['version'] = 5
    del value['closeGapFastPace']
    restored = decode(json.dumps(value).encode())
    self.assertFalse(restored.close_gap_fast_pace)
    self.assertEqual(restored.duration_s, 4.)
    self.assertTrue(restored.auto_lane_change)
    self.assertTrue(restored.close_gap)
    value['version'] = 6
    value['closeGapFastPace'] = 'yes'
    self.assertIsNone(decode(json.dumps(value).encode()))


def fill_adjacent_geometry(md):
  from openpilot.selfdrive.modeld.constants import ModelConstants
  xs = [float(i) for i in range(ModelConstants.IDX_N)]
  md.init('laneLines', 4)
  for line, y in zip(md.laneLines, (-5.4, -1.8, 1.8, 5.4), strict=True):
    line.x, line.y = xs, [y] * len(xs)
  md.laneLineProbs = [.9] * 4
  md.laneLineStds = [.1] * 4
  md.init('roadEdges', 2)
  for line, y in zip(md.roadEdges, (-8., 8.), strict=True):
    line.x, line.y = xs, [y] * len(xs)
  md.roadEdgeStds = [.1, .1]
