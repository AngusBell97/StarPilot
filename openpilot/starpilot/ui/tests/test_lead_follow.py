"""Lead follow time, its target and the custom-curve nudge are pure arithmetic over the saved profile document."""

import json
import unittest
from types import SimpleNamespace as NS

from openpilot.starpilot.longitudinal.profile_document import (FOLLOWING_SPEEDS_MPH, default_personality_profiles, interpolate_category_curve,
                                                               profile_document, strict_profile_document)
from openpilot.starpilot.ui.lead_follow import (DEFAULT_FOLLOW_S, NUDGE_STEP_S, STOP_DISTANCE_M, FollowTargetReader, default_follow_s,
                                                encode_profile_document, follow_state, follow_time_s, interpolated_follow_s, ladder_fraction,
                                                lead_distance_text, lead_speed_text, lead_time_text, nudge_following_curve,
                                                nudged_profile_document)

MPH = 0.44704


def custom_document(curve=None, personality="standard", enabled=True):
  profiles = default_personality_profiles(False)
  profiles[personality]["following"] = {"preset": "custom", "curve": list(curve or [1.5] * len(FOLLOWING_SPEEDS_MPH))}
  return strict_profile_document(profile_document(profiles, enabled=enabled))


class FollowTimeTests(unittest.TestCase):
  def test_live_follow_time_removes_the_planner_stop_margin(self):
    self.assertAlmostEqual(follow_time_s(STOP_DISTANCE_M + 30.0, 20.0), 1.5)
    self.assertEqual(follow_time_s(3.0, 20.0), 0.0, "inside the stop margin reads as zero, never negative")
    for distance, speed in ((30.0, 1.0), (0.0, 20.0), (float("nan"), 20.0), ("x", 20.0), (30.0, None)):
      self.assertIsNone(follow_time_s(distance, speed))

  def test_state_bands_and_ladder_position(self):
    self.assertEqual(follow_state(None, 1.5), "none")
    self.assertEqual(follow_state(1.5, None), "none")
    self.assertEqual(follow_state(2.1, 1.5), "far")
    self.assertEqual(follow_state(1.5, 1.5), "target")
    self.assertEqual(follow_state(1.3, 1.5), "target")
    self.assertEqual(follow_state(1.2, 1.5), "near")
    self.assertEqual(follow_state(0.8, 1.5), "close")
    self.assertEqual(ladder_fraction(1.5, 1.5), 0.5, "on target sits on the rung")
    self.assertEqual(ladder_fraction(0.0, 1.5), 0.0)
    self.assertEqual(ladder_fraction(9.0, 1.5), 1.0)
    self.assertIsNone(ladder_fraction(1.0, 0.0))

  def test_widget_text(self):
    lead = NS(distance_m=50.0, speed_mps=20.0)
    self.assertEqual(lead_speed_text(lead, False), "45 mph")
    self.assertEqual(lead_speed_text(lead, True), "72 km/h")
    self.assertEqual(lead_distance_text(lead, True), "50 m")
    self.assertEqual(lead_distance_text(lead, False), "164 ft")
    self.assertEqual(lead_time_text(lead, 20.0), "2.2 s")
    self.assertEqual(lead_time_text(lead, 0.5), "")
    self.assertEqual(lead_time_text(None, 20.0), "")
    self.assertEqual(lead_speed_text(None, False), "")

  def test_default_targets_follow_the_personality(self):
    self.assertEqual(default_follow_s(0, False), DEFAULT_FOLLOW_S["aggressive"])
    self.assertEqual(default_follow_s(1, False), DEFAULT_FOLLOW_S["standard"])
    self.assertEqual(default_follow_s(2, False), DEFAULT_FOLLOW_S["relaxed"])
    self.assertEqual(default_follow_s(1, True), DEFAULT_FOLLOW_S["traffic"])
    reader = FollowTargetReader(params=None)
    self.assertEqual(reader.sample(0, 2, 20.0, None, traffic_mode=False), DEFAULT_FOLLOW_S["relaxed"], "no car params: the default")
    self.assertIsNone(reader.sample(0, 7, 20.0, None, traffic_mode=False))


class NudgeTests(unittest.TestCase):
  def test_on_a_breakpoint_moves_only_that_point(self):
    curve = [1.5] * 10
    nudged = nudge_following_curve(curve, 30.0, NUDGE_STEP_S)
    self.assertEqual(nudged, [1.5, 1.5, 1.5, 1.55, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5])
    self.assertEqual(nudge_following_curve(curve, 0.0, -NUDGE_STEP_S)[0], 1.45)
    self.assertEqual(nudge_following_curve(curve, 95.0, NUDGE_STEP_S)[-1], 1.55)

  def test_between_breakpoints_the_interpolated_value_moves_by_one_step(self):
    curve = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8]
    for speed_mph in (23.0, 25.0, 27.5, 41.0):
      with self.subTest(speed_mph=speed_mph):
        before = interpolated_follow_s(curve, speed_mph)
        nudged = nudge_following_curve(curve, speed_mph, NUDGE_STEP_S)
        self.assertAlmostEqual(interpolated_follow_s(nudged, speed_mph) - before, NUDGE_STEP_S, places=3)
        low = int(speed_mph // 10)
        self.assertEqual([index for index, (old, new) in enumerate(zip(curve, nudged, strict=True)) if old != new], [low, low + 1])
        if speed_mph % 10 < 5:
          self.assertGreater(nudged[low] - curve[low], nudged[low + 1] - curve[low + 1], "the nearer point takes the bigger share")

  def test_bounds_and_invalid_curves(self):
    self.assertIsNone(nudge_following_curve([3.0] * 10, 30.0, NUDGE_STEP_S), "already at the maximum")
    self.assertIsNone(nudge_following_curve([0.5] * 10, 30.0, -NUDGE_STEP_S), "already at the minimum")
    self.assertEqual(nudge_following_curve([2.98] * 10, 30.0, NUDGE_STEP_S)[3], 3.0)
    self.assertIsNone(nudge_following_curve([1.5] * 9, 30.0, NUDGE_STEP_S))
    self.assertIsNone(nudge_following_curve([1.5] * 10, float("nan"), NUDGE_STEP_S))
    self.assertIsNone(nudge_following_curve([1.5] * 10, 30.0, 0.0))

  def test_document_nudge_targets_the_active_custom_profile_only(self):
    document = custom_document(personality="standard")
    nudged = nudged_profile_document(document, 1, False, 30.0 * MPH, NUDGE_STEP_S)
    self.assertIsNotNone(nudged)
    self.assertEqual(nudged["profiles"]["standard"]["following"]["curve"][3], 1.55)
    self.assertEqual(nudged["profiles"]["relaxed"], document["profiles"]["relaxed"])
    self.assertAlmostEqual(interpolate_category_curve("following", 30.0 * MPH, nudged["profiles"]["standard"]["following"], False), 1.55)
    self.assertIsNone(nudged_profile_document(document, 2, False, 30.0 * MPH, NUDGE_STEP_S), "relaxed is still a preset")
    self.assertIsNone(nudged_profile_document(document, 1, True, 30.0 * MPH, NUDGE_STEP_S), "traffic is still a preset")
    self.assertIsNone(nudged_profile_document(document, 1, False, None, NUDGE_STEP_S))
    self.assertIsNone(nudged_profile_document({}, 1, False, 10.0, NUDGE_STEP_S))
    self.assertEqual(document, custom_document(personality="standard"), "the saved document is never mutated in place")

  def test_encoded_document_round_trips_through_the_strict_reader(self):
    document = custom_document(personality="aggressive")
    nudged = nudged_profile_document(document, 0, False, 55.0 * MPH, -NUDGE_STEP_S)
    raw = encode_profile_document(nudged)
    self.assertEqual(json.loads(raw), nudged)
    self.assertEqual(strict_profile_document(raw), nudged)
    self.assertEqual(raw, encode_profile_document(json.loads(raw)), "canonical: sorted keys, no spaces")


if __name__ == "__main__":
  unittest.main()
