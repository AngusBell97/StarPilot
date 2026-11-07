"""Driving Personalities: Custom Following and Jerk leads the hub page, and a personality's rows grey when they do nothing."""

import tempfile
import unittest

from openpilot.common.params import Params
from openpilot.starpilot.ui.feature_settings_owner import FeatureSettingsOwner
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest

MASTER_OFF = "Turn on Custom Following and Jerk (Driving Personalities) first"
FOLLOW_JERK = ("AggressiveFollow", "AggressiveFollowHigh", "AggressiveJerkAcceleration", "AggressiveJerkDeceleration",
               "AggressiveJerkSpeed", "AggressiveJerkSpeedDecrease", "AggressiveJerkDanger")


class PersonalitiesPageTests(unittest.TestCase):
  def setUp(self):
    from opendbc.car.car_helpers import interfaces
    from opendbc.car.toyota.values import CAR
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.params = Params(self.temp.name)
    cp = interfaces[CAR.TOYOTA_COROLLA_TSS2].get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2)
    cp.openpilotLongitudinalControl = True
    self.owner = FeatureSettingsOwner(self.params, lambda group: True, vehicle_fingerprint=lambda: cp.carFingerprint)
    self.owner.vehicle_params = lambda: cp

  def rows(self, page):
    return self.owner.snapshot(page, parked=True, system_long=True, lateral_context=True, metric=False).rows

  def by_key(self, page):
    return {row.key: row for row in self.rows(page) if row.key}

  def test_switch_moved_from_long_planner_to_the_hub(self):
    self.assertNotIn("CustomPersonalities", self.by_key("profiles"))
    hub = self.rows("personalities")
    self.assertEqual(hub[0].key, "CustomPersonalities")
    self.assertTrue(hub[0].available)
    self.assertEqual([row.page for row in hub if row.page], ["aggressive", "standard", "relaxed", "traffic"])
    summary = next(row for row in hub if row.page == "aggressive").value
    self.assertTrue(summary.startswith("Acceleration: ") and " · Braking: " in summary, summary)

  def test_follow_and_jerk_grey_while_the_switch_is_off(self):
    self.params.put_bool("CustomPersonalities", False, block=True)
    rows = self.by_key("aggressive")
    for key in (*FOLLOW_JERK, "profile:aggressive:following"):
      with self.subTest(key=key):
        self.assertFalse(rows[key].available)
        self.assertEqual(rows[key].reason, MASTER_OFF)
        self.assertTrue(rows[key].default_reset_available)
    self.assertTrue(rows["profile:aggressive:acceleration"].available)  # acceleration and braking work either way
    self.assertTrue(rows["profile:aggressive:braking"].available)
    self.params.put_bool("CustomPersonalities", True, block=True)
    rows = self.by_key("aggressive")
    for key in FOLLOW_JERK:
      with self.subTest(on=key):
        self.assertTrue(rows[key].available)

  def test_a_following_curve_greys_low_and_high_speed_follow(self):
    self.params.put_bool("CustomPersonalities", True, block=True)
    preset = self.by_key("aggressive")["profile:aggressive:following"]
    self.assertEqual(preset.value, "StarPilot Default")
    self.assertTrue(self.owner.apply(FeatureSettingsRequest(preset.key, preset.source, "Close",
                                                            vehicle_fingerprint=preset.vehicle_fingerprint)))
    rows = self.by_key("aggressive")
    for key in ("AggressiveFollow", "AggressiveFollowHigh"):
      with self.subTest(key=key):
        self.assertFalse(rows[key].available)
        self.assertIn("Following is set to Close", rows[key].reason)
    self.assertTrue(rows["AggressiveJerkAcceleration"].available)

  def test_slc_limits_follow_use_slc_speed_change_limits(self):
    self.params.put_bool("SLCTransitionOffsets", False, block=True)
    rows = self.by_key("aggressive")
    for key in ("profile:aggressive:acceleration:slc_limit", "profile:aggressive:braking:slc_limit"):
      with self.subTest(key=key):
        self.assertFalse(rows[key].available)
        self.assertIn("Use SLC speed-change limits", rows[key].reason)
    self.params.put_bool("SLCTransitionOffsets", True, block=True)
    self.assertTrue(self.by_key("aggressive")["profile:aggressive:acceleration:slc_limit"].available)

  def test_combined_page_names_presets_by_category_and_sub_pages_keep_theirs(self):
    labels = {row.key: row.label for row in self.rows("aggressive") if row.key}
    self.assertEqual((labels["profile:aggressive:following"], labels["profile:aggressive:acceleration"],
                      labels["profile:aggressive:braking"]), ("Following preset", "Acceleration preset", "Braking preset"))
    # The Longitudinal Curves editor reads the sub-pages by their own labels.
    self.assertEqual(next(row for row in self.rows("aggressive/acceleration") if row.key == "profile:aggressive:acceleration").label, "Preset")


if __name__ == "__main__":
  unittest.main()
