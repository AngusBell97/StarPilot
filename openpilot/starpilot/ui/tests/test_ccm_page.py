"""Chill conditions page (7 Nov 2026): layout follows "Use Chill for", choice rows write saved fields atomically,
dependent rows grey out, no hidden row changes driving, and the comma screen and Galaxy share one description."""
from dataclasses import replace
from pathlib import Path
import json
import unittest

from openpilot.starpilot.conditional_mode import ccm_page
from openpilot.starpilot.conditional_mode.policy import ModeChoice
from openpilot.starpilot.conditional_mode.preferences import MPH_TO_MPS, SavedPreferences, decode_preferences, encode_preferences
from openpilot.starpilot.galaxy.setting_descriptions import description
from openpilot.starpilot.ui.feature_settings_state import FeaturePage
from openpilot.starpilot.ui.tests import test_conditional_feature as base  # module import: no re-collection

PAGE = FeaturePage.CONDITIONAL_CCM
CLIMB_KEYS = ["use", "speed_mps", "set_speed_margin_mps", "after_bend", "climb_lead", "launch_assist", "lead",
              "speed_with_lead_mps", "persist_manual"]
CLASSIC_KEYS = ["use", "speed_mps", "speed_with_lead_mps", "set_speed_margin_mps", "lead", "launch_assist",
                "allow_chill_in_bends", "persist_manual"]
# Saved CCM fields the policy reads in each layout (ModeSettings.ccm_*); every one must be reachable on the page.
CLIMB_EFFECTIVE = {"speed_mps", "set_speed_margin_mps", "allow_chill_in_bends", "climb_on_bend_exit", "climb_lead",
                   "launch_assist", "lead", "speed_with_lead_mps", "persist_manual", "climb"}
CLASSIC_EFFECTIVE = {"speed_mps", "speed_with_lead_mps", "set_speed_margin_mps", "lead", "launch_assist",
                     "allow_chill_in_bends", "persist_manual", "climb"}
ROW_FIELDS = {"use": {"climb"}, "after_bend": {"allow_chill_in_bends", "climb_on_bend_exit"}}


class CcmPageTests(unittest.TestCase):
  # Borrow only the disposable-Params owner and row helpers, not the parent's tests.
  setUp = base.ConditionalFeatureTests.setUp
  rows = base.ConditionalFeatureTests.rows
  row = base.ConditionalFeatureTests.row

  def save(self, **ccm):
    prefs = SavedPreferences(mode=ModeChoice.CCM)
    prefs = replace(prefs, ccm=replace(prefs.ccm, **ccm))
    self.params.put("ConditionalModeConfig", json.loads(encode_preferences(prefs)), block=True)

  def saved(self):
    return decode_preferences(Path(self.params.get_param_path("ConditionalModeConfig")).read_bytes())

  def page(self):
    return {row.key.split(":")[-1]: row for row in self.rows(PAGE)}

  def choose(self, field, value):
    row = self.page()[field]
    self.assertIn(value, row.choices)
    self.assertTrue(self.owner.apply(replace(base.required_change(row), value=value)))

  def test_classic_layout_is_the_original_rule_set(self):
    self.save(climb=False)
    rows = self.page()
    self.assertEqual(list(rows), CLASSIC_KEYS)
    self.assertEqual(rows["use"].value, ccm_page.USE_CLASSIC)
    self.assertEqual(rows["speed_mps"].label, "Chill above: no lead")

  def test_climbing_layout_order_labels_and_values(self):
    self.save(climb=True, climb_lead=True, speed_mps=20 * MPH_TO_MPS, set_speed_margin_mps=2 * MPH_TO_MPS)
    rows = self.page()
    self.assertEqual(list(rows), CLIMB_KEYS)
    self.assertEqual([rows[k].label for k in ("use", "speed_mps", "set_speed_margin_mps", "after_bend")],
                     ["Use Chill for", "Climb from", "Climb when below set speed by", "After a bend"])
    self.assertEqual((rows["speed_mps"].value, rows["set_speed_margin_mps"].value), ("20", "2"))
    self.assertEqual((rows["climb_lead"].value, rows["after_bend"].value), ("On", ccm_page.BEND_WAIT))

  def test_no_hidden_row_changes_driving(self):
    for climb, keys, effective in ((True, CLIMB_KEYS, CLIMB_EFFECTIVE), (False, CLASSIC_KEYS, CLASSIC_EFFECTIVE)):
      shown = set().union(*(ROW_FIELDS.get(k, {k}) for k in keys))
      self.assertEqual(shown, effective, climb)

  def test_use_choice_switches_layout_and_keeps_hidden_values(self):
    self.save(climb=True, climb_lead=True, climb_on_bend_exit=True)
    before = self.saved().ccm
    self.choose("use", ccm_page.USE_CLASSIC)
    after = self.saved().ccm
    self.assertFalse(after.climb)
    self.assertEqual(replace(after, climb=True), before)  # only the choice changed
    self.assertEqual(list(self.page()), CLASSIC_KEYS)
    self.choose("use", ccm_page.USE_CLIMB)
    self.assertEqual(self.saved().ccm, before)

  def test_after_a_bend_writes_both_saved_fields(self):
    self.save(climb=True)
    for choice, allow, on_exit in ((ccm_page.BEND_EXIT, False, True), (ccm_page.BEND_THROUGH, True, False),
                                   (ccm_page.BEND_WAIT, False, False)):
      self.choose("after_bend", choice)
      ccm = self.saved().ccm
      self.assertEqual((ccm.allow_chill_in_bends, ccm.climb_on_bend_exit), (allow, on_exit), choice)
      self.assertEqual(self.page()["after_bend"].value, choice)

  def test_lead_speed_is_greyed_until_its_switch_is_on(self):
    self.save(climb=True, lead=False)
    row = self.page()["speed_with_lead_mps"]
    self.assertFalse(row.available)
    self.assertIn("Turn on", row.reason)
    self.assertIn("Turn on", description(row))
    self.save(climb=True, lead=True)
    self.assertTrue(self.page()["speed_with_lead_mps"].available)

  def test_one_description_per_row_shared_with_galaxy(self):
    self.save(climb=True, lead=True)
    for row in self.page().values():
      if row.available:
        self.assertEqual(description(row), ccm_page.HELP_BY_LABEL[row.label], row.key)

  def test_parent_summary(self):
    self.save(climb=True, climb_lead=True, lead=False, speed_mps=20 * MPH_TO_MPS)  # as saved on the comma
    link = next(r for r in self.rows(FeaturePage.CONDITIONAL) if r.label == "Chill conditions")
    self.assertEqual(link.value, "Climb to set speed from 20 mph · keeps up with leads")
    self.save(climb=True, climb_lead=False, lead=True, speed_mps=20 * MPH_TO_MPS)
    link = next(r for r in self.rows(FeaturePage.CONDITIONAL) if r.label == "Chill conditions")
    self.assertEqual(link.value, "Climb to set speed from 20 mph · Chill behind leads")
    self.save(climb=False)
    link = next(r for r in self.rows(FeaturePage.CONDITIONAL) if r.label == "Chill conditions")
    self.assertEqual(link.value, "Classic conditions")
