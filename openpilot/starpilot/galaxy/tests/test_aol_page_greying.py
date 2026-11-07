"""Galaxy Always On Lateral page: switch first, Brake pause below greyed while AOL is off (Restore defaults still resets it),
steering-pause rows unaffected."""
from typing import cast
from unittest import TestCase
from opendbc.car import gen_empty_fingerprint
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR as HONDA
from openpilot.starpilot.galaxy.defaults_recovery import DefaultsRecovery
from openpilot.starpilot.galaxy.settings import AuthorityContext, SettingsChanged
from openpilot.starpilot.galaxy.tests import test_settings as fixtures

PAUSE = ("Pause steering below", "Pause only while signaling", "Steering resume delay")


class TestAolPageGreying(TestCase):
  def setUp(self):
    fixtures.SettingsGatewayTest.setUp(cast(fixtures.SettingsGatewayTest, self))
    honda = CarInterface.get_params(HONDA.HONDA_CIVIC_BOSCH, gen_empty_fingerprint(), [], True, False, False)
    honda.safetyConfigs[-1].safetyParam |= 0x20
    self.context.value = AuthorityContext(True, honda, b"honda-aol-greying")

  def page(self):
    return self.gateway.page("aol", self.session, self.generation)

  def test_order_and_greying_follow_the_switch(self):
    self.params.put_bool("AlwaysOnLateral", False, block=True)
    rows = self.page()["rows"]
    self.assertEqual([row["label"] for row in rows], ["Enable Always On Lateral", "Brake pause below", *PAUSE])
    by_label = {row["label"]: row for row in rows}
    self.assertTrue(by_label["Enable Always On Lateral"]["available"])
    self.assertFalse(by_label["Brake pause below"]["available"])
    self.assertTrue(by_label["Brake pause below"]["reason"].endswith("Turn on Always On Lateral first"))
    self.assertTrue(all(by_label[label]["available"] for label in PAUSE))
    self.params.put_bool("AlwaysOnLateral", True, block=True)
    by_label = {row["label"]: row for row in self.page()["rows"]}
    self.assertTrue(by_label["Brake pause below"]["available"])
    self.assertNotIn("Turn on", by_label["Brake pause below"]["reason"])

  def test_greyed_threshold_refuses_edits(self):
    self.params.put_bool("AlwaysOnLateral", False, block=True)
    page = self.page()
    index = next(i for i, row in enumerate(page["rows"]) if row["label"] == "Brake pause below")
    with self.assertRaises(SettingsChanged):
      self.gateway.preview(page["view"], index, 0, self.session, self.generation, value=20)

  def test_restore_defaults_resets_greyed_threshold_without_turning_aol_on(self):
    self.params.put_bool("AlwaysOnLateral", True, block=True)
    self.params.put("AolBrakePauseSpeedMps", 5.0, block=True)
    self.params.put_bool("AlwaysOnLateral", False, block=True)
    recovery = DefaultsRecovery(self.gateway)
    view = recovery.snapshot(self.session, self.generation)
    rows = [row for row in view["rows"] if row["page"] == "aol" and row["label"] == "Brake pause below"]
    self.assertEqual(len(rows), 1)
    self.assertTrue(rows[0]["resetAvailable"] and not rows[0]["available"])
    pending = recovery.preview(view["view"], [rows[0]["id"]], self.session, self.generation)
    self.assertTrue(recovery.confirm(pending["intent"], self.session, self.generation)["saved"])
    self.assertEqual(self.params.get("AolBrakePauseSpeedMps"), 0.0)
    self.assertFalse(self.params.get_bool("AlwaysOnLateral"))
