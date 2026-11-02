"""The follow-gap press confirmation shows briefly, yields to native alerts and keeps favourite presses allowed."""

import unittest

from openpilot.starpilot.ui.onroad_state import AlertSize, OnroadAlert, is_personality_notice
from openpilot.starpilot.ui.runtime_snapshot import FollowGapNotice


class FollowGapNoticeTests(unittest.TestCase):
  def test_shows_for_its_duration_then_clears(self):
    notice = FollowGapNotice()
    quiet = OnroadAlert()
    self.assertIs(notice.apply(10, quiet), quiet)
    notice.show("1.50 s", "Standard follow gap at 30 mph", 1_000)
    shown = notice.apply(1_000, quiet)
    self.assertEqual((shown.size, shown.text1, shown.text2), (AlertSize.MID, "1.50 s", "Standard follow gap at 30 mph"))
    self.assertTrue(is_personality_notice(shown), "presses stay allowed while the confirmation is on screen")
    self.assertEqual(notice.apply(1_000 + FollowGapNotice.DURATION_NS - 1, quiet).size, AlertSize.MID)
    self.assertIs(notice.apply(1_000 + FollowGapNotice.DURATION_NS, quiet), quiet)

  def test_native_alerts_win_and_a_new_press_replaces_the_text(self):
    notice = FollowGapNotice()
    native = OnroadAlert(AlertSize.SMALL, "Take over", "")
    notice.show("1.50 s", "Standard follow gap at 30 mph", 0)
    self.assertIs(notice.apply(1, native), native)
    notice.show("Follow gap", "Already at the limit", 5, duration_ns=100)
    self.assertEqual(notice.apply(50, OnroadAlert()).text2, "Already at the limit")
    self.assertEqual(notice.apply(105, OnroadAlert()).size, AlertSize.NONE)


if __name__ == "__main__":
  unittest.main()
