"""CEM proposes Chill for the duration of a close-gap lane change, then lets the slower-lead rule re-trigger."""

from dataclasses import replace
import unittest

from openpilot.starpilot.conditional_mode import (
  Authority, ConditionalModePolicy, LeadEvidence, ManualIntent, ModeChoice, ModeSettings, Reason, SceneEvidence,
)

AUTHORITY = Authority(True, True, False, True, True, True)
LEAD = LeadEvidence(True, True, True)


def scene(now: float, **changes) -> SceneEvidence:
  base = SceneEvidence(
    observed_mono_s=now, speed_mps=30.0, set_speed_mps=34.0, lead=LEAD, following_lead=True, speed_lead_present=True,
    left_blinker=False, right_blinker=False, lane_available=True, curve_detected=False, curve_raw=False,
    slow_lead_detected=True, stop_light_detected=False, standstill=False, traffic_mode=False, stop_sign_confirmed=False,
    forcing_stop=False, adjacent_lead_ambiguous=False, low_speed_stop_scene=False, launch_candidate=False,
    launch_forced_exit=False, launch_lead=False,
  )
  return replace(base, **changes)


class LaneChangeChillPolicyTest(unittest.TestCase):
  def _step(self, policy, now, **changes):
    return policy.step(now, ModeChoice.CEM, ManualIntent.NONE, AUTHORITY, scene(now, **changes), ModeSettings())

  def test_slower_lead_experimental_yields_to_lane_change_chill_and_returns(self):
    policy = ConditionalModePolicy()
    first = self._step(policy, 10.0)
    self.assertEqual((first.requested_experimental, first.reason, first.status_code), (True, Reason.CEM_LEAD, 4))
    # indicator on, lane change moving: Chill for the manoeuvre, whatever the lead is doing
    for t in (10.1, 10.2, 12.0, 14.0):
      chill = self._step(policy, t, lane_change_chill=True, left_blinker=True)
      self.assertEqual((chill.requested_experimental, chill.reason, chill.status_code), (False, Reason.CEM_LANE_CHANGE, 0))
      self.assertTrue(chill.qualified)
    # window closed with the slower lead still there: the lead rule re-triggers at once, no leftover hold
    back = self._step(policy, 15.0, lane_change_chill=False)
    self.assertEqual((back.requested_experimental, back.reason, back.status_code), (True, Reason.CEM_LEAD, 4))

  def test_unknown_or_false_window_changes_nothing(self):
    policy = ConditionalModePolicy()
    self._step(policy, 10.0)
    for value in (None, False):
      decision = self._step(policy, 10.1, lane_change_chill=value)
      self.assertEqual((decision.requested_experimental, decision.reason), (True, Reason.CEM_LEAD))

  def test_window_clears_a_running_hold_rather_than_waiting_it_out(self):
    policy = ConditionalModePolicy()
    self._step(policy, 10.0)
    held = self._step(policy, 10.1, slow_lead_detected=False)
    self.assertEqual((held.requested_experimental, held.reason), (True, Reason.CEM_HOLD))
    chill = self._step(policy, 10.2, slow_lead_detected=False, lane_change_chill=True)
    self.assertEqual((chill.requested_experimental, chill.reason), (False, Reason.CEM_LANE_CHANGE))
    after = self._step(policy, 10.3, slow_lead_detected=False)
    self.assertEqual((after.requested_experimental, after.reason), (False, Reason.NO_TRIGGER))

  def test_committed_stop_and_manual_intent_still_win(self):
    policy = ConditionalModePolicy()
    stop = policy.step(10.0, ModeChoice.CEM, ManualIntent.NONE, AUTHORITY,
                       scene(10.0, committed_stop=True, lane_change_chill=True), ModeSettings())
    self.assertEqual((stop.requested_experimental, stop.reason), (True, Reason.CEM_STOP))
    manual = policy.step(10.1, ModeChoice.CEM, ManualIntent.FORCE_EXPERIMENTAL, AUTHORITY,
                         scene(10.1, lane_change_chill=True), ModeSettings())
    self.assertEqual((manual.requested_experimental, manual.reason), (True, Reason.MANUAL_EXPERIMENTAL))


if __name__ == '__main__':
  unittest.main()
