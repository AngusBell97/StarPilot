"""Cancelling the turn signal stops a lane change that has already started."""

from types import SimpleNamespace
import unittest

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.starpilot.lateral.lane_change_preferences import LaneChangePolicy

Starting = log.LaneChangeState.laneChangeStarting
Off = log.LaneChangeState.off
Pre = log.LaneChangeState.preLaneChange


def car(**changes):
  values = {"vEgo": 30 * CV.MPH_TO_MS, "leftBlinker": False, "rightBlinker": False,
            "steeringPressed": False, "steeringTorque": 0.0, "leftBlindspot": False, "rightBlindspot": False}
  values.update(changes)
  return SimpleNamespace(**values)


def auto_policy():
  # The comma's saved settings: automatic, 0.2 s delay, one lane change per signal.
  return LaneChangePolicy(True, 0.0, True, True, 0.2, 0.0)


def step(helper, state, probability=1.0):
  helper.update(state, True, probability, auto_evidence=True, engaged=True)
  return helper.lane_change_state


def start_auto(helper, **signal):
  for _ in range(10):
    if step(helper, car(**signal)) == Starting:
      return
  raise AssertionError("lane change never started")


class LaneChangeCancelTests(unittest.TestCase):
  def test_signal_off_cancels_started_auto_lane_change_at_once(self):
    for when in (1, 10, 40, 90):   # 0.05 s, 0.5 s, 2 s and 4.5 s into the change
      helper = DesireHelper(auto_policy())
      start_auto(helper, leftBlinker=True)
      for _ in range(when):
        self.assertEqual(step(helper, car(leftBlinker=True)), Starting)
      self.assertEqual(helper.desire, log.Desire.laneChangeLeft)
      self.assertEqual(step(helper, car()), Off)
      self.assertEqual((helper.lane_change_direction, helper.desire), (log.LaneChangeDirection.none, log.Desire.none))

  def test_signal_off_cancels_nudged_lane_change(self):
    helper = DesireHelper(LaneChangePolicy(True, 0.0, True, False, 1.0, 0.0))
    step(helper, car(rightBlinker=True))
    self.assertEqual(step(helper, car(rightBlinker=True, steeringPressed=True, steeringTorque=-1)), Starting)
    self.assertEqual(helper.desire, log.Desire.laneChangeRight)
    self.assertEqual(step(helper, car()), Off)
    self.assertEqual(helper.desire, log.Desire.none)

  def test_hazards_cancel(self):
    helper = DesireHelper(auto_policy())
    start_auto(helper, leftBlinker=True)
    self.assertEqual(step(helper, car(leftBlinker=True, rightBlinker=True)), Off)

  def test_reversed_signal_cancels_and_needs_a_fresh_signal(self):
    helper = DesireHelper(auto_policy())
    start_auto(helper, leftBlinker=True)
    self.assertEqual(step(helper, car(rightBlinker=True)), Off)
    for _ in range(20):   # held the other way without a gap: no new lane change
      self.assertEqual(step(helper, car(rightBlinker=True)), Off)
    step(helper, car())
    self.assertEqual(step(helper, car(rightBlinker=True)), Pre)

  def test_new_signal_after_cancel_starts_again(self):
    helper = DesireHelper(auto_policy())
    start_auto(helper, leftBlinker=True)
    self.assertEqual(step(helper, car()), Off)
    start_auto(helper, leftBlinker=True)
    self.assertEqual(helper.desire, log.Desire.laneChangeLeft)

  def test_held_signal_still_finishes_when_the_model_says_done(self):
    helper = DesireHelper(auto_policy())
    start_auto(helper, leftBlinker=True)
    for _ in range(60):
      self.assertEqual(step(helper, car(leftBlinker=True), 0.9), Starting)
    # Model reports the change done; one lane change per signal, so it stops rather than starting another.
    self.assertEqual(step(helper, car(leftBlinker=True), 0.0), Off)
    for _ in range(20):
      self.assertEqual(step(helper, car(leftBlinker=True), 0.0), Off)


if __name__ == "__main__":
  unittest.main()
