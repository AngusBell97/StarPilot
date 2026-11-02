from openpilot.starpilot.longitudinal.tests.test_overtake_preparation import fill_adjacent_geometry
"""Saved close-gap admission and real native MPC follow-time behavior."""

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.plannerd import update_curve_frame
from openpilot.starpilot.lateral.lane_change_preferences import KEY, LaneChangePolicy, decode, to_value
from openpilot.starpilot.curve_speed.host import CurveHost
from openpilot.starpilot.longitudinal.lane_change_gap import GapFrame, LaneChangeGap, LaneGapPreferences, project
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import messages, snapshot
from openpilot.starpilot.longitudinal.tests.test_ioniq6_start import candidate
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR


BASE_NS = 10_000_000_000
FRAME_NS = 50_000_000
POLICY = LaneChangePolicy(minimum_speed_mps=0.0, close_gap=True, close_gap_seconds=0.75)


def required_float(value: float | None) -> float:
  if value is None:
    raise AssertionError('expected a reduced follow time')
  return value


def frame(index, *, lane_state='preLaneChange', direction='left', left_blindspot=False,
          lead_accel=0.0, speed=20.0, force=False, gas=False, brake=False, should_stop=False):
  stamp = BASE_NS + index * FRAME_NS
  return GapFrame(stamp + 10_000_000, stamp, stamp, stamp, (stamp,) * 3, speed, lane_state, direction,
                  speed <= 0, left_blindspot, False, True, lead_accel, force, gas, brake, should_stop)


class SavedLaneGapTests(unittest.TestCase):
  def test_v1_v2_decode_default_off_v3_is_strict(self):
    for document in ({'version': 1, 'enabled': True, 'minimumSpeedMps': 8.9408, 'onePerSignal': False},
                     {'version': 2, 'enabled': True, 'minimumSpeedMps': 8.9408, 'onePerSignal': False,
                      'autoLaneChange': False, 'autoDelayS': 1.0, 'minimumLaneWidthM': 0.0}):
      restored = decode(json.dumps(document).encode())
      self.assertIsNotNone(restored)
      self.assertFalse(restored.close_gap)
      self.assertEqual(restored.close_gap_seconds, 0.75)
    self.assertEqual(decode(json.dumps(to_value(POLICY)).encode()), POLICY)
    for seconds in (0.6, 0.749, 1.001, True, float('nan')):
      raw = to_value(POLICY)
      raw['closeGapSeconds'] = seconds
      self.assertIsNone(decode(json.dumps(raw).encode()))

  def test_absent_invalid_and_unreadable_never_opt_in(self):
    with tempfile.TemporaryDirectory() as path:
      params = Params(path)
      host = LaneGapPreferences(params)
      self.assertIsNone(host.sample(BASE_NS))
      params.put(KEY, to_value(POLICY), block=True)
      self.assertEqual(host.sample(BASE_NS + 1_000_000_000), POLICY)
      self.assertEqual(host.sample(BASE_NS // 2), POLICY)
      Path(params.get_param_path(KEY)).write_bytes(b'{broken')
      self.assertIsNone(host.sample(BASE_NS // 2 + 1_000_000_000))


class NativeFrame(dict):
  def __init__(self, payloads):
    super().__init__(payloads)
    self.logMonoTime = dict.fromkeys(('modelV2', 'carState', 'radarState', 'carControl', 'controlsState', 'selfdriveState'), BASE_NS)
    self.valid = dict.fromkeys(self.logMonoTime, True)
    self.alive = dict.fromkeys(self.logMonoTime, True)


class NativeLaneGapTests(unittest.TestCase):
  def test_replay_clock_from_host_qualifies_both_gates_live_clock_does_not(self):
    _, cp = candidate()
    replay = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: BASE_NS + 100_000_000_000)
    live_default = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: BASE_NS + 100_000_000_000)
    replay_sm = NativeFrame(messages(lead=True)[0])
    live_sm = NativeFrame(messages(lead=True)[0])
    for sm in (replay_sm, live_sm):
      sm['carControl'].longActive = True
      sm['carState'].canValid = True
      sm['carState'].leftBlinker = True
      for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
        lead.dRel = 50.
        lead.vRel = -1.
        lead.vLead = lead.vLeadK = 19.
      sm['modelV2'].meta.laneChangeState = 'preLaneChange'
      sm['modelV2'].meta.laneChangeDirection = 'left'
      fill_adjacent_geometry(sm['modelV2'])
      sm['modelV2'].meta.disengagePredictions.gasPressProbs = [0.34] * 6
    for index in range(20):
      stamp = BASE_NS + index * FRAME_NS
      for sm in (replay_sm, live_sm):
        sm.logMonoTime = dict.fromkeys(sm.logMonoTime, stamp)
      update_curve_frame(replay, replay_sm, cp, stamp + 10_000_000, lane_change_policy=POLICY)
      live_default.update(live_sm, lane_change_policy=POLICY)
      self.assertAlmostEqual(live_default.mpc.params[0, 4], 1.45)
      self.assertFalse(live_default.allow_throttle)
      self.assertEqual(replay.allow_throttle, index < 5)
    self.assertLess(replay.mpc.params[0, 4], 1.45)
    self.assertEqual(replay.mpc.solution_status, 0)
    self.assertEqual(live_default.mpc.solution_status, 0)

  def test_original_fast_closing_fixture_is_rejected_and_braking_is_preserved(self):
    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    stock = LongitudinalPlanner(cp, init_v=20.)
    opted = LongitudinalPlanner(cp, init_v=20.)
    for index in range(30):
      sm = NativeFrame(messages(lead=True)[0])
      stamp = BASE_NS + index * FRAME_NS
      sm.logMonoTime = dict.fromkeys(sm.logMonoTime, stamp)
      sm['carControl'].longActive = True
      sm['carState'].canValid = True
      sm['carState'].leftBlinker = True
      sm['modelV2'].meta.laneChangeState = 'preLaneChange'
      sm['modelV2'].meta.laneChangeDirection = 'left'
      fill_adjacent_geometry(sm['modelV2'])
      stock.update(sm, now_ns=stamp + 10_000_000)
      opted.update(sm, now_ns=stamp + 10_000_000, lane_change_policy=POLICY)
      self.assertAlmostEqual(opted.mpc.params[0, 4], 1.45)
      self.assertLessEqual(opted.output_a_target, stock.output_a_target + 1e-6)
      self.assertEqual(opted.mpc.solution_status, 0)

  def test_default_off_is_native_even_with_model_lane_change(self):
    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    now = [BASE_NS + 10_000_000]
    native = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: now[0])
    default_off = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: now[0])
    streams = (NativeFrame(messages(lead=True)[0]), NativeFrame(messages(lead=True)[0]))
    for sm in streams:
      sm['carControl'].longActive = True
      sm['carState'].canValid = True
      sm['carState'].leftBlinker = True
      for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
        lead.dRel = 50.
        lead.vRel = -1.
        lead.vLead = lead.vLeadK = 19.
      sm['modelV2'].meta.laneChangeState = 'preLaneChange'
      sm['modelV2'].meta.laneChangeDirection = 'left'
      fill_adjacent_geometry(sm['modelV2'])
    for index in range(20):
      stamp = BASE_NS + index * FRAME_NS
      now[0] = stamp + 10_000_000
      for sm in streams:
        sm.logMonoTime = dict.fromkeys(sm.logMonoTime, stamp)
      native.update(streams[0])
      default_off.update(streams[1], lane_change_policy=LaneChangePolicy())
      self.assertEqual(snapshot(native), snapshot(default_off))
      self.assertAlmostEqual(native.mpc.params[0, 4], default_off.mpc.params[0, 4])

  def test_saved_opt_in_reduces_follow_and_caps_positive_acceleration(self):
    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    now = [BASE_NS + 10_000_000]
    stock = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: now[0])
    opted = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: now[0])
    left, right = NativeFrame(messages(lead=True)[0]), NativeFrame(messages(lead=True)[0])
    for sm in (left, right):
      sm['carControl'].longActive = True
      sm['carState'].canValid = True
      sm['carState'].leftBlinker = True
      for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
        lead.dRel = 50.
        lead.vRel = -1.
        lead.vLead = lead.vLeadK = 19.
      sm['modelV2'].meta.laneChangeState = 'preLaneChange'
      sm['modelV2'].meta.laneChangeDirection = 'left'
      fill_adjacent_geometry(sm['modelV2'])
    for index in range(20):
      stamp = BASE_NS + index * FRAME_NS
      now[0] = stamp + 10_000_000
      for sm in (left, right):
        sm.logMonoTime = dict.fromkeys(sm.logMonoTime, stamp)
      stock.update(left)
      opted.update(right, lane_change_policy=POLICY)
      self.assertEqual(stock.mpc.solution_status, 0)
      self.assertEqual(opted.mpc.solution_status, 0)
      self.assertEqual(stock.mpc.source, opted.mpc.source)
      self.assertAlmostEqual(stock.mpc.params[0, 4], 1.45)
    self.assertLess(opted.mpc.params[0, 4], 1.45)
    self.assertGreaterEqual(opted.mpc.params[0, 4], 0.75)
    self.assertLessEqual(opted.output_a_target, 0.)
    right['modelV2'].meta.laneChangeState = 'laneChangeStarting'
    stamp += FRAME_NS
    now[0] = stamp + 10_000_000
    right.logMonoTime = dict.fromkeys(right.logMonoTime, stamp)
    opted.update(right, lane_change_policy=POLICY)
    self.assertAlmostEqual(opted.mpc.params[0, 4], 1.45)
    self.assertGreater(opted.output_a_target, 0.)
    self.assertLessEqual(opted.output_a_target, .030000001)
    right.valid['modelV2'] = False
    stamp += FRAME_NS
    now[0] = stamp + 10_000_000
    right.logMonoTime = dict.fromkeys(right.logMonoTime, stamp)
    opted.update(right, lane_change_policy=POLICY)
    self.assertAlmostEqual(opted.mpc.params[0, 4], 1.45)
    self.assertEqual(opted.mpc.solution_status, 0)
    right.valid['modelV2'] = True
    for safety_change in ('lead_two_nan', 'lead_two', 'force', 'stop'):
      stamp += FRAME_NS
      now[0] = stamp + 10_000_000
      right.logMonoTime = dict.fromkeys(right.logMonoTime, stamp)
      if safety_change == 'lead_two_nan':
        right['radarState'].leadTwo.aLeadK = float('nan')
        self.assertIsNone(project(right, now[0]))
        right['radarState'].leadTwo.aLeadK = 0.0
        continue
      elif safety_change == 'lead_two':
        right['radarState'].leadTwo.aLeadK = -0.8
      elif safety_change == 'force':
        right['radarState'].leadTwo.aLeadK = 0.0
        right['controlsState'].forceDecel = True
      else:
        right['controlsState'].forceDecel = False
        right['modelV2'].action.shouldStop = True
      opted.update(right, lane_change_policy=POLICY)
      self.assertAlmostEqual(opted.mpc.params[0, 4], 1.45)
      self.assertEqual(opted.mpc.solution_status, 0)

  def test_curve_receives_same_cycle_reduced_follow_time(self):
    cp = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
    planner = LongitudinalPlanner(cp, init_v=20.0, clock_ns=lambda: BASE_NS + 100_000_000_000)
    sm = NativeFrame(messages(lead=True)[0])
    sm['carControl'].longActive = True
    sm['carState'].canValid = True
    sm['carState'].leftBlinker = True
    for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
      lead.dRel = 50.
      lead.vRel = -1.
      lead.vLead = lead.vLeadK = 19.
    sm['modelV2'].meta.laneChangeState = 'preLaneChange'
    sm['modelV2'].meta.laneChangeDirection = 'left'
    fill_adjacent_geometry(sm['modelV2'])
    selected = []
    host = CurveHost(enabled=False, replay=True)

    def capture(_host, _sm, _cp, _now_ns, *, follow_time_s, **_kwargs):
      selected.append(follow_time_s)
      return None, None

    with patch('openpilot.selfdrive.controls.plannerd.curve_for_frame', side_effect=capture):
      for index in range(20):
        stamp = BASE_NS + index * FRAME_NS
        sm.logMonoTime = dict.fromkeys(sm.logMonoTime, stamp)
        update_curve_frame(planner, sm, cp, stamp, host=host, lane_change_policy=POLICY)
        self.assertAlmostEqual(selected[-1], float(planner.mpc.params[0, 4]))
    self.assertLess(selected[-1], 1.45)


if __name__ == '__main__':
  unittest.main()
