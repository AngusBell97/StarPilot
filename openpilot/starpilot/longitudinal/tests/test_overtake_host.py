from openpilot.starpilot.longitudinal.tests.test_overtake_preparation import fill_adjacent_geometry
"""Actual Controls.state_control with real messages/controllers; no vehicle validation."""
import os
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from dataclasses import replace

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.starpilot.lateral.lane_centering import LaneCenteringController
from openpilot.starpilot.lateral.lane_change_preferences import LaneChangePolicy
from openpilot.starpilot.lateral.lane_change_smoothing import LaneChangeSmoother
from openpilot.starpilot.lateral.pause import LateralPause, PauseSettings
from openpilot.starpilot.lateral.tests.test_lane_centering import model
from openpilot.starpilot.longitudinal.tests.extension_helpers import attach_inputs
from openpilot.starpilot.longitudinal.lane_change_gap import LaneChangePace
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR
from opendbc.car.vehicle_model import VehicleModel


def host(policy):
  c = Controls.__new__(Controls)
  c.lateral_pause = LateralPause(PauseSettings())
  attach_inputs(c)
  c.lateral_gain_owner = None
  c.aol_replay = False
  interface = interfaces[CAR.TOYOTA_COROLLA_TSS2]
  c.CP = interface.get_non_essential_params(CAR.TOYOTA_COROLLA_TSS2).as_reader()
  c.CI = interface(c.CP)
  c.VM = VehicleModel(c.CP)
  c.LaC = LatControlTorque(c.CP, c.CI, DT_CTRL)
  c.LoC = LongControl(c.CP)
  c.sm = messaging.SubMaster(["vehicleParameters", "lateralTorqueParameters", "carState", "selfdriveState",
                             "longitudinalPlan", "modelV2", "lateralManeuverPlan", "lateralDelay",
                             "onroadEvents", "radarState"], frequency=100)
  c.curvature = c.desired_curvature = 0.0
  c.steer_limited_by_safety = False
  c.lane_centering_controller = LaneCenteringController()
  c.params = Params()
  c.lane_change_preferences = SimpleNamespace(sample=lambda *a, **kw: policy)
  c.lane_change_policy = policy
  c.lane_change_smoother = LaneChangeSmoother()
  c.lane_change_pace = LaneChangePace()
  c.torque_host = None
  c.torque_learning_allowed = False
  for name in ('ioniq6_start_enabled', 'gm_start_enabled', 'gm_volt_enabled', 'toyota_sienna_replay'):
    setattr(c.longitudinal_inputs, name, False)
  c.lane_centering_host = None
  c.last_lane_centering_result = None
  return c


class NativePaceTests(unittest.TestCase):
  def test_temporary_ten_matches_existing_ten_through_native_controls(self):
    policy = LaneChangePolicy(close_gap=True, close_gap_fast_pace=True)
    controls = [host(policy), host(replace(policy, close_gap_fast_pace=False, duration_s=3.)),
                host(replace(policy, close_gap_fast_pace=False))]
    comparisons = []
    with patch.dict(os.environ, REPLAY='1'):
      for tick in range(150):
        timestamp = 1_000_000_000 + tick * 10_000_000
        inputs = []
        def message(name, size=None):
          event = messaging.new_message(name, size, valid=True, logMonoTime=timestamp)
          inputs.append(event)
          return getattr(event, name)
        cs = message('carState')
        cs.vEgo = cs.vEgoRaw = 20.
        cs.vCruise = cs.vCruiseCluster = 100.
        cs.canValid = True
        cs.leftBlinker = tick < 130
        state = message('selfdriveState')
        state.enabled = state.active = True
        state.state = 'enabled'
        lp = message('vehicleParameters')
        lp.stiffnessFactor, lp.steerRatio = 1., controls[0].CP.steerRatio
        if tick % 5 == 0:
          message('longitudinalPlan').aTarget = 0.
        message('lateralDelay').lateralDelay = .1
        message('onroadEvents', 0)
        if tick % 5 == 0:
          md = message('modelV2')
          source = model()
          md.laneLines = source.laneLines
          md.roadEdges = source.roadEdges
          md.laneLineProbs = source.laneLineProbs
          md.laneLineStds = source.laneLineStds
          md.roadEdgeStds = source.roadEdgeStds
          fill_adjacent_geometry(md)
          md.action.desiredCurvature = 0. if tick < 80 else .003
          md.meta.laneChangeState = 'preLaneChange' if tick < 80 else ('laneChangeStarting' if tick < 130 else 'off')
          md.meta.laneChangeDirection = 'left' if tick < 130 else 'none'
          radar = message('radarState')
          radar.leadOne.present = True
          radar.leadOne.dRel = 33.
          radar.leadOne.vRel = -1.
          radar.leadOne.vLead = radar.leadOne.vLeadK = 19.
        readers = [event.as_reader() for event in inputs]
        commands = []
        for control in controls:
          control.sm.update_msgs(timestamp / 1e9, readers)
          command, _ = control.state_control()
          commands.append(command)
        if 85 <= tick < 130:
          from openpilot.starpilot.longitudinal.lane_change_gap import project
          self.assertTrue(controls[0].lane_change_pace.selected, repr((tick, commands[0].longActive, commands[0].latActive,
            controls[0].CP.openpilotLongitudinalControl, project(controls[0].sm, timestamp, pace=True),
            controls[0].lane_change_pace.__dict__, controls[0].sm.freq_ok)))
          self.assertEqual(commands[0].actuators.curvature, commands[1].actuators.curvature)
          self.assertEqual(commands[0].latActive, commands[1].latActive)
          comparisons.append(abs(commands[0].actuators.curvature - commands[2].actuators.curvature))
        if tick >= 130:
          self.assertFalse(controls[0].lane_change_pace.selected)
    self.assertTrue(comparisons)
    self.assertGreater(max(comparisons), 0.)
    self.assertEqual(controls[0].lane_change_policy.duration_s, policy.duration_s)
