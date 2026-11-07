import math
import unittest
from dataclasses import replace
from unittest.mock import patch

from openpilot.starpilot.longitudinal.mode_recovery import ModeRecovery, RecoveryFrame, project
from openpilot.starpilot.longitudinal.tests.test_conditional_handoff import Frame, NOW, DRIVE
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


def frame(n, experimental=False):
  return RecoveryFrame(10_000_000_000+n*50_000_000,9_997_000_000+n*50_000_000,1,experimental)


class RecoveryTests(unittest.TestCase):
  def step(self,g,n,prev,target,experimental=False,**kw):
    return g.step(frame(n,experimental),previous_target=prev,target=target,positive_jerk=1.,**kw)

  def test_recorded_step_recovers_without_timer_jump(self):
    g=ModeRecovery();self.step(g,0,-1.37,-1.37,True);last=-1.37
    for n in range(1,30):
      val=self.step(g,n,last,.145)
      self.assertLessEqual(val,.145)
      self.assertLessEqual(val-last,.2+1e-9)
      if last>=0:self.assertLessEqual(val-last,.05+1e-9)
      last=val
    self.assertAlmostEqual(last,.145)
    self.assertFalse(g.armed)

  def test_zero_crossing_splits_remaining_frame(self):
    g=ModeRecovery();self.step(g,0,-.05,-.05,True)
    self.assertAlmostEqual(self.step(g,1,-.05,1.),.0375)

  def test_new_stronger_braking_passes_immediately(self):
    g=ModeRecovery();self.step(g,0,-1.,-1.,True)
    self.step(g,1,-1.,1.)
    self.assertEqual(self.step(g,2,-.875,-2.4),-2.4)
    self.assertFalse(g.armed)

  def test_each_stronger_final_modifier_is_retained(self):
    for target in (-3.5,-2.4,-1.2,-.9):
      g=ModeRecovery();self.step(g,0,-.5,-.5,True)
      self.assertEqual(self.step(g,1,-.5,target),target)
      self.assertFalse(g.applied)

  def test_no_exit_no_shaping_and_nonbinding_exit_identical(self):
    g=ModeRecovery()
    for n in range(10):self.assertEqual(self.step(g,n,0.,1.),1.)
    self.step(g,10,0.,0.,True)
    self.assertEqual(self.step(g,11,0.,-.5),-.5)

  def test_gap_duplicate_drive_change_and_blockers_clear(self):
    for make in (lambda:frame(1),lambda:frame(8),lambda:replace(frame(2),drive_id=2),lambda:None):
      g=ModeRecovery();self.step(g,0,-1.,-1.,True);self.step(g,1,-1.,1.)
      self.assertEqual(g.step(make(),previous_target=-.875,target=1.,positive_jerk=1.),1.)
      self.assertFalse(g.armed)
    g=ModeRecovery();self.step(g,0,-1.,-1.,True)
    self.assertEqual(self.step(g,1,-1.,1.,blocked=True),1.)
    self.assertFalse(g.armed)

  def test_reentry_passes_stronger_model_braking(self):
    g=ModeRecovery();self.step(g,0,-1.,-1.,True);self.step(g,1,-1.,1.)
    self.assertEqual(self.step(g,2,-.875,-2.,True),-2.)

  def test_reentry_with_higher_model_request_cannot_create_release_step(self):
    g=ModeRecovery();self.step(g,0,-1.,-1.,True);self.step(g,1,-1.,1.)
    self.assertAlmostEqual(self.step(g,2,-.875,.5,True),-.675)
    self.assertTrue(g.applied)
    self.assertEqual(self.step(g,3,-.75,-2.,True),-2.)
    self.assertFalse(g.armed)

  def test_projection_rejects_driver_and_stale_inputs(self):
    cp=CarInterface.get_non_essential_params(CAR.HYUNDAI_IONIQ_6);cp.openpilotLongitudinalControl=True
    for field in ('gasPressed','brakePressed','standstill','canTimeout'):
      sm=Frame();setattr(sm['carState'],field,True)
      self.assertIsNone(project(sm,cp,NOW,DRIVE))
    sm=Frame();self.assertIsNotNone(project(sm,cp,NOW,DRIVE))
    sm.logMonoTime['modelV2']=NOW-151_000_000
    self.assertIsNone(project(sm,cp,NOW,DRIVE))

  def test_real_planner_final_output_and_attribution(self):
    cp=CarInterface.get_non_essential_params(CAR.HYUNDAI_IONIQ_6);cp.openpilotLongitudinalControl=True
    sm=Frame();sm['modelV2'].action.desiredAcceleration=-1.37;sm['selfdriveState'].experimentalMode=True
    planner=LongitudinalPlanner(cp,init_v=20.)
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_accel_from_plan',return_value=.5):
      planner.update(sm,now_ns=NOW,drive_id=DRIVE)
      self.assertAlmostEqual(planner.output_a_target,-1.37,places=5)
      sm['selfdriveState'].experimentalMode=False
      sm.logMonoTime=dict.fromkeys(sm,NOW+45_000_000)
      planner.update(sm,now_ns=NOW+50_000_000,drive_id=DRIVE)
      # 7 Nov 2026: leaving Experimental fades the model out over E2E_BLEND_OUT_S, which is gentler than the
      # recovery ramp (-1.17 after one cycle), so the ramp no longer has to act.
      self.assertFalse(planner.mode_recovery.applied)
      self.assertLess(planner.output_a_target,-1.17)
      self.assertGreater(planner.output_a_target,-1.37)
      self.assertFalse(planner.last_curve_ceiling_applied)
      sm['modelV2'].action.desiredAcceleration=-2.0;sm['selfdriveState'].experimentalMode=True
      sm.logMonoTime=dict.fromkeys(sm,NOW+95_000_000)
      planner.update(sm,now_ns=NOW+100_000_000,drive_id=DRIVE)
      self.assertAlmostEqual(planner.output_a_target,-2.0,places=5)
