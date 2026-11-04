"""End-to-end admission hold: short Experimental proposals cannot steer the arbitration (3 Nov 2026 review)."""
import unittest
from unittest.mock import patch

from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, E2E_ADMIT_CYCLES
from openpilot.starpilot.longitudinal.tests.test_conditional_handoff import Frame, NOW, DRIVE


def planner():
  cp = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  cp.openpilotLongitudinalControl = True
  return LongitudinalPlanner(cp, init_v=20.)


def run(p, modes, model_accel=-0.6, v_ego=20., model_stop=False, start=0):
  """Step the real planner through a mode sequence with the MPC pinned at +0.5; return the final requests."""
  outputs = []
  with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.get_accel_from_plan', return_value=.5):
    for n, experimental in enumerate(modes, start=start):
      sm = Frame()
      now = NOW + n * 50_000_000
      sm.logMonoTime = dict.fromkeys(sm, now - 5_000_000)
      sm['carState'].vEgo = sm['carState'].vEgoCluster = v_ego
      sm['carState'].vCruise = 108.
      sm['selfdriveState'].experimentalMode = experimental
      sm['modelV2'].action.desiredAcceleration = model_accel
      sm['modelV2'].action.shouldStop = model_stop
      p.update(sm, now_ns=now, drive_id=DRIVE)
      outputs.append(float(p.output_a_target))
  return outputs


class AdmissionTests(unittest.TestCase):
  def test_first_cycle_adopts_the_published_mode(self):
    p = planner()
    out = run(p, [True])
    self.assertTrue(p.e2e_admitted)
    self.assertAlmostEqual(out[0], -0.6, places=5)

  def test_short_proposal_is_held_and_logged(self):
    p = planner()
    run(p, [False] * 5)
    with patch('openpilot.selfdrive.controls.lib.longitudinal_planner.cloudlog') as log:
      out = run(p, [True] * (E2E_ADMIT_CYCLES - 1) + [False], start=5)
      self.assertFalse(p.e2e_admitted)
      self.assertTrue(all(o > 0. for o in out), out)  # the -0.6 model request never reached the output
      log.event.assert_called_once()
      self.assertEqual(log.event.call_args.args[0], 'longitudinal.e2e_blip_held')
      self.assertEqual(log.event.call_args.kwargs['cycles'], E2E_ADMIT_CYCLES - 1)

  def test_sustained_proposal_is_admitted_after_the_hold(self):
    p = planner()
    run(p, [False] * 5)
    out = run(p, [True] * (E2E_ADMIT_CYCLES + 2), start=5)
    self.assertTrue(p.e2e_admitted)
    self.assertTrue(all(o > 0. for o in out[:E2E_ADMIT_CYCLES - 1]), out)
    self.assertAlmostEqual(out[-1], -0.6, places=5)

  def test_hard_model_braking_stop_and_launch_pass_at_once(self):
    for kwargs in (dict(model_accel=-2.0), dict(model_stop=True), dict(v_ego=1.0)):
      with self.subTest(kwargs=kwargs):
        p = planner()
        run(p, [False] * 5)
        run(p, [True], start=5, **kwargs)
        self.assertTrue(p.e2e_admitted)

  def test_leaving_experimental_releases_the_model_at_once(self):
    p = planner()
    run(p, [True] * 3)
    out = run(p, [False], start=3)
    self.assertFalse(p.e2e_admitted)
    self.assertGreater(out[0], -0.6)


if __name__ == '__main__':
  unittest.main()
