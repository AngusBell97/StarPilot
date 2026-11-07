"""End-to-end admission hold (3 Nov 2026) and mode-change blend (7 Nov 2026)."""
import unittest
from unittest.mock import patch

from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR
from openpilot.selfdrive.controls.lib.longitudinal_planner import (LongitudinalPlanner, E2E_ADMIT_CYCLES, E2E_BLEND_IN_S,
                                                                    E2E_BLEND_OUT_S)
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

  def test_sustained_proposal_is_admitted_after_the_hold_then_blended_in(self):
    p = planner()
    run(p, [False] * 5)
    blend = round(E2E_BLEND_IN_S / 0.05)
    out = run(p, [True] * (E2E_ADMIT_CYCLES + blend + 2), start=5)
    self.assertTrue(p.e2e_admitted)
    self.assertTrue(all(o > 0. for o in out[:E2E_ADMIT_CYCLES - 1]), out)
    # The model's lower request fades in: no single cycle drops by more than a small step, and it arrives.
    steps = [a - b for a, b in zip(out, out[1:], strict=False)]
    self.assertLess(max(steps), 0.25, out)
    self.assertAlmostEqual(out[-1], -0.6, places=5)

  def test_hard_model_braking_stop_and_launch_pass_at_once(self):
    for kwargs in (dict(model_accel=-2.0), dict(model_stop=True), dict(v_ego=1.0)):
      with self.subTest(kwargs=kwargs):
        p = planner()
        run(p, [False] * 5)
        run(p, [True], start=5, **kwargs)
        self.assertTrue(p.e2e_admitted)

  def test_leaving_experimental_fades_the_model_out(self):
    p = planner()
    run(p, [True] * 3)
    out = run(p, [False] * (round(E2E_BLEND_OUT_S / 0.05) + 20), start=3)
    self.assertFalse(p.e2e_admitted)
    self.assertGreater(out[0], -0.6)
    self.assertLess(out[0], -0.5)  # one cycle in, the model still holds almost all of its restraint
    self.assertGreater(out[-1], 0.)  # and is gone once the fade completes

  def test_urgent_model_braking_skips_the_blend(self):
    p = planner()
    run(p, [False] * 5)
    out = run(p, [True] * E2E_ADMIT_CYCLES, start=5, model_accel=-1.2)
    self.assertTrue(p.e2e_admitted)  # admitted after the usual hold (immediate only at -1.5)...
    self.assertAlmostEqual(p.e2e_weight, 1.0)  # ...then -1.2 applies in full, without the fade
    self.assertAlmostEqual(out[-1], -1.2, places=5)

  def test_cruise_limits_blend_between_modes(self):
    from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
    cp = planner().CP
    args = (20., 5., 0., 30., cp, 0.05, -1., True)
    chill = get_cruise_accel(False, *args, e2e_weight=0.)
    exp = get_cruise_accel(True, *args, e2e_weight=1.)
    half = get_cruise_accel(True, *args, e2e_weight=0.5)
    self.assertLessEqual(min(chill, exp) - 1e-9, half)
    self.assertLessEqual(half, max(chill, exp) + 1e-9)


if __name__ == '__main__':
  unittest.main()
