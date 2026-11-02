import math
import unittest
from dataclasses import replace
from unittest.mock import patch
from types import SimpleNamespace as NS

from openpilot.starpilot.curve_speed.runtime import Runtime
from openpilot.starpilot.curve_speed.tests.test_runtime import frame
from openpilot.starpilot.curve_speed.tests.test_host import Bus, sample_messages, NOW
from openpilot.starpilot.curve_speed.host import CurveHost
from openpilot.starpilot.lead_tracking import LeadDetector


class CeilingHysteresisTests(unittest.TestCase):
  def step(self, rt, n, target, **kwargs):
    item = frame(n, **kwargs)
    def target_step(*args, **kw):
      rt.filter.target = target
      return target
    with patch.object(rt.filter, 'step', side_effect=target_step):
      result = rt.step(item)
    rt.confirm_applied(item.now_ns, applied=result.ceiling_mps is not None)
    return result.ceiling_mps

  def test_threshold_chatter_is_one_entry_without_freezing_target(self):
    rt = Runtime(enabled=True)
    self.assertIsNone(self.step(rt, 1, 29.6))
    self.assertEqual(self.step(rt, 2, 29.4), 29.4)
    for n in range(3, 25):
      target = 29.45 if n % 2 else 29.65
      self.assertEqual(self.step(rt, n, target), target)

  def test_exit_needs_consecutive_fresh_confirmation(self):
    rt = Runtime(enabled=True)
    self.step(rt, 1, 29.4)
    for n in range(2, 8):
      self.assertEqual(self.step(rt, n, 29.9), 29.9)
    self.assertIsNone(self.step(rt, 8, 29.9, inputs_fresh=False))
    for n in range(9, 16):
      self.assertEqual(self.step(rt, n, 29.9), 29.9)
    self.assertIsNone(self.step(rt, 16, 29.9))

  def test_driver_and_invalid_gates_clear_latch(self):
    for kwargs in ({'gas_pressed':True}, {'brake_pressed':True}, {'enabled':False},
                   {'long_active':False}, {'force_decel':True}, {'following_lead':None},
                   {'following_lead':True}, {'ego_mps':0.0}, {'cruise_mps':float('nan')}):
      with self.subTest(kwargs=kwargs):
        rt=Runtime(enabled=True,no_lead=True)
        self.step(rt,1,29.4)
        self.assertIsNone(self.step(rt,2,29.8,**kwargs))
        self.assertFalse(rt.cap_latched)

  def test_gap_and_drive_change_clear_state(self):
    rt=Runtime(enabled=True)
    self.step(rt,1,29.4)
    self.assertIsNone(self.step(rt,10,29.8))
    self.step(rt,11,29.4)
    self.assertIsNone(self.step(rt,12,29.8,drive_id=2))

  def test_lower_cruise_not_overridden_and_learning_not_manufactured(self):
    rt=Runtime(enabled=True)
    self.step(rt,1,29.4)
    for n in range(2,10):
      self.assertEqual(self.step(rt,n,19.7,cruise_mps=20),19.7)
    self.assertEqual(rt.curve.revision,0)
    rt.confirm_applied(frame(9).now_ns,applied=False)
    self.assertFalse(rt.was_controlling)


class AbsentLeadPathTests(unittest.TestCase):
  def sample(self, host, sm, n):
    now=NOW+n*50_000_000
    sm.logMonoTime=dict.fromkeys(sm.messages,now)
    return host.sample(sm,NS(openpilotLongitudinalControl=True),now,follow_time_s=1.45)

  def test_recorded_wide_bend_stays_eligible_only_for_absent_lead(self):
    sm=Bus(sample_messages());sm.messages['modelV2'].position.y=[-52.59375*i/32 for i in range(33)]
    host=CurveHost(enabled=True,no_lead=True,replay=True)
    self.assertIsNotNone(self.sample(host,sm,0).ceiling_mps)
    sm.messages['radarState'].leadOne.present=True
    self.assertIsNone(self.sample(host,sm,1).ceiling_mps)
    self.assertEqual(host.runtime.eligibility,'following_unknown')

  def test_malformed_geometry_and_stale_radar_remain_unknown(self):
    for ys in ([0.0], [float('nan')]*33, [math.inf]*33):
      sm=Bus(sample_messages());sm.messages['modelV2'].position.y=ys
      host=CurveHost(enabled=True,no_lead=True,replay=True)
      self.assertIsNone(self.sample(host,sm,0).ceiling_mps)
    sm=Bus(sample_messages());sm.messages['modelV2'].position.x[8]=0.
    self.assertIsNone(self.sample(CurveHost(enabled=True,no_lead=True,replay=True),sm,0).ceiling_mps)
    sm=Bus(sample_messages());sm.valid['radarState']=False
    self.assertIsNone(self.sample(CurveHost(enabled=True,no_lead=True,replay=True),sm,0).ceiling_mps)

  def test_other_callers_keep_strict_geometry_and_lead_decay_is_retained(self):
    model=sample_messages()['modelV2'];lead=sample_messages()['radarState'].leadOne
    lead.present=True
    det=LeadDetector(allow_absent_path_offsets=True)
    def step(owner,n):
      return owner.step(lead,model,speed_mps=25.,t_follow_s=1.45,standstill=False,
                        observed_mono_s=10+n*.05,now_mono_s=10+n*.05,
                        radar_fresh=True,model_fresh=True,car_fresh=True,headway_fresh=True)
    for n in range(30):obs=step(det,n)
    self.assertTrue(obs.following)
    lead.present=False;model.position.y=[-60*i/32 for i in range(33)]
    self.assertIsNone(step(LeadDetector(),30).following)
    self.assertTrue(step(det,30).following)
    for n in range(31,60):obs=step(det,n)
    self.assertFalse(obs.following)


class DiagnosticsTests(unittest.TestCase):
  def test_bounded_logging_retains_pending_off_edge(self):
    from openpilot.starpilot.curve_speed.diagnostics import SmoothnessDiagnostics
    rt=Runtime(enabled=True)
    recovery=NS(armed=False,applied=False,reason='inactive',limit=None)
    planner=NS(mode_recovery=recovery,last_curve_ceiling_applied=False,output_a_target=0.)
    diag=SmoothnessDiagnostics()
    with patch('openpilot.starpilot.curve_speed.diagnostics.cloudlog.event') as emit:
      diag.update(1_000_000_000,rt,planner)
      diag.update(1_000_000_000,rt,planner)
      rt.enabled=False
      diag.update(1_050_000_000,rt,planner)
      self.assertEqual(emit.call_count,1)
      diag.update(2_000_000_000,rt,planner)
      self.assertEqual(emit.call_count,2)
      diag.update(3_000_000_000,rt,planner)
      self.assertEqual(emit.call_count,2)
