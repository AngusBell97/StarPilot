from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner.controller import Coordinator, approach_speed
from openpilot.starpilot.controls.lib.map_corner.types import Result, PointCap

def run(c,now=1,**kw):
  args=dict(now=now,speed=20,acceleration=0,cruise=25,visual=25,visual_valid=False,other=25,
            automated=True,parent=True,learn=True,available=True,button=False,gas=False,brake=False,
            cancel=False,confounded=False,fault=False,result=None)
  args.update(kw)
  return c.step(**args)

def result(c,now,**kw):
  args=dict(now=now,sequence=int(now*20),control_epoch=c.control_epoch,odometer=c.odometer,key='bend',
            progress=120,exit=200,points=(PointCap(0,10,10,ceiling=(10)*(4/1.8)**0.5),),status='Map corner',uncertainty=0)
  args.update(kw)
  return Result(**args)

def test_default_off_does_not_limit_or_suppress_training():
  c=Coordinator()
  d=run(c,parent=False)
  assert d.cap==0 and not d.suppress_training

def test_lower_map_target_below_visual_floor_and_other_constraints_still_win():
  c=Coordinator();run(c)
  for i in range(1,401):
    now=1+i*.05
    d=run(c,now,result=result(c,now),visual=11.176)
  assert d.cap==pytest.approx(10)
  assert min(d.cap,8,11.176)==8
  assert d.owner=='map'
  assert d.suppress_training

def test_faster_recall_without_valid_visual_uses_prior_ceiling():
  c=Coordinator();run(c)
  for i in range(1,301):
    now=1+i*.05
    d=run(c,now,result=result(c,now,points=(PointCap(0,18,12,ceiling=(12)*(4/1.8)**0.5),)),visual=14,visual_valid=False)
  assert d.cap==pytest.approx(12)

def test_explicit_map_skip_leaves_visual_constraint():
  c=Coordinator();run(c)
  for i in range(1,301):
    now=1+i*.05
    d=run(c,now,result=result(c,now),visual=15)
  assert d.owner=='map'
  d=run(c,now+.05,result=result(c,now+.05),visual=15,button=True)
  assert d.cap==0
  assert d.skip_key=='bend'
  assert min(25,15)==15

def test_stale_or_wrong_epoch_result_cannot_acquire_map_authority():
  c=Coordinator();run(c)
  assert run(c,2,result=result(c,1)).cap==0
  assert run(c,2.05,result=replace(result(c,2.05),control_epoch=999)).cap==0

def test_map_influence_excludes_global_training_after_takeover_and_learning_off():
  c=Coordinator();run(c)
  for i in range(1,301):
    now=1+i*.05
    run(c,now,result=result(c,now),learn=False)
  d=run(c,now+.05,automated=False,brake=True,learn=False,result=result(c,now+.05))
  assert d.suppress_training and d.cap==0
  assert not d.visual_nudge

def test_approach_distance_changes_envelope_and_positive_accel_needs_more_room():
  assert approach_speed(10,0,20,0,0)==10
  assert approach_speed(10,200,20,0,0)>10
  assert approach_speed(10,200,20,1,0)<approach_speed(10,200,20,0,0)

def test_gas_takeover_skips_using_previous_epoch_identity():
  c=Coordinator();run(c)
  for i in range(1,301):
    now=1+i*.05;run(c,now,result=result(c,now))
  d=run(c,now+.05,automated=False,gas=True,result=result(c,now+.05))
  assert d.skip_key=='bend' and d.intervention and d.cap==0

def test_no_lead_or_indicator_gate_removes_map_candidate():
  c=Coordinator();run(c)
  for i in range(1,301):
    now=1+i*.05;run(c,now,result=result(c,now))
  assert run(c,now+.05,available=False,result=result(c,now+.05)).cap==0

def test_loss_of_visual_confirmation_cannot_retain_faster_memory_cap():
  c=Coordinator();baseline=Coordinator();run(c);run(baseline)
  for i in range(1,401):
    now=1+i*.05
    run(c,now,visual=22,visual_valid=True,result=result(c,now,points=(PointCap(0,17,12,ceiling=(12)*(4/1.8)**0.5),)))
    base=run(baseline,now,result=result(baseline,now,points=(PointCap(0,12,12,ceiling=(12)*(4/1.8)**0.5),)))
  assert c.cap>base.cap
  changed=run(c,now+.05,visual_valid=False,result=result(c,now+.05,points=(PointCap(0,17,12,ceiling=(12)*(4/1.8)**0.5),)))
  assert changed.cap<=base.cap

def test_skip_for_one_corner_does_not_skip_following_corner():
  c=Coordinator();run(c)
  for i in range(1,301):
    now=1+i*.05;run(c,now,result=result(c,now))
  run(c,now+.05,button=True,result=result(c,now+.05))
  assert c.skip_key=='bend'
  d=run(c,now+.1,result=result(c,now+.1,key='second'))
  assert not d.skip_key and d.cap>0

def test_skip_is_filtered_by_binding_bend_while_another_bend_remains_in_result():
  c=Coordinator();run(c)
  points=(PointCap(0,15,15,'first',ceiling=(15)*(4/1.8)**0.5),PointCap(10,10,10,'second',ceiling=(10)*(4/1.8)**0.5))
  for i in range(1,301):
    now=1+i*.05;run(c,now,result=result(c,now,key='first',points=points))
  assert c.binding_key=='second'
  d=run(c,now+.05,button=True,result=result(c,now+.05,key='first',points=points))
  assert d.skip_key=='second' and d.cap==0
  d=run(c,now+.1,result=result(c,now+.1,key='first',points=points))
  assert d.skip_key=='second' and d.cap>=15

def test_manual_throttle_does_not_skip_corner_on_later_engagement():
  c=Coordinator();run(c,automated=False)
  d=run(c,1.05,automated=False,gas=True,result=result(c,1.05))
  assert not d.skip_key
  run(c,1.1,automated=True,result=result(c,1.1))
  d=run(c,1.15,automated=True,result=result(c,1.15))
  assert not d.skip_key and d.cap>0
