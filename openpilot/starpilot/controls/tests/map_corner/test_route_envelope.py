"""The fastest plausible continuation governs; compare approach envelopes."""
import math
from types import SimpleNamespace
from openpilot.starpilot.controls.lib.map_corner.controller import Coordinator
from openpilot.starpilot.controls.lib.map_corner.types import PointCap

def decision(options,visual=30):
  co=Coordinator();co.previous_auto=True;co.previous_parent=True
  result=SimpleNamespace(now=1,sequence=1,control_epoch=0,odometer=1,key='route',progress=0,exit=300,
    points=(),route_options=options,status='Map corner',remembered=False,uncertainty=0,visual_confirmed=False)
  return co.step(now=1,speed=20,acceleration=-.6,cruise=30,visual=visual,visual_valid=False,other=30,
    automated=True,parent=True,learn=True,available=True,button=False,gas=False,brake=False,cancel=False,
    confounded=False,fault=False,result=result)

def test_straight_continuation_prevents_slowdown_for_a_side_turn():
  d=decision(((PointCap(100,10,10,ceiling=(10)*(4/1.8)**0.5),),()))
  assert d.cap==0

def test_all_slow_options_use_highest_approach_speed():
  d=decision(((PointCap(100,10,10,ceiling=(10)*(4/1.8)**0.5),),(PointCap(100,20,20,ceiling=(20)*(4/1.8)**0.5),)))
  assert math.isclose(d.cap,math.sqrt(520),rel_tol=1e-6)

def test_compare_full_approaches_not_only_corner_apex_speeds():
  d=decision(((PointCap(400,10,10,ceiling=(10)*(4/1.8)**0.5),),(PointCap(20,20,20,ceiling=(20)*(4/1.8)**0.5),)))
  assert math.isclose(d.cap,math.sqrt(580),rel_tol=1e-6)

def test_unknown_continuation_cannot_be_dropped_from_maximum():
  assert decision(((PointCap(20,10,10,ceiling=(10)*(4/1.8)**0.5),),(),(PointCap(20,12,12,ceiling=(12)*(4/1.8)**0.5),))).cap==0

def test_aggregate_prediction_cannot_bind_learning_to_an_unchosen_exit():
  d=decision(((PointCap(100,10,10,'left',ceiling=(10)*(4/1.8)**0.5),),(PointCap(100,20,20,'right',ceiling=(20)*(4/1.8)**0.5),)))
  assert d.binding_key==''

def test_overflow_retains_the_permissive_alternative_and_common_prefix():
  from openpilot.starpilot.controls.lib.map_corner.controller import route_envelope
  slow=PointCap(0,10,10,'slow',ceiling=(10)*(4/1.8)**0.5);common=PointCap(100,20,20,'common',ceiling=(20)*(4/1.8)**0.5)
  for options in ((tuple([slow]*129),()),tuple([(slow,)]*32+[()])):
    raw,_,_,_=route_envelope((),options,cruise=30,speed=20,acceleration=-.6,uncertainty=0)
    assert raw==30
    raw,_,_,_=route_envelope((common,),options,cruise=30,speed=20,acceleration=-.6,uncertainty=0)
    assert math.isclose(raw,math.sqrt(520))
  raw,_,_,_=route_envelope(tuple([common]*25),((slow,),),cruise=30,speed=20,acceleration=-.6,uncertainty=0)
  assert math.isclose(raw,math.sqrt(520))

def test_aggregate_bypass_survives_preview_key_changes_and_expires():
  co=Coordinator();co.previous_auto=True;co.previous_parent=True
  def step(t,key,button=False):
    result=SimpleNamespace(now=t,sequence=round(t*20),control_epoch=co.control_epoch,odometer=co.odometer,
      key=key,progress=0,exit=300,points=(),route_options=((PointCap(100,10,10,'left',ceiling=(10)*(4/1.8)**0.5),),),
      status='Map corner',remembered=False,uncertainty=0,visual_confirmed=False)
    return co.step(now=t,speed=10,acceleration=-.6,cruise=30,visual=12,visual_valid=False,other=30,
      automated=True,parent=True,learn=True,available=True,button=button,gas=False,brake=False,cancel=False,
      confounded=False,fault=False,result=result)
  step(1,'a');d=step(1.05,'a',True)
  assert d.skip_key=='@routes' and d.cap==0
  for i in range(2,901):
    d=step(1+i*.05,'b'+str(i))
    assert d.cap==0
  assert step(46.1,'c').cap>0
