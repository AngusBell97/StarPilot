from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.controls.lib.map_corner.worker import Engine,visual_agrees
from openpilot.starpilot.controls.lib.map_corner.types import Corner,Fix,Frame,Match
from openpilot.starpilot.controls.lib.map_corner.store import CornerStore

def corner(): return Corner('bend',100,1,'r',300,350,400,(13,13,13),((0,0),(10,2)),90,curvature=tuple(1.8/(v*v) for v in (13,13,13)))
def frame(**kw):
  values=dict(now=1,sequence=1,control_epoch=1,observation_fence=1,fix=Fix(1,1,51,0,90,1,20,True),
              speed=20,acceleration=0,odometer=0,automated=True,learn=True,owner='map',intervention=False,
              observation_id='',confounded=False,fault=False,visual_valid=True,clear_conditions=True,
              visual_profile=((350,.01065,True),))
  values.update(kw);return Frame(**values)

class Provider:
  status='Map ready'
  def update(self,f,now): return Match(100,1,0,0,11,(corner(),),'local')

def test_worker_acquires_early_corner_and_freezes_recall_for_current_pass(tmp_path):
  with CornerStore(tmp_path) as store:
    e=Engine(Provider(),store)
    r=e.step(frame())
    assert r.points and r.points[0].speed==13
    store.observe(corner(),(11,11,11),'one',False)
    r=e.step(frame(now=1.05,sequence=2,fix=replace(frame().fix,received_at=1.05,identity=2)))
    assert r.points[0].speed==13

def test_visual_evidence_requires_signed_aligned_unclamped_point():
  c=corner();f=frame()
  assert visual_agrees(f,c,0)
  assert not visual_agrees(replace(f,visual_profile=((350,.01065,False),)),c,0)
  assert not visual_agrees(replace(f,visual_profile=((350,-.01065,True),)),c,0)
  assert not visual_agrees(replace(f,visual_profile=((100,.01065,True),)),c,0)

def test_late_acquisition_generates_no_map_target():
  p=Provider()
  p.update=lambda f,now: Match(100,1,290,0,11,(corner(),),'local')
  e=Engine(p)
  assert not e.step(frame()).points
  assert e.stats['late']==1

def test_cancelled_transaction_does_not_resurrect_learning(tmp_path):
  with CornerStore(tmp_path) as s:
    with pytest.raises(ValueError): s.observe(corner(),(11,11,11),'one',False,valid=lambda:False)
    assert s.stats()['corners']==0

def test_visual_confirmation_for_first_bend_cannot_release_faster_memory_on_later_bend():
  first=corner()
  later=replace(first,key='later',entry=500,apex=550,exit=600)
  p=Provider();p.update=lambda f,now:Match(100,1,0,0,11,(first,later),'local')
  store=SimpleNamespace(lookup=lambda c:SimpleNamespace(profile=(15,15,15)),mark_map=lambda *args:None)
  r=Engine(p,store).step(frame())
  assert r.visual_confirmed
  assert r.points[0].speed==15
  assert all(point.speed<=point.prior for point in r.points[3:])

def test_anticipated_second_bend_keeps_control_when_it_becomes_primary():
  first=corner();later=replace(first,key='later',entry=480,apex=530,exit=580,prior=(16,10,14))
  p=Provider();p.update=lambda f,now:Match(100,1,0,0,11,(first,later),'local')
  e=Engine(p)
  assert len(e.step(frame()).points)==6
  p.update=lambda f,now:Match(100,1,420,0,11,(first,later),'local')
  r=e.step(frame(now=1.05,sequence=2,speed=16,fix=replace(frame().fix,received_at=1.05,identity=2)))
  assert r.key=='later' and r.points

def test_later_binding_corner_does_not_teach_nearer_corner(tmp_path):
  from test_map_corner_learning import passage
  p=Provider();e=Engine(p)
  e.step(frame(binding_key='later'))
  assert e.observer.history[-1][1]=='other'
  e.step(frame(now=1.05,sequence=2,binding_key='bend',fix=replace(frame().fix,received_at=1.05,identity=2)))
  assert e.observer.history[-1][1]=='map'
