from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.controls.lib.map_corner.types import Way,Fix,Frame,Match,Corner,PointCap
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.worker import Engine
from openpilot.starpilot.controls.lib.map_corner.provider import MatchTracker
from test_map_corner_provider import ll,fix,bend_way

def frame(t=1,**changes):
  base=Frame(t,round(t*20),0,0,fix(t),5,0,0,True,True,'other',False,'',False,False,False,False)
  return replace(base,**changes)

def test_unique_connected_way_handoff_does_not_restart_qualification():
  a=Way(1,tuple(ll(x,0) for x in range(-400,1,10)),False,'secondary','')
  b=Way(2,tuple(ll(x,0) for x in range(0,1001,10)),False,'secondary','')
  tile=G.build_tile((a,b),'r');tracker=MatchTracker();results=[]
  for i in range(191):
    match=tracker.update(tile,fix(1+i*.1,x=-350+2*i))
    if i>=100:results.append(match)
  assert all(results) and results[-1].way_id==2

def test_per_way_corner_beyond_a_fork_is_not_an_unconditional_target_or_lookup():
  road=bend_way();branch=Way(2,tuple(ll(x,0) for x in range(0,401,10)),False,'secondary','')
  tile=G.build_tile((road,branch),'r');match=G.find_match(tile,fix())
  calls=[]
  store=SimpleNamespace(lookup=lambda c:calls.append(c.key))
  provider=SimpleNamespace(tile=tile,status='Map ready',update=lambda f,n:match)
  result=Engine(provider,store).step(frame())
  assert result.points==() and calls==[]
  assert result.route_options and any(not path for path in result.route_options)

def test_shared_profile_targets_are_at_bin_starts():
  c=Corner('c',1,1,'r',300,345,390,(10,11,12),((0,0),(1,1)),0,curvature=tuple(1.8/(v*v) for v in (10,11,12)))
  provider=SimpleNamespace(status='Map ready',update=lambda f,n:Match(1,1,0,0,10,(c,),'r'))
  points=Engine(provider).step(frame()).points
  assert [p.distance for p in points]==[300,330,360]

def test_published_but_nonbinding_branch_does_not_receive_anticipation(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(10,10,10,'branch',ceiling=(10)*(4/1.8)**0.5)
  previews=[Preview('choice',0,((),(p,)),True),Preview('chosen',0,((p,),),True)]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:previews.pop(0))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=25))
  result=engine.step(frame(1.05,speed=25))
  assert engine.stats['late']>0
  assert result.route_options==((),)

def test_actual_applied_cap_allows_a_continuous_aggregate_after_handoff(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(10,10,10,'branch',ceiling=(10)*(4/1.8)**0.5)
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('chosen',0,((p,),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(2,1,0,0,0,(),'new-way'))
  engine=Engine(provider)
  engine.step(frame(speed=5))
  result=engine.step(frame(1.05,speed=25,applied_map_cap=10))
  assert result.route_options==((p,),)

def test_commanded_approach_survives_changed_acceleration_reserve(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(400,12,12,'bend',ceiling=(12)*(4/1.8)**0.5)
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('chosen',0,((p,),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider)
  assert engine.step(frame(speed=24,acceleration=-.6)).route_options==((p,),)
  # The cap actually published from the previous approach remains above the
  # recomputed reserve envelope; this is not discovery of a different corner.
  result=engine.step(frame(1.05,speed=24,acceleration=1,applied_map_cap=24.8))
  assert result.route_options==((p,),)

def test_command_on_weaker_route_cannot_authorize_new_tight_exit(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  weak=PointCap(10,24,24,'weak',ceiling=(24)*(4/1.8)**0.5);tight=PointCap(10,10,10,'tight',ceiling=(10)*(4/1.8)**0.5)
  previews=[Preview('choice',0,((weak,),(tight,)),True),Preview('chosen',0,((tight,),),True)]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:previews.pop(0))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=24,acceleration=-.6))
  result=engine.step(frame(1.05,speed=24,acceleration=-.6,applied_map_cap=24))
  assert result.route_options==((),)

def test_unbounded_gap_or_fence_cannot_preserve_command_anticipation(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(400,12,12,'bend',ceiling=(12)*(4/1.8)**0.5)
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('chosen',0,((p,),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  for t,fence in ((2.1,0),(1.05,1)):
    engine=Engine(provider);engine.step(frame(speed=24,acceleration=-.6))
    result=engine.step(frame(t,speed=24,acceleration=1,applied_map_cap=24.8,observation_fence=fence))
    assert result.route_options==((),)

def test_successive_small_geometry_shrinks_cannot_ratchet_past_late_gate(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  speeds=iter((26,25.8,25.6))
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(0,next(speeds),26,'bend',ceiling=(26)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=5))
  assert any(engine.step(frame(1.05,speed=27,applied_map_cap=30)).route_options)
  assert engine.step(frame(1.1,speed=27,applied_map_cap=30)).route_options==((),)

def test_rejected_exit_and_changed_reserve_preserve_commanded_common_bend(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  tight=PointCap(0,10,10,'exit',ceiling=(10)*(4/1.8)**0.5);weak=PointCap(0,40,40,'weak',ceiling=(40)*(4/1.8)**0.5)
  previews=iter((Preview('choice',1000,((weak,),(tight,)),True),
                 Preview('chosen',1000,((tight,),),True),Preview('chosen',1000,((tight,),),True)))
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:next(previews))
  c=Corner('common',1,1,'r',100,175,250,(20,20,20),((0,0),(1,1)),0,curvature=tuple(1.8/(v*v) for v in (20,20,20)))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(c,),'r'))
  engine=Engine(provider);engine.step(frame(speed=5,acceleration=-.6))
  r=engine.step(frame(1.05,speed=27,acceleration=-.6,applied_map_cap=25))
  assert r.points and all(p.key=='common' for p in r.points) and r.route_options==((),)
  r=engine.step(frame(1.1,speed=27,acceleration=1,applied_map_cap=25))
  assert r.points and r.route_options==((),)

def test_current_uncertainty_used_for_both_continuation_envelopes(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(400,12,12,'bend',ceiling=(12)*(4/1.8)**0.5)
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((p,),),True))
  matches=iter((Match(1,1,0,0,0,(),'r'),Match(1,1,0,0,25,(),'r')))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:next(matches))
  engine=Engine(provider);engine.step(frame(speed=24,acceleration=-.6))
  r=engine.step(frame(1.05,speed=26,acceleration=-.6,applied_map_cap=27))
  assert r.route_options==((p,),)

def test_empty_memory_does_not_report_remembered_after_map_projection_changes(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',1000,((),),True))
  c=Corner('common',1,1,'r',300,350,400,(20,20,20),((0,0),(1,1)),0,curvature=tuple(1.8/(v*v) for v in (20,20,20)))
  corners=iter((c,replace(c,prior=(20.0000001,20.0000001,20.0000001))))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(next(corners),),'r'))
  engine=Engine(provider);engine.step(frame())
  r=engine.step(frame(1.05))
  assert not r.remembered and r.status=='Map corner'

def test_manual_slowdown_after_map_owned_takeover_completes_common_corner_learning(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',1000,((),),True))
  c=Corner('common',1,1,'r',100,145,190,(20,20,20),((0,0),(1,1)),0,curvature=tuple(1.8/(v*v) for v in (20,20,20)))
  progress=[0];observed=[]
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,progress[0],0,0,(c,),'r'))
  store=SimpleNamespace(lookup=lambda c:None,observe=lambda *args:observed.append(args))
  engine=Engine(provider,store)
  for i in range(205):
    progress[0]=i;auto=i<75
    engine.step(frame(1+i*.05,speed=22 if auto else max(15,22-(i-75)*.25),acceleration=-.6,
      odometer=i,automated=auto,owner='map' if auto else 'manual',binding_key='common' if auto else '',
      intervention=i==75,observation_id='takeover' if i==75 else '',applied_map_cap=20 if auto else 0,
      control_epoch=0 if auto else 1))
  assert len(observed)==1 and observed[0][0].key=='common'
  assert observed[0][1]==(15,15,15)

def test_known_preview_can_begin_braking_at_cruise_without_a_previous_cap(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  # Previous envelope25.02, current24.99 due ordinary forward travel, not a
  # newly revealed exit. Worker runs before Coordinator can issue its first cap.
  distance=(25.02**2-20**2)/1.2+5
  travelled=(25.02**2-24.99**2)/1.2
  previews=iter((Preview('route',0,((PointCap(distance,20,20,'bend',ceiling=(20)*(4/1.8)**0.5),),),True),
                 Preview('route',0,((PointCap(distance-travelled,20,20,'bend',ceiling=(20)*(4/1.8)**0.5),),),True)))
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:next(previews))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=25,acceleration=-.6))
  r=engine.step(frame(1.05,speed=25,acceleration=-.6,odometer=travelled))
  assert any(r.route_options)

def test_bounded_worker_stall_retains_same_previously_commanded_approach(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  distance=[300.]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(distance[0],12,12,'bend',ceiling=(12)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=5,acceleration=-.6))
  engine.step(frame(1.05,speed=24,acceleration=-.6,applied_map_cap=20))
  distance[0]-=14.4
  r=engine.step(frame(1.65,speed=24,acceleration=-.6,odometer=14.4,applied_map_cap=0))
  assert any(r.route_options)

def test_bounded_stall_cannot_authorize_a_new_tighter_exit(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=PointCap(300,12,12,'bend',ceiling=(12)*(4/1.8)**0.5);tight=PointCap(0,10,10,'new-exit',ceiling=(10)*(4/1.8)**0.5)
  previews=iter((Preview('route',0,((p,),),True),Preview('route',0,((p,),),True),
                 Preview('new-route',0,((tight,),),True)))
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:next(previews))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=5,acceleration=-.6))
  engine.step(frame(1.05,speed=24,acceleration=-.6,applied_map_cap=20))
  r=engine.step(frame(1.65,speed=24,acceleration=-.6,odometer=14.4))
  assert r.route_options==((),)

@pytest.mark.parametrize('discard_stale',[True,False])
def test_worker_backlog_retains_multiple_fresh_frames_without_a_new_command(monkeypatch,discard_stale):
  import ctypes,os,time
  import openpilot.starpilot.controls.lib.map_corner.worker as worker
  import openpilot.starpilot.controls.lib.map_corner.snapshot_provider as provider_module
  import openpilot.starpilot.controls.lib.map_corner.store as store_module
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  distance=[300.];flags=[1,1,0];results=[]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(distance[0],12,12,'bend',ceiling=(12)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  monkeypatch.setattr(provider_module,'SnapshotProvider',lambda root:provider)
  monkeypatch.setattr(store_module,'CornerStore',lambda root:SimpleNamespace(close=lambda:None))
  monkeypatch.setattr(worker,'demote',lambda:None)
  monkeypatch.setattr(ctypes,'CDLL',lambda *a,**k:SimpleNamespace(prctl=lambda *a:0))
  times=(1.65,1.7,1.75,1.8) if discard_stale else (1.1,1.15,1.2,1.25)
  wall=1.8 if discard_stale else 1.575
  queued=[frame(speed=5,acceleration=-.6),frame(1.05,speed=24,acceleration=-.6,applied_map_cap=20)]
  if discard_stale:queued.append(frame(1.1,speed=24,acceleration=-.6))
  queued.extend(frame(t,speed=24,acceleration=-.6,odometer=(t-1.05)*24) for t in times)
  clocks=iter([1,1,1.05]+[wall]*(len(queued)-2))
  monkeypatch.setattr(time,'monotonic',lambda:next(clocks,wall))
  frames=iter(queued)
  def get(**kwargs):
    f=next(frames)
    distance[0]=300-f.odometer
    if f.now==times[-1]:flags[0]=0
    return f
  worker.worker_main(SimpleNamespace(get=get),SimpleNamespace(put_nowait=results.append),flags,'','',os.getppid())
  assert len(results)==6 and all(any(r.route_options) for r in results)

def test_uncommanded_startup_spans_multiple_exchanges_but_expires(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  base=(25.02**2-20**2)/1.2+5;travel=(25.02**2-24.99**2)/1.2;d=[base]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(d[0],20,20,'bend',ceiling=(20)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=25,acceleration=-.6))
  for i in range(1,21):
    d[0]=base-i*travel
    r=engine.step(frame(1+i*.05,speed=25,acceleration=-.6,odometer=i*travel))
    assert any(r.route_options)
  d[0]=base-21*travel
  r=engine.step(frame(2.05,speed=25,acceleration=-.6,odometer=21*travel))
  assert r.route_options==((),)

def test_uncommanded_bridge_cannot_ratchet_small_geometry_shrinks(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  target=[26.]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(0,target[0],26,'bend',ceiling=(26)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=5))
  for i in range(1,9):
    target[0]=26-.05*i
    r=engine.step(frame(1+i*.05,speed=27))
  assert r.route_options==((),)

def test_overspeed_above_existing_cruise_does_not_discard_future_map_approach(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  base=(26.02**2-20**2)/1.2+5.2;d=[base]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(d[0],20,20,'bend',ceiling=(20)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider)
  for i in range(51):
    d[0]=base-i*1.3
    r=engine.step(frame(1+i*.05,speed=26,reference_speed=25,acceleration=-.6,
      odometer=i*1.3,applied_map_cap=25 if i>=35 else 0))
    assert any(r.route_options)
    if i==35:assert engine.continuation_deficit==0

def test_lower_existing_cruise_does_not_authorize_a_new_tighter_exit(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  p=[PointCap(500,20,20,'known',ceiling=(20)*(4/1.8)**0.5)]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((p[0],),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider)
  for i in range(31):engine.step(frame(1+i*.05,speed=26,reference_speed=25,acceleration=-.6))
  p[0]=PointCap(0,10,10,'new-exit',ceiling=(10)*(4/1.8)**0.5)
  r=engine.step(frame(2.55,speed=26,reference_speed=25,acceleration=-.6))
  assert r.route_options==((),)

def test_target_between_lower_reference_and_measured_speed_is_still_rejected(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  target=[PointCap(100,30,30,'known',ceiling=(30)*(4/1.8)**0.5)]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((target[0],),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=26,reference_speed=15,acceleration=-.6))
  target[0]=PointCap(0,20,20,'new-exit',ceiling=(20)*(4/1.8)**0.5)
  assert engine.step(frame(1.05,speed=26,reference_speed=15,acceleration=-.6)).route_options==((),)
  assert engine.step(frame(1.1,speed=26,reference_speed=26,acceleration=-.6)).route_options==((),)

def test_lower_reference_does_not_disable_geometry_deficit_accounting(monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.routes import Preview
  import openpilot.starpilot.controls.lib.map_corner.routes as routes
  target=[26.]
  monkeypatch.setattr(routes,'preview_routes',lambda tile,match:Preview('route',0,((PointCap(0,target[0],26,'bend',ceiling=(26)*(4/1.8)**0.5),),),True))
  provider=SimpleNamespace(tile=object(),status='Map ready',update=lambda f,n:Match(1,1,0,0,0,(),'r'))
  engine=Engine(provider);engine.step(frame(speed=5,reference_speed=15))
  for i in range(1,9):
    target[0]=26-.05*i
    r=engine.step(frame(1+i*.05,speed=27,reference_speed=15))
  assert r.route_options==((),)
