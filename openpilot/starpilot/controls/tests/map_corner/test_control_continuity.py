from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.provider import MatchTracker
from openpilot.starpilot.controls.lib.map_corner.worker import Engine
from openpilot.starpilot.controls.lib.map_corner.types import Way,Match,Corner
from test_map_corner_provider import ll,fix
from test_connected_tracking import road,parallel,travel
from test_route_worker import frame


def test_proven_disconnected_continuity_is_control_trusted():
  tr,rows=travel(parallel())
  assert tr.qualified and tr.contested
  assert getattr(tr,'control_trusted',False)
  assert tr.control_reason=='proven_continuity'


def test_trusted_contested_match_can_publish_fresh_targets():
  c=Corner('c',1,1,'r',300,345,390,(10,11,12),((0,0),(1,1)),0,curvature=(.01,.01,.01))
  match=Match(1,1,0,0,10,(c,),'r')
  provider=SimpleNamespace(status='Map ready',tracker=SimpleNamespace(contested=True,control_trusted=True),update=lambda f,n:match)
  result=Engine(provider).step(frame())
  assert result.points


def crossing(join=200):
  other=Way(2,(ll(0,-100),ll(0,100),ll(join,100),ll(join,0)),False,'secondary','')
  return G.build_tile((road(),other),'r')


def test_remote_connection_does_not_drop_crossing_road_track():
  tr,rows=travel(crossing(),end=30)
  assert all(m and m.way_id==1 for x,m,_,_ in rows if x>=-15)
  assert tr.control_trusted


def test_near_connection_still_rejects_crossing_ambiguity():
  tile=crossing(20)
  previous=G.find_match(G.build_tile((road(),),'r'),fix(x=-2))
  match=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  guard=G.ContinuityGuard(tile,fix(x=0),previous)
  assert not guard.permits(match,2,0)




def test_real_parallel_merge_stays_ambiguous():
  other=Way(2,(ll(0,12),ll(200,12),ll(200,0)),True,'secondary','')
  tr,rows=travel(G.build_tile((road(),other),'r'))
  assert any(m is None for x,m,_,_ in rows if x>=0)


def test_better_competitor_cannot_get_control_from_retained_identity():
  tr=MatchTracker();tile=parallel(8);saw_competitor=False
  for x in range(-200,45,2):
    m=tr.update(tile,fix(1+(x+200)/20,x=x,y=min(6,max(0,(x+40)*.1))))
    if getattr(tr,'better_match',None) is not None:
      saw_competitor=True
      assert not getattr(tr,'control_trusted',False)

  assert saw_competitor

def test_invalid_fix_releases_control_trust():
  tr,_=travel(parallel());assert tr.control_trusted
  tr.update(parallel(),replace(fix(tr.last_fix.received_at+.1,x=102),has_fix=False))
  assert not tr.control_trusted


def test_radius_cache_cannot_weaken_full_connection_proof():
  tile=crossing(200);m=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  guard=G.ContinuityGuard(tile,fix(x=0),m)
  assert guard.disconnected(m,2,0,limit=60) is True
  assert guard.disconnected(m,2,0) is False


def test_crossing_relaxation_needs_fork_free_history():
  tile=crossing(200);m=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  guard=G.ContinuityGuard(tile,fix(x=0),m,fork_free=0)
  assert not guard.permits(m,2,0)


def test_unknown_full_search_never_falls_back_to_small_radius():
  tile=crossing(200);m=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  guard=G.ContinuityGuard(tile,fix(x=0),m,fork_free=200)
  guard.remaining=1
  assert not guard.permits(m,2,0) and guard.remaining==0


def test_contested_control_cannot_collect_learning_history():
  c=Corner('c',1,1,'r',300,345,390,(10,11,12),((0,0),(1,1)),0,curvature=(.01,.01,.01))
  match=Match(1,1,0,0,10,(c,),'r')
  provider=SimpleNamespace(status='Map ready',tracker=SimpleNamespace(contested=True,control_trusted=True),update=lambda f,n:match)
  engine=Engine(provider)
  for i in range(40):engine.step(frame(1+i*.05,binding_key='c',owner='map'))
  assert not engine.observer.history and not engine.observer.active
  assert engine.stats['commits']==0


def test_slow_unchecked_interval_resets_fork_free_history():
  tile=G.build_tile((road(),),'r');tr,rows=travel(tile,end=20)
  assert tr.fork_free_metres>60
  f=replace(fix(tr.last_fix.received_at+.1,x=20.2),speed=2)
  tr.update(tile,f)
  assert tr.fork_free_metres==0


def test_unknown_guard_state_cannot_be_hidden_by_an_ordinary_predicate():
  tile=crossing(20);m=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  guard=G.ContinuityGuard(tile,fix(x=0),m,fork_free=200)
  guard.remaining=0
  assert not guard.permits(m,2,0)
  assert guard.unknown
  assert guard.proof_state=='unknown'


def test_duplicate_fix_does_not_accrue_trust_distance_or_refresh_age():
  tile=parallel();tr,_=travel(tile);f=tr.last_fix;before=tr.fork_free_metres
  assert tr.control_trusted
  for _ in range(20):tr.update(tile,f)
  assert tr.last_fix is f and tr.fork_free_metres==before


def test_unsupported_parallel_road_still_blocks_control():
  other=replace(road(2,start=0,y=14),road_class='service')
  tile=G.build_tile((road(),other),'r');tr,rows=travel(tile)
  assert not tr.control_trusted
  assert any(m is None for x,m,_,_ in rows if x>=0)


def test_full_disconnection_proof_retains_checked_fork_free_distance():
  tr,rows=travel(parallel())
  assert tr.contested and tr.control_trusted
  assert tr.fork_free_metres>=100


def test_fork_crossed_during_a_proved_contest_resets_fork_free_history():
  branch=Way(3,(ll(220,0),ll(220,-100),ll(600,-100)),True,'secondary','')
  tile=G.build_tile((road(),road(2,start=0,y=14),branch),'r');tr=MatchTracker()
  for i,x in enumerate(range(-400,186,5)):
    tr.update(tile,replace(fix(1+i*.1,x=x),speed=50))
  assert tr.contested and tr.control_trusted and tr.fork_free_metres>60
  tr.update(tile,replace(fix(tr.last_fix.received_at+.74,x=222),speed=50))
  assert tr.fork_free_metres==0 and not tr.control_trusted


def opposing(angle=180,spacing=14,join=200,lanes=2,heading=90,offset=0):
  import math
  ux=math.cos(math.radians(angle));uy=-math.sin(math.radians(angle))
  other=Way(2,(ll(-100*ux,spacing-100*uy),ll(100*ux,spacing+100*uy),ll(join,100),ll(join,0)),True,'secondary','',lanes)
  tile=G.build_tile((road(),other),'r');m=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  m=replace(m,offset=offset)
  guard=G.ContinuityGuard(tile,replace(fix(x=0),bearing=heading),m,fork_free=200)
  return guard,m

@pytest.mark.parametrize('angle',[119,120,121,150,180])
def test_local_opposing_or_crossing_boundary_needs_real_disconnection(angle):
  guard,m=opposing(angle)
  assert guard.disconnected(m,2,0) is False
  assert guard.disconnected(m,2,0,limit=60) is True
  assert guard.permits(m,2,0)

@pytest.mark.parametrize('angle',[120,150,180])
def test_opposing_road_inside_its_corridor_cannot_be_ignored(angle):
  guard,m=opposing(angle,spacing=1.5)
  assert not guard.permits(m,2,0)


def test_opposing_heading_must_agree_with_selected_tangent_too():
  guard,m=opposing(115,heading=80)
  assert not guard.permits(m,2,0)

@pytest.mark.parametrize('spacing,lanes,offset,expected',[(4.49,2,0,False),(4.51,2,0,True),(6,4,0,False),(4.8,2,2,False),(5.1,2,2,True)])
def test_opposing_lane_corridor_and_offset_margin_boundaries(spacing,lanes,offset,expected):
  guard,m=opposing(spacing=spacing,lanes=lanes,offset=offset)
  assert guard.permits(m,2,0) is expected


def test_near_connection_vetoes_opposing_local_proof():
  guard,m=opposing(join=30)
  assert guard.disconnected(m,2,0,limit=60) is False
  assert not guard.permits(m,2,0)


def test_opposing_proof_is_unavailable_before_full_qualification(monkeypatch):
  import openpilot.starpilot.controls.lib.map_corner.provider as pm
  guard,m=opposing(120);calls=[];original=pm.find_match
  def traced(*args,**kwargs):
    calls.append(kwargs.get('provisional',False));return original(*args,**kwargs)
  def forbidden(*args,**kwargs):raise AssertionError('Unqualified motion used a continuity guard')
  monkeypatch.setattr(pm,'find_match',traced);monkeypatch.setattr(pm,'ContinuityGuard',forbidden)
  tr=MatchTracker()
  for i,x in enumerate(range(-70,1,2)):
    tr.update(guard.tile,fix(1+i*.1,x=x))
    assert not tr.qualified and not tr.control_trusted
  assert any(calls), 'The sequence must enter provisional tracking before the competing road'
  assert tr.last_match is None, 'The ambiguous final fix must still be rejected'


def test_opposing_proof_unknown_budget_does_not_authorize_control():
  guard,m=opposing(150);guard.remaining=1
  assert not guard.permits(m,2,0) and guard.unknown
