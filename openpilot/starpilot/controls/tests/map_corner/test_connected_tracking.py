from dataclasses import replace
from types import SimpleNamespace
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.provider import MatchTracker
from openpilot.starpilot.controls.lib.map_corner.worker import Engine
from openpilot.starpilot.controls.lib.map_corner.types import Way,Match,Corner
from test_map_corner_provider import ll,fix
from test_route_worker import frame

def road(i=1,start=-600,end=1000,y=0):return Way(i,tuple(ll(x,y) for x in range(start,end+1,10)),True,'secondary','')
def parallel(y=14):return G.build_tile((road(),road(2,start=0,y=y)),'r')
def travel(tile,end=100,y=lambda x:0):
  tr=MatchTracker();rows=[]
  for x in range(-200,end+1,2):
    m=tr.update(tile,fix(1+(x+200)/20,x=x,y=y(x)))
    rows.append((x,m,getattr(tr,'contested',False),getattr(tr,'release_reason','')))
  return tr,rows

def test_established_disconnected_parallel_road_keeps_identity_but_is_contested():
  tr,rows=travel(parallel())
  at=[r for r in rows if r[0]>=0]
  assert all(m and m.way_id==1 for _,m,_,_ in at)
  assert all(c for _,_,c,_ in at)
  assert tr.qualified

def test_qualification_resumes_immediately_when_disconnected_road_separates():
  other=Way(2,tuple(ll(x,14) for x in range(0,61,10))+(ll(80,100),),True,'secondary','')
  tr,rows=travel(G.build_tile((road(),other),'r'),end=110)
  assert all(m for x,m,_,_ in rows if x>=0)
  assert not rows[-1][2] and tr.qualified

def test_cold_parallel_match_is_still_ambiguous():assert G.find_match(parallel(),fix(x=20)) is None

def test_consistently_better_competitor_releases_instead_of_switching():
  tr,rows=travel(parallel(8),end=90,y=lambda x:min(6,max(0,(x+40)*.1)))
  assert any(reason=='better competitor' for _,_,_,reason in rows)
  assert all(m is None or m.way_id==1 for _,m,_,_ in rows)

@pytest.mark.parametrize('kind',['fork','merge'])
def test_connected_parallel_branch_is_not_locked_out(kind):
  pts=[(0,0),(20,10),(600,10)] if kind=='fork' else [(-20,10),(180,10),(200,0)]
  other=Way(2,tuple(ll(x,y) for x,y in pts),True,'secondary','')
  tr,rows=travel(G.build_tile((road(),other),'r'),end=450 if kind=='fork' else 100)
  assert any(m is None for x,m,_,_ in rows if x>=0)
  assert not any(c for _,_,c,_ in rows), 'A reachable road is not a disconnected rescue'

def test_unique_split_keeps_qualified_identity_on_successor():
  tile=G.build_tile((road(end=0),road(2,start=0)),'r');tr,rows=travel(tile)
  assert all(m for x,m,_,_ in rows if x>=-50)
  assert rows[-1][1].way_id==2

@pytest.mark.parametrize('change',[{'has_fix':False},{'bearing':270},{'accuracy':6},{'speed':0}])
def test_held_identity_releases_on_invalid_or_contradictory_motion(change):
  tr,rows=travel(parallel(),end=40);f=replace(fix(13.1,x=42),**change)
  assert tr.update(parallel(),f) is None
  assert not tr.qualified

def test_changed_tile_cannot_inherit_lock():
  tr,_=travel(parallel(),end=40);tile=parallel();tile.provenance='updated'
  assert tr.update(tile,fix(13.1,x=42)) is None and not tr.qualified

def test_duplicate_and_reordered_fixes_cannot_refresh_protection():
  tr,_=travel(parallel(),end=40);before=tr.last_fix
  m=tr.update(parallel(),before)
  assert m and tr.contested
  assert tr.update(parallel(),replace(before,identity=before.identity-1)) is None
  assert not tr.qualified

def test_contested_match_cannot_publish_common_or_route_targets():
  c=Corner('c',1,1,'r',300,345,390,(10,11,12),((0,0),(1,1)),0,curvature=(.01,.01,.01))
  m=Match(1,1,0,0,10,(c,),'r')
  provider=SimpleNamespace(status='Map ready',tracker=SimpleNamespace(contested=True),update=lambda f,n:m)
  engine=Engine(provider);out=engine.step(frame())
  assert not out.points and not out.route_options and not out.key
  assert not engine.command_history_valid and not engine.anticipated


def test_three_fresh_drifting_steps_release_a_wrong_lock():
  def y(x):
    if x<=18:return min(4.6,max(0,(x+5)*.2))
    return min(6.4,4.6+(x-18)*.3)
  tr,rows=travel(parallel(6),end=26,y=y)
  assert any(reason=='better competitor' for x,_,_,reason in rows if 18<=x<=24)


def test_better_competitor_counter_survives_degree_two_way_splits():
  others=tuple(road(2+i,start=i*10,end=(i+1)*10,y=8) for i in range(16))
  tr,rows=travel(G.build_tile((road(),*others),'r'),end=90,y=lambda x:min(6,max(0,(x+40)*.1)))
  assert any(reason=='better competitor' for _,_,_,reason in rows)


def test_nonfinite_fresh_fix_releases_without_graph_work():
  tr,_=travel(parallel(),end=40)
  assert tr.update(parallel(),replace(fix(13.1,x=42),latitude=float('nan'))) is None
  assert not tr.qualified


def test_unsupported_competing_geometry_cannot_earn_protection():
  other=replace(road(2,start=0,y=14),road_class='service')
  tr,rows=travel(G.build_tile((road(),other),'r'))
  assert not any(c for _,_,c,_ in rows)
  assert rows[-1][1] is None


def test_fork_shadow_remains_after_connection_is_more_than300m_behind():
  main=road();other=Way(2,(ll(0,0),ll(10,12),ll(1000,12)),True,'secondary','')
  tile=G.build_tile((main,other),'r')
  m=G.find_match(G.build_tile((main,),'r'),fix(x=410))
  guard=G.ContinuityGuard(tile,fix(x=412),m,shadow=True)
  assert not guard.permits(m,2,1)
  assert not guard.clear_shadow(m)


def test_connection_search_overflow_never_proves_disconnection():
  main=Way(1,tuple(ll(x*.1,0) for x in range(-1000,5001)),True,'secondary','')
  other=road(2,start=0,end=500,y=14);tile=G.build_tile((main,other),'r')
  m=G.find_match(G.build_tile((main,),'r'),fix(x=10));guard=G.ContinuityGuard(tile,fix(x=12),m)
  assert guard.disconnected(m,2,1) is None
  assert guard.remaining==0


def test_structural_cache_is_bounded_and_invalidated_on_provenance_change():
  main=road();other=Way(2,tuple(ll(x*.1,14) for x in range(650)),True,'secondary','')
  tile=G.build_tile((main,other),'r');m=G.find_match(G.build_tile((main,),'r'),fix(x=-30))
  for segment in range(649):
    guard=G.ContinuityGuard(tile,fix(x=-30),m)
    assert guard.disconnected(m,2,segment) is True
  assert len(tile.continuity_cache)<=512
  tile.provenance='new';G.ContinuityGuard(tile,fix(x=-30),m)
  assert not tile.continuity_cache


def test_shared_budget_cannot_reset_for_each_competitor():
  tile=parallel();main=road();m=G.find_match(G.build_tile((main,),'r'),fix(x=-30))
  guard=G.ContinuityGuard(tile,fix(x=-30),m);guard.remaining=1
  assert guard.disconnected(m,2,1) is None
  assert guard.disconnected(m,2,2) is None
  assert guard.remaining==0


def test_two_possible_shared_nodes_inside50m_do_not_authorize_handoff():
  a=road(end=100);b=Way(2,(ll(20,0),ll(30,10),ll(40,0),ll(60,0)),True,'secondary','')
  tile=G.build_tile((a,b),'r');old=Match(1,1,610,0,13,(),'old');new=Match(2,1,35,0,13,(),'new')
  assert G.transition_distance(tile,old,new) is None
  assert G.transition_distance(tile,old,new,unique_join_only=True) is None


def test_fork_crossed_between_fresh_samples_does_not_earn_lock():
  branch=Way(2,tuple(ll(x,x*.18) for x in range(0,501,10)),True,'secondary','')
  tile=G.build_tile((road(),branch),'r');tr=MatchTracker()
  for i,x in enumerate(range(-350,-19,5)):
    tr.update(tile,replace(fix(1+i*.1,x=x),speed=50))
  assert tr.qualified
  t=tr.last_fix.received_at+.56
  assert tr.update(tile,replace(fix(t,x=8,y=1),speed=50)) is None
  assert not tr.contested


def test_dense_handoff_scan_obeys_shared_guard_work_budget():
  old=Way(1,tuple(ll(x*.1,0) for x in range(-6000,1)),True,'secondary','')
  new=road(2,start=0,end=100)
  tile=G.build_tile((old,new),'r')
  before=Match(1,1,tile.roads[1].along[-1]-1,0,13,(),'old')
  after=Match(2,1,1,0,13,(),'new')
  guard=G.ContinuityGuard(tile,fix(x=1),before)
  guard.remaining=32
  class CountedNodes(dict):
    reads=0
    def get(self,key,default=None):
      self.reads+=1
      return super().get(key,default)
  tile.nodes=CountedNodes(tile.nodes)
  assert guard.crossed_fork(after) is None
  assert tile.nodes.reads<=128, 'Guard must not rescan whole ways outside its shared budget'


@pytest.mark.parametrize('kind',['split-diverge','split-merge'])
def test_connection_search_follows_split_branch_in_both_directions(kind):
  if kind=='split-diverge':
    link=Way(2,(ll(0,0),ll(50,14)),True,'secondary','')
    other=road(3,start=50,end=600,y=14)
  else:
    link=Way(2,(ll(150,14),ll(200,0)),True,'secondary','')
    other=road(3,start=-100,end=150,y=14)
  tile=G.build_tile((road(),link,other),'r')
  m=G.find_match(G.build_tile((road(),),'r'),fix(x=100))
  guard=G.ContinuityGuard(tile,fix(x=102),m)
  assert guard.disconnected(m,3,0) is False
