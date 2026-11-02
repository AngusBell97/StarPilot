import math
from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G,routes as Q
from openpilot.starpilot.controls.lib.map_corner.types import Way
from test_map_corner_provider import ll,fix

def way(i,points,**kw):return Way(i,tuple(ll(x,y) for x,y in points),kw.get('one_way',True),kw.get('kind','secondary'),kw.get('hazard',''))
def near():return way(9,[(0,35),(300,35)],one_way=False,kind='unclassified')
def unknown(main,others=(),index=1):
  tile=G.build_tile((main,*others,near()),'r')
  return Q.unknown_continuation(tile,main.points[index],(main.identity,index-1,1))

@pytest.mark.parametrize('gap',[20,79,96,150])
def test_unbranched_straight_does_not_invent_a_disconnected_exit(gap):
  main=way(1,[(-2*gap,0),(-gap,0),(0,0),(gap,0),(2*gap,0)])
  assert not unknown(main,index=2)

def test_real_sharp_turn_keeps_missing_straight_protection():
  assert unknown(way(1,[(-100,0),(0,0),(0,100)]))

def test_many_small_turns_are_checked_over_whole_window():
  pts=[(-80,0),(-20,0),(0,0),(8,3.7),(14,11),(17,20),(17,80)]
  assert unknown(way(1,pts),index=2)

def test_branch_inside_window_keeps_unknown_choice():
  main=way(1,[(-100,0),(0,0),(20,0),(100,0)])
  branch=way(3,[(20,0),(20,100)])
  assert unknown(main,(branch,))

@pytest.mark.parametrize('pts',[[(-100,0),(0,0)],[(-10,0),(0,0),(100,0)]])
def test_insufficient_window_never_clamps_to_an_endpoint(pts):assert unknown(way(1,pts))

@pytest.mark.parametrize('kw',[{'hazard':'school'},{'kind':'service'}])
def test_unsupported_through_geometry_keeps_permissive_option(kw):assert unknown(way(1,[(-100,0),(0,0),(100,0)],**kw))

def test_unique_short_way_join_is_still_a_through_road():
  a=way(1,[(-100,0),(-40,0),(0,0)]);b=way(2,[(0,0),(40,0),(100,0)])
  assert not unknown(a,(b,),index=2)

def test_sparse_cross_way_join_keeps_missing_connection_protection():
  a=way(1,[(-200,0),(-100,0),(0,0)]);b=way(2,[(0,0),(100,0),(200,0)])
  assert unknown(a,(b,),index=2)

def test_other_road_does_not_hide_downstream_bend_on_only_reachable_path():
  # Other road ends before the real bend and shares no vertices.
  points=[(x,0) for x in range(-400,1,10)]
  points += [(120*math.sin(i*math.pi/36),120*(1-math.cos(i*math.pi/36))) for i in range(1,19)]
  points += [(120,y) for y in range(130,351,10)]
  main=way(1,points);other=way(9,[(-200,35),(-80,35)],one_way=False)
  tile=G.build_tile((main,other),'r');m=G.find_match(tile,fix(x=-300))
  p=Q.preview_routes(tile,m)
  assert p.complete and all(p.options), 'Unreachable route must not cancel the bend'

def test_disconnected_road_not_added_as_graph_edge():
  main=way(1,[(-100,0),(0,0),(100,0)]);tile=G.build_tile((main,near()),'r')
  assert Q.outgoing(tile,ll(0,0),(1,0,1))==((1,1,1),)


def test_unmapped_shallow_straight_is_an_explicit_residual_limit():
  # Both this geometry and a genuinely disconnected road have the same schema.
  # The rule cannot prove real-world completeness; this records that boundary.
  angle=math.radians(20)
  main=way(1,[(-100,0),(0,0),(100*math.cos(angle),100*math.sin(angle))])
  missing=way(2,[(10,1),(200,1)])
  tile=G.build_tile((main,missing),'r')
  assert not Q.unknown_continuation(tile,ll(0,0),(1,0,1))
