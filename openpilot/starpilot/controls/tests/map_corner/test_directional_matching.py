"""Opposite traffic is not a candidate route, but proximity alone proves nothing."""
import math
from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G, routes as Q
from openpilot.starpilot.controls.lib.map_corner.types import Way
from test_map_corner_provider import ll,fix

def road(identity,y=0,reverse=False,one_way=True):
  pts=tuple(ll(x,y) for x in range(-400,401,20))
  return Way(identity,tuple(reversed(pts)) if reverse else pts,one_way,'secondary','')

@pytest.mark.parametrize('mode',['cold','provisional','qualified'])
def test_opposite_carriageway_no_longer_vetoes_selected_road(mode):
  main=road(1);tile=G.build_tile((main,road(2,14,True)),'r')
  previous=None if mode=='cold' else G.find_match(G.build_tile((main,),'r'),fix(x=-110))
  m=G.find_match(tile,fix(),previous,provisional=mode=='provisional')
  assert m and (m.way_id,m.direction)==(1,1)

@pytest.mark.parametrize('reverse,one_way',[(False,True),(True,False),(False,False)])
def test_plausible_same_direction_or_two_way_road_still_vetoes(reverse,one_way):
  assert G.find_match(G.build_tile((road(1),road(2,14,reverse,one_way)),'r'),fix()) is None

@pytest.mark.parametrize('y,offset,accuracy,lanes',[(7,3,1,2),(6,2,1,2),(8,2,3,2),(10,3,1,4)])
def test_inside_opponent_corridor_or_uncertainty_margin_keeps_veto(y,offset,accuracy,lanes):
  opponent=replace(road(2,y,True),lanes=lanes)
  assert G.find_match(G.build_tile((road(1),opponent),'r'),fix(y=offset,accuracy=accuracy)) is None

@pytest.mark.parametrize('speed',[0,1,2.99])
def test_slow_noisy_heading_does_not_exempt_opposite_road(speed):
  tile=G.build_tile((road(1),road(2,14,True)),'r')
  assert G.find_match(tile,replace(fix(),speed=speed)) is None

@pytest.mark.parametrize('heading,expected',[(30.1,False),(30,True),(29.9,True),(0,True),(359.9,True)])
def test_legal_heading_error_boundaries_and_wrap(heading,expected):
  # Exact Cartesian north segment avoids latitude projection boundary noise.
  r=G.Road(road(2),((0,0),(0,-100)),(0,100))
  assert (G.legal_heading_error(r,0,heading)>=150)==expected

@pytest.mark.parametrize('other_heading,expected',[(239.9,False),(240.0,True),(240.1,True)])
def test_match_exemption_heading_threshold(monkeypatch,other_heading,expected):
  tile=G.build_tile((road(1),road(2,14,True)),'r')
  original=G.bearing
  def heading(a,b):
    return other_heading if b[0]<a[0] else original(a,b)
  monkeypatch.setattr(G,'bearing',heading)
  assert (G.find_match(tile,fix()) is not None)==expected

def test_skewed_fix_does_not_turn_diverging_segment_into_opposite():
  # Competitor is 140deg from selected road but 160deg from the fix bearing.
  pts=tuple(ll(-100+t*math.sin(math.radians(230)),14+t*math.cos(math.radians(230))) for t in (-4,4))
  tile=G.build_tile((road(1),Way(2,pts,True,'secondary','')),'r')
  assert G.find_match(tile,fix(bearing=70)) is None

def test_curving_competitor_still_vetoes_via_non_opposing_segment():
  opponent=Way(2,(ll(100,14),ll(-100,14),ll(-100,-40)),True,'secondary','')
  assert G.find_match(G.build_tile((road(1),opponent),'r'),fix()) is None

def test_shared_vertex_crossover_is_not_excused_by_opposing_heading():
  opponent=Way(2,(ll(100,14),ll(-100,0),ll(-400,14)),True,'secondary','')
  assert G.find_match(G.build_tile((road(1),opponent),'r'),fix()) is None

def test_normalized_minus_one_way_and_unnormalized_conservative_case():
  # Generator reverses OSM points for -1, so stored forward is legal traffic.
  assert G.find_match(G.build_tile((road(1),road(2,14,True)),'r'),fix())
  assert G.find_match(G.build_tile((road(1),road(2,14,False)),'r'),fix()) is None

@pytest.mark.parametrize('reverse,one_way,expected',[(True,True,False),(True,False,True),(False,True,True),(False,False,True)])
def test_unknown_continuation_uses_legal_direction(reverse,one_way,expected):
  incoming=Way(1,(ll(-100,0),ll(0,0)),True,'secondary','')
  pts=(ll(0,14),ll(100,14))
  other=Way(2,tuple(reversed(pts)) if reverse else pts,one_way,'secondary','')
  tile=G.build_tile((incoming,other),'r')
  assert Q.unknown_continuation(tile,ll(0,0),(1,0,1))==expected

def test_opposite_road_cannot_hide_mapped_all_slow_fork():
  incoming=Way(1,tuple(ll(x,0) for x in range(-400,1,10)),True,'secondary','')
  left=Way(2,tuple(ll(0,y) for y in range(0,201,10)),True,'secondary','')
  right=Way(3,tuple(ll(0,-y) for y in range(0,201,10)),True,'secondary','')
  opposite=Way(4,(ll(200,14),ll(0,14),ll(-400,14)),True,'secondary','')
  tile=G.build_tile((incoming,left,right,opposite),'r')
  m=G.find_match(tile,fix(x=-200));assert m
  result=Q.preview_routes(tile,m)
  assert result.complete and result.options and all(result.options)

def test_opposite_exemption_preserves_straight_alternative_at_side_turn():
  main=road(1);side=Way(3,(ll(0,0),ll(0,200)),True,'secondary','')
  tile=G.build_tile((main,road(2,14,True),side),'r')
  m=G.find_match(tile,fix(x=-200));assert m
  result=Q.preview_routes(tile,m)
  assert any(not option for option in result.options)


def test_fix_heading_must_independently_exclude_opposing_candidate():
  # Selected heading90 vs competitor250 is160deg; fix110 vs competitor is140deg.
  pts=tuple(ll(-100+t*math.sin(math.radians(250)),14+t*math.cos(math.radians(250))) for t in (-4,4))
  tile=G.build_tile((road(1),Way(2,pts,True,'secondary','')),'r')
  assert G.find_match(tile,fix(bearing=110)) is None


def test_accuracy_separation_guard_outside_opponent_corridor():
  # Dist6.5 > corridor5.5 and > selected3+floor3, but <= selected3+2*accuracy2.
  tile=G.build_tile((road(1),road(2,9.5,True)),'r')
  assert G.find_match(tile,fix(y=3,accuracy=2)) is None


def test_three_metre_separation_floor_outside_opponent_corridor():
  # Dist5 > corridor2.75 and > selected2.5+2*accuracy1, but <= selected2.5+floor3.
  tile=G.build_tile((road(1),replace(road(2,7.5,True),lanes=1)),'r')
  assert G.find_match(tile,fix(y=2.5,accuracy=1)) is None
