import math
from dataclasses import replace
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.types import Way,PointCap
from test_map_corner_provider import ll,fix,bend_way

def preview(ways,x=-100,bearing=90):
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  tile=G.build_tile(tuple(ways),'r');match=G.find_match(tile,fix(x=x,bearing=bearing))
  assert match
  return preview_routes(tile,match)

def straight(identity=1,y=0):
  return Way(identity,tuple(ll(x,y) for x in range(-400,501,10)),False,'secondary','')

def test_graph_keeps_straight_option_at_side_turn():
  branch=Way(2,tuple(ll(0,y) for y in range(0,201,10)),False,'unclassified','')
  result=preview((straight(),branch))
  assert len(result.options)>=2
  assert any(not option for option in result.options)
  assert any(option for option in result.options)

def test_ambiguous_all_slow_fork_retains_both_turn_profiles():
  incoming=Way(1,tuple(ll(x,0) for x in range(-400,1,10)),False,'secondary','')
  left=Way(2,tuple(ll(0,y) for y in range(0,201,10)),False,'secondary','')
  right=Way(3,tuple(ll(0,-y) for y in range(0,201,10)),False,'secondary','')
  result=preview((incoming,left,right))
  assert len(result.options)==2 and all(result.options)
  assert all(any(16<p.prior<19 for p in option) for option in result.options)

def test_missing_straight_connection_adds_unknown_continuation():
  turn=Way(1,tuple(ll(x,0) for x in range(-400,1,10))+tuple(ll(0,y) for y in range(10,201,10)),False,'secondary','')
  missing=Way(2,tuple(ll(x,1) for x in range(10,501,10)),False,'secondary','')
  assert any(not option for option in preview((turn,missing)).options)

def test_capacity_failure_keeps_an_unrestricted_alternative():
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  incoming=Way(1,tuple(ll(x,0) for x in range(-400,1,10)),False,'secondary','')
  branches=[Way(i+2,(ll(0,0),ll(100*math.cos(i),100*math.sin(i))),False,'secondary','') for i in range(40)]
  tile=G.build_tile((incoming,*branches),'r');match=G.find_match(tile,fix(x=-200))
  assert match is not None
  result=preview_routes(tile,match)
  assert not result.complete and any(not option for option in result.options)

def test_directed_roundabout_all_exits_share_some_slow_geometry():
  # Circle tangent entry at its bottom; two outward exits and legal circulation.
  ringpts=[ll(40*math.sin(i*math.pi/12),40-40*math.cos(i*math.pi/12)) for i in range(24)]
  ring=Way(2,tuple(ringpts+[ringpts[0]]),True,'secondary','')
  entry=Way(1,tuple(ll(x,0) for x in range(-400,1,10)),True,'secondary','')
  exit1=Way(3,(ringpts[6],*tuple(ll(x,40) for x in range(50,201,10))),True,'secondary','')
  exit2=Way(4,(ringpts[12],*tuple(ll(0,y) for y in range(90,251,10))),True,'secondary','')
  result=preview((entry,ring,exit1,exit2))
  assert result.complete and len(result.options)>=2
  assert all(any(p.prior<20 for p in option) for option in result.options)

def test_geometric_crossing_does_not_create_an_unsupported_connection():
  crossing=Way(2,(ll(40,-100),ll(40,100)),False,'secondary','')
  result=preview((straight(),crossing))
  assert not any(any(p.prior<30 for p in option) for option in result.options)

def test_sparse_straight_junction_arms_still_bound_all_slow_options():
  # A long straight arm does not erase an explicit right-angle connection.
  # Sparse bend samples still have their separate curvature-support gate.
  incoming=Way(1,(ll(-400,0),ll(-100,0),ll(0,0)),False,'secondary','')
  left=Way(2,(ll(0,0),ll(0,120),ll(0,240)),False,'secondary','')
  right=Way(3,(ll(0,0),ll(0,-120),ll(0,-240)),False,'secondary','')
  result=preview((incoming,left,right),x=-150)
  assert len(result.options)==2 and all(result.options)
  assert all(any(16<p.prior<19 for p in option) for option in result.options)

def test_sparse_side_turn_never_limits_straight_main_road():
  main=Way(1,tuple(ll(x,0) for x in (-400,-200,0,200,400)),False,'secondary','')
  side=Way(2,(ll(0,0),ll(0,150)),False,'unclassified','')
  result=preview((main,side),x=-150)
  assert any(not option for option in result.options)

def test_illegal_reverse_exit_is_not_an_onward_option():
  from openpilot.starpilot.controls.lib.map_corner.routes import outgoing
  incoming=Way(1,tuple(ll(x,0) for x in range(-400,1,10)),False,'secondary','')
  turn=Way(2,tuple(ll(0,y) for y in range(0,201,10)),True,'secondary','')
  towards=Way(3,tuple(ll(x,0) for x in range(400,-1,-10)),True,'secondary','')
  tile=G.build_tile((incoming,turn,towards),'r')
  assert outgoing(tile,ll(0,0),(1,len(incoming.points)-2,1))==((2,0,1),)

def test_unsupported_onward_road_is_preserved_as_permissive_option():
  road=bend_way();unsupported=Way(2,tuple(ll(x,0) for x in range(0,401,10)),False,'service','')
  result=preview((road,unsupported))
  assert any(not option for option in result.options)

def test_shared_near_duplicate_vertex_cannot_silently_disconnect_a_branch():
  import pytest
  main=Way(1,(ll(-400,0),ll(0,0),ll(.01,0),ll(400,0)),False,'secondary','')
  side=Way(2,(ll(.01,0),ll(.01,200)),False,'secondary','')
  with pytest.raises(ValueError,match='Invalid road geometry'):
    G.build_tile((main,side),'r')

def test_known_corner_has_same_prior_profile_before_route_commitment():
  import pytest
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  road=bend_way();side=Way(2,tuple(ll(0,-y) for y in range(0,301,10)),False,'unclassified','')
  tile=G.build_tile((road,side),'r');match=G.find_match(tile,fix(x=-100));assert match and match.corners
  c=match.corners[0];result=preview_routes(tile,match)
  paths=[path for path in result.options if any(p.key==c.key for p in path)]
  assert paths
  expected=[(c.entry+i*(c.exit-c.entry)/3-match.progress,c.prior[i]) for i in range(3)]
  for path in paths:
    actual=[(p.distance,p.prior) for p in path if p.key==c.key]
    assert len(actual)==3
    for a,b in zip(actual,expected):assert a==pytest.approx(b)
