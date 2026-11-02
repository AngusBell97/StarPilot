"""Behavior regressions derived from the observed rejection mechanisms."""
from dataclasses import replace
from openpilot.starpilot.controls.lib.map_corner.types import Way
from openpilot.starpilot.controls.lib.map_corner.geometry import build_tile,find_match
from openpilot.starpilot.controls.lib.map_corner.provider import MatchTracker
from test_map_corner_provider import ll,fix,bend_way

def straight():
  return Way(1,tuple(ll(x,0) for x in range(-400,1001,10)),False,'secondary','')

def test_gradual_residual_change_does_not_erase_an_established_unique_road():
  tracker=MatchTracker();tile=build_tile((straight(),),'r')
  for i in range(101):
    match=tracker.update(tile,fix(1+i*.1,x=-350+2*i,y=3.4*i/100))
  assert match is not None

def test_temporal_track_survives_perpendicular_side_access_without_choosing_it():
  branch=Way(2,(ll(0,0),ll(0,100)),False,'unclassified','')
  tracker=MatchTracker();tile=build_tile((straight(),branch),'r')
  for i in range(176):
    match=tracker.update(tile,fix(1+i*.1,x=-350+2*i))
  assert match is not None and match.way_id==1
  assert tracker.update(tile,fix(18.6,x=0,y=2,bearing=0)) is None

def test_established_track_keeps_identity_but_contests_disconnected_parallel_road():
  parallel=Way(2,tuple(ll(x,10) for x in range(100,1001,10)),False,'secondary','')
  tracker=MatchTracker();tile=build_tile((straight(),parallel),'r')
  for i in range(221):match=tracker.update(tile,fix(1+i*.1,x=-350+2*i))
  assert match is not None and match.way_id==1
  assert tracker.contested, "Retained identity must not authorize new map braking"

def test_late_junction_does_not_erase_supported_earlier_bend():
  base=bend_way();road=replace(base,points=base.points[:34]+(ll(100,120),))
  branch=Way(2,(road.points[-1],ll(200,120)),False,'unclassified','')
  tile=build_tile((road,branch),'r');match=find_match(tile,fix(x=-100))
  assert match is not None and match.corners
  assert all(c.exit<tile.roads[road.identity].along[-1]-40 for c in match.corners)

def test_shared_node_does_not_authorize_a_shallow_fork_after_a_missing_fix():
  import math
  branch=Way(2,tuple(ll(x,x*math.tan(math.radians(10))) for x in range(0,501,10)),False,'secondary','')
  tile=build_tile((straight(),branch),'r')
  previous=find_match(tile,replace(fix(x=-25),speed=55))
  assert previous is not None
  # A0.6s GPS gap at55m/s can jump over the pre-junction ambiguity zone. Both
  # onward directions still fit heading; the old main road continues past node.
  assert find_match(tile,replace(fix(1.6,x=8,y=1.4),speed=55),previous) is None
