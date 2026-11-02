import math
from dataclasses import replace
import pytest

from openpilot.starpilot.controls.lib.map_corner.types import Fix, Way
from openpilot.starpilot.controls.lib.map_corner.geometry import build_tile, find_match
from openpilot.starpilot.controls.lib.map_corner.provider import MatchTracker

LAT, LON = 51.0, -0.1
def ll(x, y):
  return LAT + y / 111195, LON + x / (111195 * math.cos(math.radians(LAT)))

def bend_way(identity=100, offset=0):
  xy = [(x, 0) for x in range(-150, 1, 10)]
  xy += [(100*math.sin(i*math.pi/36), 100*(1-math.cos(i*math.pi/36))) for i in range(1, 19)]
  xy += [(100, y) for y in range(110, 251, 10)]
  return Way(identity, tuple(ll(x, y+offset) for x,y in xy), False, 'secondary', '')

def fix(t=1, x=-100, y=0, bearing=90, **kw):
  lat, lon = ll(x,y)
  return Fix(t, int(t*100), lat, lon, bearing, kw.get('accuracy', 1), 20, True)

def test_forward_bend_has_geometry_prior_and_opposite_direction_distinct_key():
  tile = build_tile((bend_way(),), 'revision-a')
  forward = find_match(tile, fix())
  reverse = find_match(tile, fix(x=100,y=200,bearing=180))
  assert forward is not None and reverse is not None
  assert len(forward.corners) == len(reverse.corners) == 1
  assert forward.corners[0].key != reverse.corners[0].key
  assert 15 < min(forward.corners[0].prior) < 16
  assert forward.corners[0].entry > forward.progress

def test_straight_has_no_corner_target():
  way = Way(1,tuple(ll(x,0) for x in range(-150,251,10)),False,'secondary','')
  assert find_match(build_tile((way,), 'r'), fix()).corners == ()

@pytest.mark.parametrize('extra', [bend_way(200,10), Way(201,(ll(-100,-100),ll(-100,100)),False,'secondary','')])
def test_nearby_parallel_or_crossing_road_rejects_identity(extra):
  assert find_match(build_tile((bend_way(),extra),'r'),fix()) is None

def test_supported_corner_subspans_stay_outside_junction_buffer():
  road=bend_way()
  branch=Way(2,(road.points[20],ll(200,50)),False,'secondary','')
  tile=build_tile((road,branch),'r')
  s=tile.roads[road.identity].along[20]
  assert all(c.exit<s-40 or c.entry>s+40 for c in find_match(tile,fix()).corners)

@pytest.mark.parametrize('f', [fix(accuracy=8), fix(bearing=180), replace(fix(),has_fix=False), replace(fix(),latitude=float('nan'))])
def test_bad_localization_cannot_match(f):
  assert find_match(build_tile((bend_way(),),'r'),f) is None

def test_unrelated_tile_edit_keeps_corner_key_but_local_geometry_edit_invalidates():
  a=find_match(build_tile((bend_way(),),'old'),fix()).corners[0]
  b=find_match(build_tile((bend_way(),),'new'),fix()).corners[0]
  assert a.key == b.key
  road=bend_way()
  points=list(road.points)
  points[25]=(points[25][0]+.00001,points[25][1])
  c=find_match(build_tile((replace(road,points=tuple(points)),),'new'),fix()).corners[0]
  assert a.key != c.key

def test_one_way_cannot_be_recalled_in_reverse():
  assert find_match(build_tile((replace(bend_way(),one_way=True),),'r'),fix(x=100,y=200,bearing=180)) is None

def test_closed_way_does_not_generate_roundabout_corner():
  road=bend_way()
  road=replace(road,points=road.points+(road.points[0],))
  result=find_match(build_tile((road,),'r'),fix())
  assert result is None or result.corners == ()

def test_match_needs_distinct_fixes_and_sustained_progress_and_rejects_jump():
  tile=build_tile((bend_way(),),'r')
  tracker=MatchTracker()
  f=fix(1)
  assert tracker.update(tile,f) is None
  for _ in range(20): assert tracker.update(tile,f) is None
  assert tracker.update(tile,fix(1.3,x=-94)) is None
  assert tracker.update(tile,fix(1.6,x=-88)) is None
  for i in range(7,52):
    tracker.update(tile,fix(1+i*.1,x=-100+2*i))
  assert tracker.qualified
  assert tracker.update(tile,fix(6.2,x=100,y=180,bearing=180)) is None

def test_sparse_map_geometry_does_not_invent_sharp_corner():
  road=Way(1,tuple(ll(x,y) for x,y in [(-150,0),(0,0),(100,100),(100,250)]),False,'secondary','')
  result=find_match(build_tile((road,),'r'),fix())
  assert result is None or not result.corners

def test_tile_replacement_invalidates_cached_match_even_before_next_gps_message():
  tile=build_tile((bend_way(),),'old')
  tracker=MatchTracker()
  for i in range(52):
    f=fix(1+i*.1,x=-100+2*i)
    tracker.update(tile,f)
  assert tracker.qualified
  assert tracker.update(build_tile((bend_way(),),'new'),f) is None
  assert not tracker.qualified

def test_bad_tile_is_not_reparsed_at_control_frequency(tmp_path,monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.provider import OfflineProvider
  p=OfflineProvider(tmp_path)
  calls=[]
  monkeypatch.setattr(p,'_stamp',lambda path:(100,1,1))
  def broken(*args):
    calls.append(1)
    raise ValueError('Unsupported map')
  monkeypatch.setattr(p,'_load',broken)
  for i in range(20):
    assert p.update(fix(1+i*.05),1+i*.05) is None
  assert len(calls)<=1

def test_duplicate_vertex_preserves_road_and_competition_geometry():
  road=bend_way()
  repeated=replace(road,points=road.points[:10]+(road.points[9],)+road.points[10:])
  match=find_match(build_tile((repeated,),'r'),fix())
  assert match and match.corners
  assert find_match(build_tile((repeated,bend_way(200,10)),'r'),fix()) is None

def test_neighbour_edit_far_from_bend_does_not_change_local_corner_key():
  road=bend_way()
  neighbour=Way(2,tuple(ll(x,190) for x in range(-1000,61,10)),False,'secondary','')
  a=find_match(build_tile((road,neighbour),'a'),fix()).corners[0]
  changed=replace(neighbour,points=(ll(-1100,200),)+neighbour.points[1:])
  b=find_match(build_tile((road,changed),'b'),fix()).corners[0]
  assert a.key==b.key

def test_real_capnp_tile_is_decoded_once_across_local_reindex(tmp_path):
  from pathlib import Path
  import capnp
  import openpilot.starpilot.controls.lib.map_corner.provider as mod
  schema=capnp.load(str(Path(mod.__file__).with_name('offline.capnp')))
  road=bend_way()
  msg=schema.Offline.new_message(minLat=51,minLon=-.25,maxLat=51.25,maxLon=0,overlap=0,
    ways=[{'id':road.identity,'highwayClass':'secondary','nodes':[{'latitude':a,'longitude':b} for a,b in road.points]}])
  path=tmp_path/'tile';path.write_bytes(msg.to_bytes_packed())
  provider=mod.OfflineProvider(tmp_path)
  provider._load(path,fix())
  provider._load(path,fix(x=600))
  assert provider.decode_count==1 and provider.reindex_count==2

def test_corrupt_tile_is_decoded_once_per_file_stamp(tmp_path,monkeypatch):
  from openpilot.starpilot.controls.lib.map_corner.provider import OfflineProvider
  p=OfflineProvider(tmp_path);path=tmp_path/'bad';path.write_bytes(b'x'*100)
  calls=[]
  def corrupt(path): calls.append(1);raise RuntimeError('bad packed data')
  monkeypatch.setattr(p,'_decode',corrupt)
  for _ in range(3):
    with pytest.raises((RuntimeError,ValueError)):p._load(path,fix())
  assert len(calls)==1
