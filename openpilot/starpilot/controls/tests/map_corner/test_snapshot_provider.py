from pathlib import Path
import hashlib
import json
import pytest
from openpilot.starpilot.controls.lib.map_corner.snapshot_provider import SnapshotProvider
from openpilot.starpilot.controls.lib.map_corner.provider import tile_relative_path
from openpilot.starpilot.controls.tests.map_corner.test_map_corner_provider import bend_way,fix


def package(root,*,name='one'):
  import capnp
  from openpilot.starpilot.controls.lib.map_corner import provider
  schema=capnp.load(str(Path(provider.__file__).with_name('offline.capnp')))
  road=bend_way();f=fix();relative=tile_relative_path(f.latitude,f.longitude)
  tile=schema.Offline.new_message(minLat=51.,minLon=-.25,maxLat=51.25,maxLon=0.,overlap=.01)
  ways=tile.init('ways',1);way=ways[0];way.id=road.identity;way.highwayClass='secondary';way.lanes=2
  nodes=way.init('nodes',len(road.points))
  for node,(lat,lon) in zip(nodes,road.points):node.latitude=lat;node.longitude=lon
  data=tile.to_bytes_packed()
  receipt={'version':1,'sourceKind':'local-pbf','sourceVerified':False,
    'tileSchema':'offline.capnp:0xda3a0d9284ca402f',
    'attribution':'© OpenStreetMap contributors; https://www.openstreetmap.org/copyright (ODbL)',
    'inputs':[{'name':name+'.pbf','size':1,'sha256':'a'*64}],
    'tiles':[{'path':relative.as_posix(),'size':len(data),'sha256':hashlib.sha256(data).hexdigest()}]}
  raw=json.dumps(receipt,sort_keys=True).encode();generation=hashlib.sha256(raw).hexdigest()
  directory=root/'generations'/generation;path=directory/relative;path.parent.mkdir(parents=True,exist_ok=True)
  path.write_bytes(data);(directory/'receipt.json').write_bytes(raw)
  (root/'current.json').write_text(json.dumps({'version':1,'generation':generation}))
  return path,generation


def test_selected_generation_decodes_exact_original_geometry(tmp_path):
  path,generation=package(tmp_path);p=SnapshotProvider(tmp_path)
  assert p._select(1.) and p.generation==generation
  ways,_,_,_,_,_=p._decode(path)
  assert ways[0].identity==bend_way().identity and ways[0].points==bend_way().points
  assert p._load(path)[0].roads


@pytest.mark.parametrize('bad',['bytes','symlink','size','receipt','selector'])
def test_tampered_snapshot_cannot_supply_geometry(tmp_path,bad):
  path,_=package(tmp_path);p=SnapshotProvider(tmp_path)
  if bad=='receipt':(path.parents[2]/'receipt.json').write_text('{}')
  if bad=='selector':(tmp_path/'current.json').write_text('{"version":1,"generation":"../old"}')
  if bad in ('receipt','selector'):
    assert not p._select(1.);return
  assert p._select(1.)
  if bad=='bytes':
    data=path.read_bytes();path.write_bytes(bytes([data[0]^1])+data[1:])
  elif bad=='size':path.write_bytes(b'changed')
  else:
    original=path.with_name('original');path.rename(original);path.symlink_to(original.name)
  with pytest.raises((ValueError,OSError)):p._decode(path)


def test_reselection_resets_qualification_but_geometry_keeps_memory_identity(tmp_path):
  path,first=package(tmp_path);p=SnapshotProvider(tmp_path);assert p._select(1.)
  old=p._load(path)[0];p.tracker.qualified=True;p.decoded[path]=('cached',)
  path,second=package(tmp_path,name='two');assert first!=second
  assert p._select(2.) and p.generation==second and not p.tracker.qualified and not p.decoded
  new=p._load(path)[0]
  from openpilot.starpilot.controls.lib.map_corner.geometry import find_match
  from openpilot.starpilot.controls.lib.map_corner.geometry import build_tile
  # _load returns a border tile, unindexed until its neighbours are merged.
  # At this fixture border, single-tile matching must remain unavailable.
  assert find_match(old,fix()) is None
  old=build_tile(tuple(r.way for r in old.roads.values()),old.provenance)
  new=build_tile(tuple(r.way for r in new.roads.values()),new.provenance)
  assert find_match(old,fix()).corners[0].key==find_match(new,fix()).corners[0].key


def test_no_legacy_fallback_and_private_namespace(monkeypatch,tmp_path):
  (tmp_path/'50').mkdir()
  p=SnapshotProvider(tmp_path);assert not p._select(1.)
  monkeypatch.setenv('OPENPILOT_PREFIX','sp031-test')
  assert SnapshotProvider().package_root==Path('/data/media/0/starpilot-sp031-test/maps/offline')
