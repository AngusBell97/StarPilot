from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner import geometry as G
from openpilot.starpilot.controls.lib.map_corner.types import Way
from test_connected_tracking import road
from test_map_corner_provider import ll,fix

LINKS=('motorwayLink','trunkLink','primaryLink','secondaryLink','tertiaryLink')

def fixture(kind='motorwayLink',connect=False,hazard=''):
  link=Way(3,(ll(200,0),ll(200,14 if connect else 100)),True,kind,hazard)
  tile=G.build_tile((road(),road(2,y=14),link),'r')
  match=G.find_match(G.build_tile((road(),),'r'),fix(x=0))
  return tile,match

@pytest.mark.parametrize('kind',LINKS)
def test_known_links_complete_physical_search_without_adding_match_eligibility(kind):
  tile,m=fixture(kind)
  guard=G.ContinuityGuard(tile,fix(x=0),m)
  assert guard.disconnected(m,2,0) is True
  unsupported=replace(road(),road_class=kind)
  assert G.find_match(G.build_tile((unsupported,),'r'),fix(x=0)) is None

@pytest.mark.parametrize('kind',LINKS)
def test_connection_through_known_link_must_veto_disconnection(kind):
  tile,m=fixture(kind,connect=True)
  assert G.ContinuityGuard(tile,fix(x=0),m).disconnected(m,2,0) is False

@pytest.mark.parametrize('kind,hazard',[('unknown',''),('service',''),('livingStreet',''),('motorwayLink','hazard')])
def test_unknown_or_hazardous_intermediate_geometry_stays_unknown(kind,hazard):
  tile,m=fixture(kind,hazard=hazard)
  assert G.ContinuityGuard(tile,fix(x=0),m).disconnected(m,2,0) is None


def test_link_search_keeps_radius_and_shared_budget():
  tile,m=fixture(connect=True);g=G.ContinuityGuard(tile,fix(x=0),m)
  assert g.disconnected(m,2,0,limit=60) is True
  assert g.disconnected(m,2,0) is False
  g=G.ContinuityGuard(tile,fix(x=0),m);g.remaining=1;tile.continuity_cache.clear()
  assert g.disconnected(m,2,0) is None and g.remaining==0


def test_packed_decoder_uses_expected_link_class_spelling(tmp_path):
  from pathlib import Path
  import capnp
  from openpilot.starpilot.controls.lib.map_corner.provider import OfflineProvider
  import openpilot.starpilot.controls.lib.map_corner.provider as provider_module
  schema=capnp.load(str(Path(provider_module.__file__).with_name('offline.capnp')))
  msg=schema.Offline.new_message(minLat=51,minLon=-.25,maxLat=51.25,maxLon=0,overlap=0,
    ways=[dict(id=i+1,highwayClass=kind,nodes=[dict(latitude=51.1,longitude=-.1),dict(latitude=51.101,longitude=-.1)]) for i,kind in enumerate(LINKS)])
  path=tmp_path/'tile';path.write_bytes(msg.to_bytes_packed())
  provider=OfflineProvider(tmp_path);provider.schema=schema
  ways,*_=provider._decode(path)
  assert tuple(w.road_class for w in ways)==LINKS
  assert all(w.road_class in G.TOPOLOGY_CLASSES for w in ways)


def test_link_cannot_supply_competitor_or_curvature_targets():
  from openpilot.starpilot.controls.lib.map_corner.routes import preview_routes
  from test_map_corner_provider import bend_way
  main=road(end=0);link=replace(bend_way(2),road_class='motorwayLink')
  tile=G.build_tile((main,link),'r');m=G.find_match(G.build_tile((main,),'r'),fix(x=-200))
  guard=G.ContinuityGuard(tile,fix(x=-200),m)
  assert guard.competitor(m) is None
  assert not any(preview_routes(tile,m).options)
