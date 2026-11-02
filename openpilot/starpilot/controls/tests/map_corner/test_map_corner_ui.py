"""Shared 7.0 row/write contract with file-backed preferences and real memory."""
from types import SimpleNamespace as NS
from openpilot.starpilot.ui.map_corner_feature import MapCornerFeature
from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
from openpilot.starpilot.controls.lib.map_corner.settings import MemorySettings
from openpilot.starpilot.controls.lib.map_corner.store import CornerStore
from test_map_corner_store import corner
from openpilot.starpilot.tests.port_settings_fixture import Saved
from opendbc.car import gen_empty_fingerprint
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR

def test_actual_shared_rows_defaults_writes_and_parked_memory(tmp_path):
  params=Saved(tmp_path); parked=[True]
  fp=gen_empty_fingerprint();fp[2][0x489]=8
  cp=CarInterface.get_params(CAR.TESLA_MODEL_3,fp,[],False,False,False)
  cp.openpilotLongitudinalControl=True
  owner=NS(params=params,vehicle_params=lambda:cp,vehicle_fingerprint=lambda:'car',authority=lambda _:parked[0])
  feature=MapCornerFeature(owner);feature.memory=MemorySettings(lambda:not parked[0],tmp_path/'memory')
  def rows():return {r.key:r for r in feature.rows(parked[0],False)}
  def request(key,value='',confirmation=True):
    row=rows()[key]
    return FeatureSettingsRequest(key,row.source,value,confirmation=confirmation,vehicle_fingerprint='car')
  assert rows()['MapCornerSpeed'].value=='Off'
  assert feature.apply(request('MapCornerSpeed','On'))
  assert rows()['MapCornerSpeed'].value=='On'
  assert feature.apply(request('LearnCornerSpeeds','On'))
  with CornerStore(tmp_path/'memory') as store:store.observe(corner(),(14,14,14),'one',False)
  forget=rows()['map_memory:forget:0']
  assert not forget.choices and forget.label.startswith('Forget ')
  req=request('map_memory:forget:0','confirm')
  parked[0]=False
  assert not feature.apply(req)
  parked[0]=True
  assert feature.apply(req)
  with CornerStore(tmp_path/'memory') as store:assert store.stats()['corners']==0
  stale=request('map_memory:clear')
  with CornerStore(tmp_path/'memory') as store:store.observe(corner('new'),(14,14,14),'two',False)
  assert not feature.apply(stale)
  assert feature.apply(request('map_memory:clear'))
