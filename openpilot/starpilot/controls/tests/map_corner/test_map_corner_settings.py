import pytest
from openpilot.starpilot.controls.lib.map_corner.settings import MemorySettings
from openpilot.starpilot.controls.lib.map_corner.store import CornerStore

def test_onroad_management_cannot_create_or_clear_memory(tmp_path):
  ui=MemorySettings(lambda:True,tmp_path)
  assert not ui.action('clear')
  assert not (tmp_path/'corners.sqlite').exists()
  assert 'Park' in ui.status

def test_parked_clear_undo_and_busy_status_are_truthful(tmp_path):
  ui=MemorySettings(lambda:False,tmp_path)
  assert ui.action('refresh')
  assert ui.stats['corners']==0
  with CornerStore(tmp_path):
    assert not ui.action('clear')
    assert 'busy' in ui.status.lower()
  assert ui.action('clear')
  assert ui.stats['corners']==0

def test_offroad_is_checked_again_under_lock(tmp_path):
  checks=iter([False,True])
  ui=MemorySettings(lambda:next(checks),tmp_path)
  assert not ui.action('clear')
  assert 'Park' in ui.status

def test_clear_can_remove_corrupt_database(tmp_path):
  (tmp_path/'corners.sqlite').write_bytes(b'corrupt database')
  ui=MemorySettings(lambda:False,tmp_path)
  assert ui.action('clear')
  assert ui.stats['corners']==0
def test_refresh_without_memory_does_not_create_directory(tmp_path):
  root=tmp_path/'absent'
  ui=MemorySettings(lambda:False,root)
  assert ui.action('refresh')
  assert ui.stats['corners']==0 and not root.exists()


def test_parked_action_rechecks_exact_memory_under_store_lock(tmp_path):
  from openpilot.starpilot.controls.tests.map_corner.test_map_corner_provider import bend_way,fix
  from openpilot.starpilot.controls.lib.map_corner.geometry import build_tile,find_match
  corner=find_match(build_tile((bend_way(),),'r'),fix()).corners[0]
  with CornerStore(tmp_path) as store:store.observe(corner,corner.prior,'first',False)
  ui=MemorySettings(lambda:False,tmp_path);assert ui.action('refresh');expected=ui.snapshot()
  with CornerStore(tmp_path) as store:store.observe(corner,corner.prior,'second',False)
  assert not ui.action('clear',expected=expected)
  assert ui.stats['corners']==1 and 'changed' in ui.status
  assert ui.action('refresh') and ui.action('clear',expected=ui.snapshot())
  assert ui.stats['corners']==0
