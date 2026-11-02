"""Real-owner/gateway regression for navigation without changed saved semantics."""
from dataclasses import replace
from unittest.mock import patch
import pytest
from openpilot.starpilot.galaxy.tests import test_settings as gateway_fixtures
from openpilot.starpilot.galaxy.settings import SettingsChanged
from openpilot.starpilot.ui.feature_settings_state import feature_parent_page, feature_parent_title
from openpilot.starpilot.ui.slc_menu import SLC_PAGES
from openpilot.starpilot.speed_limits import offset_document as od

@pytest.fixture
def setup():
  fixture = gateway_fixtures.SettingsGatewayTest(); fixture.setUp()
  yield fixture
  fixture.doCleanups()

def test_root_links_and_all_existing_rows_keep_exact_permissions(setup):
  gateway = setup.gateway; ctx = setup.context.value
  for malformed in (False, True):
    if malformed: setup.params.put('SLCOffsetSchedule', {'invalid': True}, block=True)
    with patch('openpilot.starpilot.ui.feature_settings_owner.menu_rows', side_effect=lambda page, rows: ('flat', rows)):
      original = gateway._state('slc', ctx).rows
    root = gateway._state('slc', ctx)
    assert {row.page for row in root.rows if row.page} == set(SLC_PAGES)
    assert {row.key for row in root.rows if row.key} == {'SpeedLimitController', 'ShowSpeedLimits'}
    collected = [row for row in root.rows if not row.page]
    for page in SLC_PAGES:
      assert feature_parent_page(page) == 'slc'
      assert feature_parent_title(page) == 'Speed Limit Controller'
      collected.extend(gateway._state(page, ctx).rows)
    assert len(collected) == len(original)
    assert sorted(map(repr, collected)) == sorted(map(repr, original))

def test_seven_offsets_edit_in_submenu_and_keep_si(setup):
  setup.params.put('SLCOffsetSchedule', od.to_value(od.adopt_legacy(False, (0.,)*7)), block=True)
  page = setup.page('slc/offsets')
  rows = [(i,row) for i,row in enumerate(page['rows']) if row['label'].endswith(' offset')]
  assert len(rows) == 7
  index,_ = rows[3]
  intent = setup.gateway.preview(page['view'], index, 0, setup.session, setup.generation, value=4)
  assert setup.params.get('SLCOffsetSchedule')['offsets_mps'][3] == 0
  assert setup.gateway.confirm(intent['intent'], setup.session, setup.generation)
  assert setup.params.get('SLCOffsetSchedule')['offsets_mps'][3] == pytest.approx(4*od.MPH_TO_MPS)

def test_invalid_offsets_repair_remains_reachable(setup):
  setup.params.put('SLCOffsetSchedule', {'invalid': True}, block=True)
  root = setup.page('slc')
  assert next(row for row in root['rows'] if row['label']=='Speed-limit offsets')['available']
  assert not next(row for row in root['rows'] if row['label']=='Speed Limit Controller')['available']
  page = setup.page('slc/offsets')
  index = next(i for i,row in enumerate(page['rows']) if row['label']=='Reset saved offsets')
  assert page['rows'][index]['available']
  intent = setup.gateway.preview(page['view'], index, 0, setup.session, setup.generation)
  assert setup.params.get('SLCOffsetSchedule') == {'invalid': True}
  assert setup.gateway.confirm(intent['intent'], setup.session, setup.generation)
  assert setup.params.get('SLCOffsetSchedule')['offsets_mps'] == [0.]*7

def test_submenu_saved_intent_rejects_vehicle_change(setup):
  page = setup.page('slc/timing')
  index = next(i for i,row in enumerate(page['rows']) if row['label']=='Higher-limit start distance')
  intent = setup.gateway.preview(page['view'], index, 1, setup.session, setup.generation)
  setup.context.value = replace(setup.context.value, cp=None, cp_raw=b'changed')
  with pytest.raises(SettingsChanged):setup.gateway.confirm(intent['intent'], setup.session, setup.generation)
