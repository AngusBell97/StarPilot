"""Real Params, owner and Galaxy transactions for live v4/v5 selection."""
from dataclasses import replace
import pytest
from openpilot.starpilot.ui.tests.test_lane_v4_settings import f,edit
from openpilot.starpilot.lateral.lane_versions import VERSION,INITIALIZED,KEY_MAP,read_selected
from openpilot.starpilot.lateral.lane_runtime import read_settings
from openpilot.starpilot.galaxy.settings import SettingsChanged


@pytest.mark.parametrize('version', ['v5', 'v6'])
@pytest.mark.parametrize('parked', [True, False])
def test_strength_galaxy_display_grid_saves_and_reloads(f, version, parked):
 f.params.put('LaneCenteringStrength', 1.5, block=True)
 edit(f, 'lane', 'Lane centering version', version)
 f.context.value = replace(f.context.value, parked=parked)
 key = 'LaneCentering' + version.upper() + 'Strength'
 # Exact integer percentages emitted by Galaxy's 1-10 presentation.
 for percent in (33, 50, 67, 83, 100, 120, 140, 160, 180, 200):
  page = f.page('lane/preferences')
  row = next(r for r in page['rows'] if r['label'] == 'Lane Centering Strength')
  assert (row['minimum'], row['maximum'], row['step']) == (33, 200, 1)
  assert row['displayScale'] == 'lane-strength'
  edit(f, 'lane/preferences', 'Lane Centering Strength', percent)
  assert f.params.get(key) == pytest.approx(max(.5, percent * 1.5 / 100))
  assert read_selected(f.params)[1].strength == pytest.approx(max(.5, percent * 1.5 / 100))
  row = next(r for r in f.page('lane/preferences')['rows'] if r['label'] == 'Lane Centering Strength')
  assert row['value'] == str(percent)
 assert f.params.get('LaneCenteringStrength') == 1.5
 for percent in (32, 201, 100.5):
  with pytest.raises(SettingsChanged):
   edit(f, 'lane/preferences', 'Lane Centering Strength', percent)
 assert f.params.get(key) == 3.


@pytest.mark.parametrize('version', ['v5', 'v6'])
def test_strength_galaxy_keeps_stale_saved_value_guard(f, version):
 f.params.put('LaneCenteringStrength', 1.5, block=True)
 edit(f, 'lane', 'Lane centering version', version)
 page = f.page('lane/preferences')
 index = next(i for i, r in enumerate(page['rows']) if r['label'] == 'Lane Centering Strength')
 intent = f.gateway.preview(page['view'], index, 0, f.session, f.generation, value=120)
 key = 'LaneCentering' + version.upper() + 'Strength'
 assert f.params.get(key) == 1.5
 f.params.put(key, 2.1, block=True)
 assert not f.gateway.confirm(intent['intent'], f.session, f.generation)
 assert f.params.get(key) == 2.1


def test_live_version_select_first_snapshot_and_independent_tunes(f):
 f.params.put('LaneCenteringStrength',1.5,block=True)
 f.params.put('LaneCenteringV4ResponseMin',.2,block=True)
 original=read_settings(f.params)
 f.context.value=replace(f.context.value,parked=False)
 edit(f,'lane','Lane centering version','v5')
 assert f.params.get(VERSION)==5 and f.params.get(INITIALIZED) is True
 version,settings=read_selected(f.params)
 assert version==5 and settings.response_min==.2 and settings.clearance==.2 and settings.single_clearance==.45
 edit(f,'lane/preferences','Maximum preview time',2.5)
 edit(f,'lane/preferences/smoothing','Boundary smoothing time',.4)
 assert read_selected(f.params)[1].preview_time==2.5
 assert read_selected(f.params)[1].smoothing_tau==.4 and read_settings(f.params)==original
 edit(f,'lane','Lane centering version','v4')
 assert read_selected(f.params)==(4,original)
 edit(f,'lane','Lane centering version','v5')
 assert read_selected(f.params)[1].smoothing_tau==.4


def test_old_v4_intent_does_not_write_after_version_switch(f):
 page=f.page('lane/preferences/smoothing');index=next(i for i,r in enumerate(page['rows']) if r['label']=='Boundary smoothing time')
 intent=f.gateway.preview(page['view'],index,0,f.session,f.generation,value=.4)
 edit(f,'lane','Lane centering version','v5')
 assert not f.gateway.confirm(intent['intent'],f.session,f.generation)
 assert f.params.get('LaneCenteringV4SmoothingTime') is None


def test_v5_native_controls_and_minmax_units(f):
 edit(f,'lane','Lane centering version','v5')
 owner=f.gateway._owner(f.context.value)
 native=owner.snapshot('lane',parked=True,system_long=True,lateral_context=True,metric=True)
 assert [r.key for r in native.rows]==['LaneCentering','LaneCenteringV5PauseOnSignal','LaneCenteringV5Offset','LaneCenteringV5E2EAuthority','LaneCenteringV5Strength']
 rows=f.page('lane/preferences/smoothing')['rows']
 pairs=[r['rangePair'] for r in rows if r.get('rangePair')]
 assert {p['end'] for p in pairs}=={'min','max'}
 with pytest.raises(SettingsChanged):edit(f,'lane/preferences/smoothing','Minimum correction response time',.9)
 f.params.put_bool('IsMetric',False,block=True)
 rows=f.page('lane/activation/geometry')['rows']
 assert next(r for r in rows if r['label']=='Car body width')['unit']=='in'
 edit(f,'lane/activation/geometry','Two-boundary minimum clearance',18)
 assert read_selected(f.params)[1].clearance==pytest.approx(.4572)


def test_v5_disabling_common_native_master_stops_both(f):
 edit(f,'lane','Lane centering version','v5')
 edit(f,'lane','Enable Lane Centering','Off')
 assert not read_settings(f.params).enabled and not read_selected(f.params)[1].enabled


@pytest.mark.parametrize('version', ['v4', 'v5', 'v6'])
def test_removed_target_line_has_no_menu_or_edit_route(f, version):
 from openpilot.starpilot.ui.feature_settings_state import FeatureSettingsRequest
 from openpilot.starpilot.ui import lane_versions_menu
 key='LaneCenteringShowTargetPath'
 f.params.put_bool(key, True, block=True)
 edit(f,'lane','Lane centering version',version)
 before=read_selected(f.params)
 owner=f.gateway._owner(f.context.value)
 for galaxy in (False,True):
  state=owner.snapshot('lane',parked=True,system_long=True,lateral_context=True,metric=True,lane_v4=galaxy)
  assert all(row.key != key for row in state.rows)
 request=FeatureSettingsRequest(key=key,expected=b'1',value='Off',vehicle_fingerprint=owner.vehicle_fingerprint())
 assert lane_versions_menu.error(owner,request)
 assert not owner.apply(request)
 assert f.params.get_bool(key) and read_selected(f.params)==before



def test_v6_seeds_selected_tune_and_keeps_versions_independent(f):
 edit(f,'lane','Lane centering version','v5')
 edit(f,'lane/preferences/smoothing','Boundary smoothing time',.31)
 # Hidden single-boundary values remain stored and copy unchanged to v6.
 f.params.put('LaneCenteringV5SingleClearance',.46,block=True)
 source=read_selected(f.params)[1]
 edit(f,'lane','Lane centering version','v6')
 assert read_selected(f.params)==(6,source)
 edit(f,'lane/preferences/smoothing','Boundary smoothing time',.22)
 assert read_selected(f.params)[1].smoothing_tau==.22
 edit(f,'lane','Lane centering version','v5')
 assert read_selected(f.params)==(5,source)
 edit(f,'lane','Lane centering version','v6')
 assert read_selected(f.params)[1].smoothing_tau==.22


def test_v6_rejects_stale_v5_intent_and_retains_bounds(f):
 edit(f,'lane','Lane centering version','v5')
 page=f.page('lane/preferences/smoothing');index=next(i for i,r in enumerate(page['rows']) if r['label']=='Boundary smoothing time')
 intent=f.gateway.preview(page['view'],index,0,f.session,f.generation,value=.4)
 edit(f,'lane','Lane centering version','v6')
 assert not f.gateway.confirm(intent['intent'],f.session,f.generation)
 with pytest.raises(SettingsChanged):edit(f,'lane/preferences/smoothing','Minimum correction response time',.9)
 f.params.put_bool('IsMetric',False,block=True)
 rows=f.page('lane/activation/geometry')['rows']
 assert next(r for r in rows if r['label']=='Car body width')['unit']=='in'
