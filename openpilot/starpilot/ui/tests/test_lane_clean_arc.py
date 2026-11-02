"""Display-only v6 style, signed capacity, cached selection and Galaxy preview."""
import math
import numpy as np
import pytest
from unittest.mock import Mock, patch
from openpilot.starpilot.ui.tests.test_lane_bar_widget import draw
from openpilot.starpilot.ui.tests.test_runtime_snapshot import RuntimeSnapshotAdapter, ui_fake, NOW
from openpilot.starpilot.ui.shell import ShellMode
from openpilot.starpilot.ui.onroad_torque import _clean_lane_fill
from openpilot.starpilot.ui.onroad_customization import default_document, LANE_BAR_WIDGET
from openpilot.starpilot.ui.layout_preview_renderer import LayoutPreviewRenderer

@pytest.mark.parametrize('profile', ['large', 'compact'])
@pytest.mark.parametrize('fraction', [-1., -.42, -.001, .001, .42, 1.])
@pytest.mark.parametrize('limit', [.0012, .0018, .0024])
def test_v6_signed_scale_and_fixed_native_track(profile, fraction, limit):
  old, old_text = draw(profile, correction=fraction*limit, limit=limit)
  new, text = draw(profile, correction=fraction*limit, limit=limit, lane_clean_arc=True)
  assert len(old)==2 and len(new)==3
  np.testing.assert_array_equal(new[0][0], old[0][0])
  assert text == []
  assert old_text[0][0] == f'{abs(fraction):.0%}'
  assert tuple(getattr(new[0][1], k) for k in 'rgba')==(220,227,234,63)
  assert tuple(getattr(new[1][1], k) for k in 'rgba')==(0,176,220,255)
  assert tuple(getattr(new[2][1], k) for k in 'rgba')==(220,227,234,140)
  origin=new[2][0][:,0].mean()
  assert (new[1][0][:,0].min()>=origin-.002 if fraction>0 else new[1][0][:,0].max()<=origin+.002)
  for points,_ in new[1:]:
    assert points[:,0].min()>=new[0][0][:,0].min()-.01
    assert points[:,0].max()<=new[0][0][:,0].max()+.01

@pytest.mark.parametrize('start,end', [(-6.,0.),(0.,6.),(0.,.001),(-.001,0.)])
def test_round_fill_has_exact_endpoints_and_stays_inside_track(start,end):
  p=_clean_lane_fill(1200.,7.,start,end).astype(float)
  angles=np.degrees(np.arctan2(p[:,0],-p[:,1]))
  assert abs(angles.min()-start)<1e-5 and abs(angles.max()-end)<1e-5
  r=np.linalg.norm(p,axis=1)
  assert r.min()>=1196.5-.001 and r.max()<=1203.5+.001
  for angle in (start,end):
    tip=p[np.isclose(angles,angle,atol=1e-7)]
    assert np.ptp(np.linalg.norm(tip,axis=1))<.001
  assert not _clean_lane_fill(1200.,7.,start,end).flags.writeable

@pytest.mark.parametrize('change',[{'correction':0},{'camera_available':False},{'lateral_active':False},{'limit':0}])
def test_v6_keeps_visibility_guards(change):
  assert draw('large',lane_clean_arc=True,**change)[0]==[]

def test_v6_widget_stays_default_off():
  assert draw('large',document=default_document(),lane_clean_arc=True)[0]==[]

def test_selection_updates_on_existing_one_second_refresh():
  ui=ui_fake();ui.params.values['LaneCenteringVersion']=6
  owner=RuntimeSnapshotAdapter(ui)
  owner.build(ShellMode.ONROAD, now_ns=NOW)
  assert owner._lane_clean_arc
  ui.params.values['LaneCenteringVersion']='5'
  owner.build(ShellMode.ONROAD, now_ns=NOW+100_000_000)
  assert owner._lane_clean_arc
  owner.build(ShellMode.ONROAD, now_ns=NOW+1_000_000_000)
  assert not owner._lane_clean_arc
  ui.params.values['LaneCenteringVersion']=b'6'
  owner.invalidate_appearance();owner.build(ShellMode.ONROAD, now_ns=NOW+1_100_000_000)
  assert owner._lane_clean_arc

@pytest.mark.parametrize('profile',['large','compact'])
@pytest.mark.parametrize('art',[False,True,'backdrop'])
def test_editor_uses_selected_style(profile,art):
  with patch('openpilot.starpilot.ui.layout_preview_renderer.rl.is_window_ready',return_value=True), patch('openpilot.starpilot.ui.layout_preview_renderer._Canvas') as canvas:
    selected=[True]; renderer=LayoutPreviewRenderer(clean_lane_bar=lambda:selected[0])
    request={'profile':profile,'document':default_document(),'scene':'engaged'}
    if art:request['art']=art
    for value in (True,False,True):
      selected[0]=value;renderer(request)
      output=canvas.return_value.render_widget_art if art is True else canvas.return_value.render
      assert output.call_args.args[0].lane_clean_arc is value


from openpilot.starpilot.ui.tests.test_lane_v4_settings import f

def test_native_params_version_drives_real_preview_callback(f):
  from openpilot.starpilot.ui.layout_preview_runtime import LayoutPreviewRuntime
  with patch('openpilot.starpilot.ui.layout_preview_runtime.PreviewService'):
    runtime = LayoutPreviewRuntime(f.params, 'large', authority=Mock())
    try:
      for version in (4, 5, 6, 5, 6, 4):
        f.params.put('LaneCenteringVersion', version, block=True)
        assert type(f.params.get('LaneCenteringVersion')) is int
        assert runtime.renderer._clean_lane_bar() is (version == 6)
    finally:
      runtime.close()
