"""SP-016 independent default-off editor widget and cap-use rendering contract."""
import copy
import json
from dataclasses import replace
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import pytest
from openpilot.starpilot.ui.raylib import rl
from openpilot.starpilot.ui.onroad_customization import (LANE_BAR_WIDGET as KEY, PROFILES, DEFAULT_WIDGET_ORDER,
  RUNTIME_PREVIEW_WIDGETS, default_document, validate_document, decode_document)
from openpilot.starpilot.ui.onroad_torque import draw_lane_bar
from openpilot.starpilot.ui.onroad_torque_geometry import lane_bar_footprint, maximum_footprint
from openpilot.starpilot.ui.layout_preview_renderer import sample_state
from openpilot.starpilot.system.starport.projection_layout import (
  default_layout_for_viewport, validate_layout_for_viewport, layout_metadata_for_viewport, projection_customization)


@pytest.mark.parametrize("profile", ["large", "compact"])
def test_old_layout_adds_disabled_widget_without_changing_other_values_or_order(profile):
  doc=default_document()
  assert not doc["layouts"][profile][KEY]["enabled"]
  for layout in doc["layouts"].values():layout.pop(KEY)
  doc["layouts"][profile]["torque_bar"]["x"]+=10
  order=[k for k in DEFAULT_WIDGET_ORDER[profile] if k!=KEY]
  doc["widgetOrder"]={profile:list(reversed(order))}
  before=copy.deepcopy(doc)
  new=decode_document(json.dumps(doc).encode())
  assert doc==before
  for p,layout in before["layouts"].items():
    assert not new["layouts"][p][KEY]["enabled"]
    assert all(new["layouts"][p][k]==v for k,v in layout.items())
  assert new["widgetOrder"][profile]==before["widgetOrder"][profile]+[KEY]
  assert KEY in RUNTIME_PREVIEW_WIDGETS[profile]


@pytest.mark.parametrize("viewport", [(1860,1080),(2880,1080),(4096,2160)])
def test_projection_optional_migration_and_independent_placement(viewport):
  doc=default_layout_for_viewport(viewport);doc['widgets'].pop(KEY)
  doc['widgetOrder']=[k for k in layout_metadata_for_viewport(viewport)['widgetOrder'] if k!=KEY]
  new=validate_layout_for_viewport(doc,viewport)
  assert not new['widgets'][KEY]['enabled']
  assert [k for k in new['widgetOrder'] if k!=KEY]==doc['widgetOrder']
  assert all(new['widgets'][k]==v for k,v in doc['widgets'].items())
  new['widgets'][KEY].update(enabled=True,x=100,y=200)
  projected=projection_customization(validate_layout_for_viewport(new,viewport),default_document())
  _,_,w,h=lane_bar_footprint(30,30,viewport[0]-60,viewport[1]-60,viewport[0])
  calls,_=draw('large',projected,viewport=viewport)
  assert len(calls)==2
  points=calls[0][0]
  assert points[:,0].min()>=100 and points[:,0].max()<=100+w
  assert points[:,1].min()>=200 and points[:,1].max()<=200+h
  assert default_document()['layouts']['large'][KEY]['enabled'] is False


def draw(profile,document=None,*,torque=0,correction=.0009,limit=.0018,viewport=None,**changes):
  if document is None:
    document=default_document();document['layouts'][profile][KEY]['enabled']=True
  state=replace(sample_state('engaged',document),**(dict(camera_available=True,visual_preview=None,lane_correction=correction,lane_correction_limit=limit)|changes))
  bounds=PROFILES[profile]['bounds'];sw=PROFILES[profile]['width']
  rect=rl.Rectangle(*(bounds[k] for k in ('x','y','width','height')))
  if viewport:rect=rl.Rectangle(30,30,viewport[0]-60,viewport[1]-60);sw=viewport[0]
  calls=[];texts=[]
  fonts=NS(measure=lambda text,role,size:NS(width=len(text)*size*.6,height=size),draw=lambda *args:texts.append(args))
  with patch('openpilot.starpilot.ui.onroad_torque.draw_polygon',side_effect=lambda rect,points,**kw:calls.append((points.copy(),kw['color']))):
    draw_lane_bar(rect,state,sw,torque,fonts,projection=viewport is not None)
  return calls,texts


@pytest.mark.parametrize('profile',['large','compact'])
def test_fixed_track_has_independent_placement_and_fits_its_editor_box(profile):
  base,text=draw(profile)
  for torque in (-1,-.5,0,.5,1):
    calls,_=draw(profile,torque=torque)
    np.testing.assert_array_equal(calls[0][0],base[0][0])
  doc=default_document();doc['layouts'][profile][KEY].update(enabled=True)
  doc['layouts'][profile]['torque_bar']['enabled']=False
  doc['layouts'][profile]['torque_bar']['x']+=10
  calls,_=draw(profile,doc);np.testing.assert_array_equal(calls[0][0],base[0][0])
  doc['layouts'][profile][KEY]['x']+=10;doc['layouts'][profile][KEY]['y']-=40
  calls,_=draw(profile,validate_document(doc))
  for old,new in zip(base,calls):np.testing.assert_allclose(new[0],old[0]+np.array([10,-40],dtype=np.float32),atol=0.0001,rtol=0)
  widget=PROFILES[profile]['widgets'][KEY];x,y=widget['default']['x'],widget['default']['y']
  for points,_ in base:
    assert points[:,0].min()>=x and points[:,0].max()<=x+widget['width']
    assert points[:,1].min()>=y and points[:,1].max()<=y+widget['height']
  assert y+widget['height'] < PROFILES[profile]['widgets']['torque_bar']['default']['y']
  assert all(y<=args[4]<=y+widget['height'] for args in text)
  assert base[0][1].a==63 and base[1][1].a==229


@pytest.mark.parametrize('cap',[.0012,.0018,.0024])
@pytest.mark.parametrize('fraction',[-1,-.25,.25,1])
def test_live_cap_and_signed_fill_unchanged(cap,fraction):
  calls,texts=draw('compact',correction=cap*fraction,limit=cap)
  assert texts[-1][0]==f'{abs(fraction):.0%}'
  points=calls[1][0];mid=(calls[0][0][:,0].min()+calls[0][0][:,0].max())/2
  assert points[:,0].max()<=mid+.001 if fraction<0 else points[:,0].min()>=mid-.001


@pytest.mark.parametrize('changes',[dict(correction=0),dict(correction=float('nan')),dict(limit=0),dict(camera_available=False),dict(lateral_active=False)])
def test_runtime_visibility_guards(changes):
  assert draw('compact',**changes)[0]==[]


def test_disabled_default_and_missing_legacy_entry_do_not_draw():
  doc=default_document()
  assert draw('compact',doc)[0]==[]
  doc['layouts']['compact'].pop(KEY)
  assert draw('compact',doc)[0]==[]


def test_preview_sample_is_explicit_without_live_camera():
  doc=default_document();doc['layouts']['compact'][KEY]['enabled']=True
  state=sample_state('engaged',doc)
  assert not state.camera_available and state.visual_preview.layout_sample
  assert state.lane_correction/state.lane_correction_limit==.5
  with patch('openpilot.starpilot.ui.onroad_torque.draw_polygon') as painter:
    draw_lane_bar(rl.Rectangle(0,0,476,240),state,536)
    assert painter.call_count==2


def test_editor_migration_and_save_keep_profiles_independent_and_check_authority(tmp_path):
  from openpilot.common.params import Params
  from openpilot.starpilot.galaxy.onroad_layout import LayoutChanged, OnroadLayoutOwner
  from openpilot.starpilot.ui.onroad_customization import PARAM_KEY
  from pathlib import Path
  params=Params(str(tmp_path));parked=[True]
  old=default_document()
  for layout in old['layouts'].values():layout.pop(KEY)
  raw=json.dumps(old).encode();path=Path(params.get_param_path(PARAM_KEY));path.write_bytes(raw)
  owner=OnroadLayoutOwner(params,lambda:parked[0]);snapshot=owner.snapshot()
  assert snapshot['valid'] and path.read_bytes()==raw
  doc=copy.deepcopy(snapshot['document']);doc['layouts']['compact'][KEY].update(enabled=True,x=90,y=90)
  payload={'revision':snapshot['revision'],'document':doc}
  result=owner.save(payload,session_valid=lambda:True)
  assert result['document']==doc and params.get(PARAM_KEY)==doc
  assert not result['document']['layouts']['large'][KEY]['enabled']
  with pytest.raises(LayoutChanged):owner.save(payload,session_valid=lambda:True)
  fresh={'revision':result['revision'],'document':doc}
  parked[0]=False
  with pytest.raises(LayoutChanged):owner.save(fresh,session_valid=lambda:True)
  parked[0]=True
  with pytest.raises(LayoutChanged):owner.save(fresh,session_valid=lambda:False)
  assert params.get(PARAM_KEY)==doc


@pytest.mark.parametrize('viewport',[(1860,1080),(2880,1080),(4096,2160)])
def test_projection_sprite_fits_bounded_sheet_and_retains_display_geometry(viewport):
  from openpilot.starpilot.ui.layout_preview_renderer import widget_art_size
  _,_,width,height=lane_bar_footprint(30,30,viewport[0]-60,viewport[1]-60,viewport[0])
  sw,sh=widget_art_size(width,height)
  assert 0<sw<=1024 and 0<sh<=height
  assert abs(sh/sw-height/width)<=1/sw
  assert width<=4096 and height<=2160


@pytest.mark.parametrize('viewport',[None,(1860,1080),(2880,1080),(4096,2160)])
def test_complete_native_sprite_sheet_stays_within_transport_bounds(viewport):
  from openpilot.starpilot.ui import layout_preview_renderer as renderer
  from openpilot.starpilot.ui.presentation import Profile
  from openpilot.starpilot.ui.layout_preview_transport import validate_png
  import struct
  profile=Profile.LARGE if viewport else Profile.COMPACT
  native=default_document()
  if viewport:
    layout=default_layout_for_viewport(viewport);layout['widgets'][KEY]['enabled']=True
    native=projection_customization(layout,native);positions=layout['widgets']
  else:
    native['layouts']['compact'][KEY]['enabled']=True;positions=native['layouts']['compact']
  canvas=renderer._Canvas.__new__(renderer._Canvas)
  canvas.profile=profile;canvas.viewport=viewport;canvas.fonts=object();canvas.target=NS(texture=object())
  canvas.view=NS(_ordered_widgets=lambda *args,**kwargs:None);canvas._settle=lambda state:None
  def crop(image,rect):image.data=bytes(int(rect.width*rect.height)*4)
  def resize(image,width,height):image.data=bytes(width*height*4)
  fake=NS(Rectangle=rl.Rectangle,BLANK=None,PixelFormat=NS(PIXELFORMAT_UNCOMPRESSED_R8G8B8A8=0),
    BlendMode=NS(BLEND_CUSTOM_SEPARATE=0),rl_set_blend_factors_separate=lambda *args:None,
    begin_texture_mode=lambda *args:None,end_texture_mode=lambda:None,clear_background=lambda *args:None,
    begin_blend_mode=lambda *args:None,end_blend_mode=lambda:None,load_image_from_texture=lambda *args:NS(data=b''),
    image_format=lambda *args:None,image_flip_vertical=lambda *args:None,image_crop=crop,image_resize=resize,
    unload_image=lambda *args:None)
  with patch.object(renderer,'rl',fake):png=canvas.render_widget_art(sample_state('engaged',native),positions)
  assert validate_png(png)==png
  width,height=struct.unpack('!II',png[16:24]);assert width<=1024 and height<=1440
