from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from openpilot.starpilot.lateral.tests.test_lane_path_wire import setup
from openpilot.selfdrive.ui.onroad import model_renderer as large
from openpilot.selfdrive.ui.mici.onroad import model_renderer as small

@pytest.mark.parametrize('module',[large,small])
def test_removed_overlay_preserves_model_path_and_lanes(module,monkeypatch):
 c,event,sm=setup();model=sm['modelV2'];model.acceleration=NS(x=[0.]*33)
 sm.update(carOutput=NS(actuatorsOutput=NS(torque=0.)),selfdriveState=NS(experimentalMode=False),extrinsicsCalibration=NS(height=[1.2]))
 sm.recv_frame.update(extrinsicsCalibration=5);sm.valid.update(radarState=False)
 sm.updated={'modelV2':True,'carParams':False,'radarState':False}
 clock=[1_000_000_000];params=NS(get_bool=lambda key:True)
 monkeypatch.setattr(module,'ui_state',NS(sm=sm,started_frame=0,display_time_ns=lambda:clock[0],params=params))
 view=object.__new__(module.ModelRenderer)
 view._path=large.ModelPoints();view._lane_lines=[large.ModelPoints() for _ in range(4)];view._road_edges=[large.ModelPoints() for _ in range(2)]
 view._car_space_transform=np.array([[580,-900,0],[220,0,900],[1,0,0]],dtype=np.float32)
 view._rect=NS(x=0,y=0,width=1160,height=800);view._longitudinal_control=False;view.lead_display=None
 view._torque_filter=Mock();view._acceleration_x_filter=Mock();view._acceleration_x_filter2=Mock()
 view._lead_indicator_enabled=False;view._visual_status=lambda:small.UIStatus.ENGAGED
 monkeypatch.setattr(module,"draw_polygon",Mock())
 view._draw_lane_lines=Mock();view._draw_path=Mock();view._draw_lead_indicator=Mock();view._transform_dirty=True
 rect=NS(x=0,y=0,width=1160,height=800);view._render(rect)
 projected=view._path.projected_points.copy();lanes=[p.projected_points.copy() for p in view._lane_lines]
 assert len(projected)>4 and np.isfinite(projected).all()
 np.testing.assert_allclose(view._path.raw_points,np.array([model.position.x,model.position.y,model.position.z]).T)
 assert not hasattr(view, '_lane_path_view') and not hasattr(view, '_lane_target_polygon')
 assert not module.draw_polygon.called
 assert view._draw_path.call_count == 1 and view._draw_lane_lines.call_count == 1
 module.draw_polygon.reset_mock()
 # Missing target clears only the line; normal path and lane polygons remain identical.
 event.starpilotLateralState.lanePath.valid=False;sm.updated['modelV2']=False;clock[0]+=10_000_000
 view._render(rect)
 np.testing.assert_allclose(view._path.raw_points,np.array([model.position.x,model.position.y,model.position.z]).T)
 np.testing.assert_array_equal(projected,view._path.projected_points)
 assert not module.draw_polygon.called
 assert view._draw_path.call_count == 2 and view._draw_lane_lines.call_count == 2
 for before,after in zip(lanes,view._lane_lines):np.testing.assert_array_equal(before,after.projected_points)
 assert model.position.y==[0.]*33
