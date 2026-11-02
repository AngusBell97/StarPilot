from dataclasses import replace
import numpy as np
import pytest
from openpilot.starpilot.lateral.lane_v6 import LaneCenteringV6Controller
from openpilot.starpilot.lateral.tests.test_lane_v5 import obs, scene
from openpilot.starpilot.lateral.lane_path import CubicFit


def test_fit_cache_only_reuses_exact_coordinates_and_recomputes_y(monkeypatch):
 original=np.linalg.svd;calls=[]
 def svd(*args,**kwargs): calls.append(1);return original(*args,**kwargs)
 monkeypatch.setattr(np.linalg,'svd',svd)
 fit=CubicFit();x=np.linspace(-1.,1.,9);y=x*x
 a=fit(x,y,3);projection=fit.projection
 b=fit(x.copy(),y+.1,3)
 assert len(calls)==1 and fit.projection is projection and not np.array_equal(a,b)
 xx=x.copy();xx[3]=np.nextafter(xx[3],np.inf);fit(xx,y,3)
 assert len(calls)==2 and fit.projection is not projection


def test_controller_fit_cache_lifecycle_and_live_settings():
 c=LaneCenteringV6Controller(clock=lambda:0.)
 for i in range(6):c.update(obs(i,settings=replace(obs().settings,curve_adaptation=True)))
 fit=c._bend_fit;assert fit.projection is not None
 c.update(obs(6,settings=replace(obs().settings,response_tau=.4,curve_adaptation=True)))
 assert c._bend_fit is fit
 c.update(obs(7,turn_signal_active=True));assert c._bend_fit is not fit and c._bend_fit.x is None
 c.reset();assert c._bend_fit.x is None


@pytest.mark.parametrize('elapsed,reason',[(.029999,'qualified'),(.03,'qualified'),(.030001,'v6_compute_budget')])
def test_exact_30ms_deadline(elapsed,reason):
 values=iter([0.,elapsed,elapsed]);c=LaneCenteringV6Controller(clock=lambda:next(values))
 assert c.update(obs()).reason==reason


@pytest.mark.parametrize('kind',['shared','different','missing','signed_zero'])
def test_original_knots_retained(kind):
 model=scene();xx=np.array(model.laneLines[1].x);xx[10]=15.375;xx.sort()
 if kind=='signed_zero':xx[0]=-0.
 model.laneLines[1].x=xx.tolist();model.laneLines[2].x=xx.tolist()
 if kind=='different':model.laneLines[2].x[11]+=.01
 if kind=='missing':model.laneLineProbs[2]=0.;model.roadEdgeStds[1]=.1;model.roadEdges[1].x=xx.tolist()
 model.laneLines[1].y[10]=-.1
 (model.roadEdges[1] if kind=='missing' else model.laneLines[2]).y[10]=.1
 c=LaneCenteringV6Controller(clock=lambda:0.);c.update(obs(model=model));assert c.planned_path is not None
 at=np.searchsorted(c.planned_path.x,xx[10]);assert c.diagnostic['v6_regions'][at]==0 and c.diagnostic['v6_regions'][at-1]==0


def test_diagnostics_preserve_custom_settings_equality():
 from openpilot.starpilot.lateral.lane_centering import LaneCenteringSettings
 from openpilot.starpilot.lateral.tests.test_lane_diagnostics import setup,send
 from openpilot.starpilot.lateral.lane_diagnostics import LaneDiagnostics
 class Nonreflexive(LaneCenteringSettings):
  def __ne__(self,other):return True
 c,q,r=setup();q=replace(q,settings=Nonreflexive());events=[];d=LaneDiagnostics()
 send(d,c,q,r,events);send(d,c,q,r,events)
 assert sum(name=='lane_centering_settings' for name,_ in events)==2


def test_controller_construction_prewarms_all_v6_grids():
 from openpilot.starpilot.lateral import lane_v6
 from openpilot.starpilot.lateral.lane_path_v6 import _free_fit,_unit_shape,_interval_support
 lane_v6._warm_matrices.cache_clear()
 LaneCenteringV6Controller(clock=lambda:0.)
 before=[f.cache_info().misses for f in (_free_fit,_unit_shape,_interval_support)]
 for count in range(21,82):
  horizon=(count-1)/20
  for local in (False,True):
   for near in (False,True):_free_fit(horizon,count,near,local);_unit_shape(horizon,count,near,local)
   _interval_support(horizon,count,local)
 assert before==[f.cache_info().misses for f in (_free_fit,_unit_shape,_interval_support)]
