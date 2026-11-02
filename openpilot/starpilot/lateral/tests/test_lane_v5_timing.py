from dataclasses import replace
from unittest.mock import patch
import numpy as np
import pytest
from openpilot.starpilot.lateral import lane_centering, lane_path
from openpilot.starpilot.lateral.tests.test_lane_v5 import controller, obs


def test_settings_refresh_revalidates_and_invalid_edit_rejects():
 c=controller();first=obs()
 with patch.object(lane_centering, 'valid_tuning', wraps=lane_centering.valid_tuning) as check:
  assert c.update(first).reason=='qualified'
  for i in range(1,5): c.update(replace(obs(i), settings=first.settings))
  assert check.call_count==1
  updated=replace(first.settings, smoothing_tau=.25)
  c.update(replace(obs(6),settings=updated))
  assert check.call_count==2
  bad=replace(updated, min_confidence=.9, confidence_split=.2)
  assert c.update(replace(obs(7),settings=bad)).reason=='invalid_input'
  assert check.call_count==3


def test_frame_cache_never_carries_old_boundary_geometry():
 c=controller();first=obs();c.update(first)
 other=obs(5)
 other.model.laneLines[1].y=[float('nan')]*len(other.model.laneLines[1].y)
 other.model.laneLines[2].y=[float('nan')]*len(other.model.laneLines[2].y)
 other.model.roadEdges[0].y=[float('nan')]*len(other.model.roadEdges[0].y)
 other.model.roadEdges[1].y=[float('nan')]*len(other.model.roadEdges[1].y)
 assert c.update(other).reason!='qualified' and c.target_path is None


@pytest.mark.parametrize('count',[4,5,9,17,33])
def test_cubic_fit_matches_scaled_polynomial_fit_and_invalidates_grid(count):
 rng=np.random.default_rng(16)
 fit=lane_path.CubicFit()
 for _ in range(20):
  x=np.sort(rng.uniform(-1,1,count));y=rng.normal(size=count)
  for yy in (y,-y,y+.2):
   np.testing.assert_allclose(fit(x,yy,3),np.polynomial.polynomial.polyfit(x,yy,3),rtol=1e-9,atol=1e-8)
 with pytest.raises(ValueError):fit(x,y,2)


def test_feasible_free_fit_skips_active_set_but_bounds_still_use_it():
 from openpilot.starpilot.lateral.tests.test_lane_path import args
 with patch.object(lane_path,'_fit',wraps=lane_path._fit) as fit:
  target=lane_path.build_path(**args(.02))
  assert target.fit_iterations==0 and fit.call_count==0
  a=args(.5);a['upper'][:]=.01
  target=lane_path.build_path(**a)
  assert fit.call_count==1 and np.max(target.target_y)<=.01+1e-7
