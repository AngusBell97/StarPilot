import unittest, json, time, hashlib
import numpy as np
from openpilot.starpilot.lateral.lane_path import build_path
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan as converter

def args(desired=.2):
 t=np.linspace(0,3,121)
 return dict(frame=1,t=t,x=15*t,model_y=np.zeros(len(t)),desired_offset=np.full(len(t),desired) if np.isscalar(desired) else desired,lower=np.full(len(t),-.7),upper=np.full(len(t),.7),speed=15.,action_delay=.31,base_curvature=.002,cap=.0018,converter=converter)

from openpilot.starpilot.lateral.lane_path import _matrices, _fit
class Tests(unittest.TestCase):
 def test_zero_preserves_action_exactly(self):
  for base in (-.02,0.,.013):
   a=args(0.);a['base_curvature']=base;r=build_path(**a)
   self.assertEqual(r.candidate_curvature,base);self.assertEqual(r.correction,0.)
   np.testing.assert_array_equal(r.target_y,r.model_y)
 def test_beginning_preserves_pose_heading_curvature(self):
  r=build_path(**args());self.assertEqual(r.target_y[0],r.model_y[0])
  self.assertEqual(r.heading_delta[0],0.);self.assertEqual(r.curvature_delta[0],0.)
 def test_same_target_drives_display_and_converter(self):
  a=args();r=build_path(**a)
  self.assertAlmostEqual(converter(r.heading_delta,r.yaw_rate_delta,r.t,a['speed'],a['action_delay']),r.correction)
  np.testing.assert_array_equal(r.display(True)[1],r.target_y)
  np.testing.assert_array_equal(r.display(False)[1],r.model_y)
  self.assertFalse(r.target_y.flags.writeable)
 def test_opposing_preview_in_one_path(self):
  t=args()['t'];r=build_path(**args(np.interp(t,[0,1,2,3],[.2,.2,-.2,-.2])))
  self.assertGreater(np.interp(1,t,r.target_y),.02)
  self.assertLess(np.interp(2.5,t,r.target_y),-.02)
 def test_far_shape_changes_near_request(self):
  t=args()['t'];a=build_path(**args(.02));b=build_path(**args(np.interp(t,[0,1,2,3],[.02,.02,-.02,-.02])))
  self.assertNotAlmostEqual(a.correction,b.correction,places=9)
 def test_mirror(self):
  a=build_path(**args(.2));b=build_path(**args(-.2))
  np.testing.assert_allclose(a.target_y,-b.target_y,atol=1e-10)
  self.assertAlmostEqual(a.correction,-b.correction)
 def test_cap_scales_target_too(self):
  a=args(.5);a['cap']=.0001;r=build_path(**a)
  self.assertLess(r.cap_scale,1.);self.assertLessEqual(abs(r.correction),a['cap']+1e-10)
  self.assertLessEqual(np.max(np.abs(r.curvature_delta)),a['cap']+1e-10)
 def test_direction_and_bounds(self):
  for sign in (-1,1):
   a=args(.5*sign);a['away']=sign;r=build_path(**a)
   self.assertGreaterEqual(np.min(sign*r.target_y),-1e-8)
   self.assertGreaterEqual(sign*r.correction,-1e-10)
 def test_bounds_constrain_fit(self):
  a=args(.5);a['upper'][:]=.04;r=build_path(**a)
  self.assertLessEqual(np.max(r.target_y),.04+1e-7)
 def test_fully_suppressed_region(self):
  a=args(.2);a['lower'][:45]=0.;a['upper'][:45]=0.;r=build_path(**a)
  self.assertLess(np.max(np.abs(r.target_y[:45])),1e-7)
 def test_no_mutation(self):
  a=args();original={k:v.copy() for k,v in a.items() if isinstance(v,np.ndarray)};build_path(**a)
  for k,v in original.items():np.testing.assert_array_equal(v,a[k])
 def test_invalid_inputs(self):
  for key,value in [('speed',float('nan')),('speed',.5),('action_delay',0.),('cap',0.),('cap',float('inf')),('away',2)]:
   a=args();a[key]=value
   with self.subTest(key=key,value=value),self.assertRaises(ValueError):build_path(**a)
  for key in ('x','t','model_y','desired_offset','lower','upper'):
   a=args();a[key][5]=float('nan')
   with self.subTest(key=key),self.assertRaises(ValueError):build_path(**a)
 def test_no_extrapolation(self):
  for key in ('x','t'):
   a=args();a[key][10]=a[key][9]
   with self.subTest(key=key),self.assertRaises(ValueError):build_path(**a)
 def test_curved_original_and_action_residual(self):
  a=args(0.);a['model_y']=.001*a['x']**2;r=build_path(**a)
  self.assertEqual(r.correction,0.);self.assertEqual(r.candidate_curvature,a['base_curvature'])
 def test_boundary_basis_continuity(self):
  from openpilot.starpilot.lateral.lane_path import _basis
  h=3.;knots=np.r_[np.zeros(4),np.linspace(0,h,11)[1:-1],np.full(4,h)]
  for k in knots[4:-4]:
   for order in range(3):
    b=_basis(np.array([k-1e-9,k+1e-9]),knots,3,order)
    np.testing.assert_allclose(b[0],b[1],atol=1e-6)
if __name__=='__main__': unittest.main(verbosity=2)
