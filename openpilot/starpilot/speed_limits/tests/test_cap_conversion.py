"""Fixed-value conversion preserves the existing application and bypass policy."""
from types import SimpleNamespace as NS
import pytest
from openpilot.starpilot.speed_limits.tests.reference_percentage import Adjustment as Percent
from openpilot.starpilot.speed_limits.tests.reference_percentage_planner import LongitudinalPlanner as PercentagePlanner
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, get_max_accel
from openpilot.starpilot.speed_limits.transition_comfort import Adjustment as Fixed
from openpilot.starpilot.longitudinal.tests.test_cruise_ceiling import V_EGO,messages
from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
from openpilot.starpilot.controllers.coast import CoastPlan
from openpilot.starpilot.longitudinal.force_stop import StopPlan
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR


@pytest.mark.parametrize('direction',[-1,1])
@pytest.mark.parametrize('cap_above_profile',[False,True])
@pytest.mark.parametrize('case',['ordinary','lead','throttle','curve','stop','force','traffic','coast_floor','coast_target','stop_target'])
def test_same_response_as_equivalent_percentage_at_same_profile_value(direction,cap_above_profile,case):
  cp=CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)
  old=PercentagePlanner(cp,init_v=V_EGO);new=LongitudinalPlanner(cp,init_v=V_EGO)
  base=get_max_accel(V_EGO) if direction>0 else 1.2
  limit=3.5 if cap_above_profile else (.7 if direction>0 else .8)
  percent=100.*(min(base,limit)/base-1.)
  target=30. if direction>0 else 50/3.6
  class Checked(dict):
    def all_checks(self,services):return True
  for i in range(50):
    now=1_000_000_000+i*50_000_000
    sm=Checked(messages(lead=case=='lead',force=case=='force')[0]);sm.logMonoTime={'modelV2':now}
    sm['carControl'].longActive=True
    sm['carState'].vCruise=target*3.6;sm['carState'].vEgoCluster=V_EGO
    sm['modelV2'].action.shouldStop=case=='stop'
    sm['modelV2'].meta.disengagePredictions.gasPressProbs=[1.,.1 if case=='throttle' else 1.]
    kwargs=dict(now_ns=now,global_braking_response='standard',traffic_mode=case=='traffic')
    if case=='curve':kwargs['curve_ceiling']=CurveCeiling(25. if direction>0 else 10.,now)
    if case in ('coast_floor','coast_target'):
      plan=CoastPlan(brake_floor=-1.4) if case=='coast_floor' else CoastPlan(speed_ceiling_mps=10.)
      kwargs['wheel_coast']=(NS(sample=lambda *a,**kw:plan),None)
    if case=='stop_target':kwargs['force_stop_provider']=lambda _:StopPlan(model_ns=now,speed_ceiling_mps=10.)
    old.update(sm,slc_comfort=Percent(now,target,direction,percent),**kwargs)
    new.update(sm,slc_comfort=Fixed(now,target,direction,limit),**kwargs)
    assert new.a_cruise==pytest.approx(old.a_cruise,abs=1e-8)
    assert new.output_a_target==pytest.approx(old.output_a_target,abs=1e-8)
    assert new.output_should_stop==old.output_should_stop
