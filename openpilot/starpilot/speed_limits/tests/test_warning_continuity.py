"""SP-011 warning identity is independent of map control-generation fences."""
from dataclasses import replace
import pytest
from openpilot.starpilot.speed_limits.mapped_timing import MappedTiming
from openpilot.starpilot.speed_limits.tests.test_mapped_timing import BASE,OBS,NOW
from openpilot.starpilot.speed_limits.runtime_settings import parse
from openpilot.starpilot.speed_limits import acceptance as a
from openpilot.starpilot.common.slc_warning import BoundaryWarningIdentity


def tick(h,t,d,g=5,way=50,current=8.9408,upcoming=13.4112):
  now=NOW+int(t*1e9)
  settings=parse({'SpeedLimitController':True,'SLCPriority1':'Maps','SLCHigherLimitDistance':80,'SLCWarningLeadTime':2})
  frame=replace(BASE,event_mono_ns=now,computed_mono_ns=now,gps_mono_ns=now,generation=g,
                speed_mps=current,next_speed_mps=upcoming,next_distance_m=d,next_way_id=way)
  obs=replace(OBS,candidate=replace(OBS.candidate,speed_mps=current))
  result=h.project(obs,frame,now_ns=now,now_boot_ns=now,speed=11,acceleration=0,cluster_delta=0,settings=settings,enabled=True)
  h.warn(now,eligible=True,target_change=True,speed=11,settings=settings)
  return result


def test_recorded_generation_reset_warns_once_but_retains_control_fence():
  h=MappedTiming();tick(h,0,120);tick(h,1,95)
  first=h.warning_event;first_control=h.identity
  assert first is not None
  # Source loss does not itself produce a warning or new map authority.
  h.project(a.Observation(a.ObservationKind.UNKNOWN),None,now_ns=NOW+2_000_000_000,
            now_boot_ns=NOW+2_000_000_000,speed=11,acceleration=0,cluster_delta=0,settings=parse({}),enabled=True)
  assert not h.approach.active
  tick(h,2.7,77,g=7)
  assert h.identity!=first_control
  assert h.warning_identity==first['id']
  assert h.warning_event is None
  assert h.warning_identity in h.warning.announced


@pytest.mark.parametrize('change',[{'way':51},{'d':600},{'t':5},{'current':10}])
def test_new_boundary_distance_or_long_gap_does_not_reuse_warning(change):
  h=MappedTiming();tick(h,0,120);tick(h,1,95);old=h.warning_identity
  args=dict(t=2,d=85,g=7);args.update(change);tick(h,**args)
  assert h.warning_identity!=old


def test_warning_continuity_cannot_reuse_unknown_boundary_ids():
  h=MappedTiming();tick(h,0,120,way=0);tick(h,1,95,way=0);old=h.warning_identity
  tick(h,2,77,g=7,way=0);assert h.warning_identity!=old


def test_warning_continuity_stays_stable_on_long_continuous_approach():
  h=BoundaryWarningIdentity();key=(1,'external',1,2,20,30)
  assert h.update('first',key=key,now=0,distance=100,speed=0)=='first'
  for t in range(1,126):assert h.update(str(t),key=key,now=t,distance=100,speed=0)=='first'
  assert h.update('new',key=key,now=128,distance=100,speed=0)=='new'
