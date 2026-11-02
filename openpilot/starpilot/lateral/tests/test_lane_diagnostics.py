from dataclasses import replace
import json
import pytest
from openpilot.starpilot.lateral.lane_centering import LaneCenteringController, LaneCenteringRequest, ControlMode
from openpilot.starpilot.lateral.lane_diagnostics import LaneDiagnostics
from openpilot.starpilot.lateral.tests.test_lane_v4 import scene,tick
from openpilot.starpilot.lateral.tests.test_lane_centering import observation


def setup():
 c=LaneCenteringController();o=observation(scene(),model_sample_id=100)
 r=c.update(o);q=LaneCenteringRequest(ControlMode.LATERAL_ONLY,o.settings,.01)
 return c,q,r


def send(d,c,q,r,events,model=100,now=1_000_000_000):
 d.emit(lambda name,**fields:events.append((name,fields)),c,q,r,model_ns=model,now_ns=now,base=.001,final=.002,applied=.0001)


def test_settings_changes_periodic_snapshot_and_frame_cadence():
 d=LaneDiagnostics();c,q,r=setup();events=[]
 send(d,c,q,r,events);send(d,c,q,r,events)
 assert [n for n,_ in events]==['lane_centering_settings','lane_centering_diagnostic']
 send(d,c,q,r,events,101,1_010_000_000)
 assert len(events)==2
 q=replace(q,settings=replace(q.settings,response_min=.2))
 send(d,c,q,r,events,101,1_050_000_000)
 assert events[-2][0]=='lane_centering_settings' and events[-1][1]['revision']==events[-2][1]['revision']
 assert events[-2][1]['revision']!=events[0][1]['revision']
 send(d,c,q,r,events,102,62_000_000_000)
 assert events[-2][0]=='lane_centering_settings'
 assert events[-1][1]['geometry']=={}  # Never report an older model frame as fresh geometry.
 json.dumps(events,allow_nan=False)


def test_logging_failure_does_not_mutate_controller():
 d=LaneDiagnostics();c,q,r=setup();before=(c._correction,c._raw,c._sample_id,list(c._smoothed))
 def fail(*a,**k):raise RuntimeError('sink unavailable')
 d.emit(fail,c,q,r,model_ns=100,now_ns=1000)
 assert d.failures==1
 assert before==(c._correction,c._raw,c._sample_id,list(c._smoothed))
 events=[];send(d,c,q,r,events);assert events[-1][1]['failures']==1


def test_geometry_targets_and_single_boundary_no_attraction():
 c=LaneCenteringController();tick(c,scene(left=-1.5,right=2.1),frame=10)
 assert c.diagnostic['raw_target_m']==pytest.approx(.3)
 assert c.diagnostic['smoothed_target_m']==pytest.approx(.3)
 tick(c,scene(left=-.9,rp=0),frame=11)
 assert c.selected_mode==3 and c.diagnostic['raw_shortfall_m']>0
 tick(c,scene(left=-3,rp=0),frame=12)
 assert c.diagnostic['geometry']=='single_clear'
 c.reset();assert c.diagnostic=={}


def test_bend_reasons_and_normal_zero_bend():
 c=LaneCenteringController();tick(c,scene(),curve_adaptation=True)
 assert c.diagnostic['bend_reason']=='valid'
 assert c.diagnostic['bend_target']==0
 tick(c,scene(rp=0),frame=2,curve_adaptation=True)
 assert c.diagnostic['bend_reason']=='single_boundary_hold'
 assert 'bend_target' not in c.diagnostic
 tick(c,scene(),frame=3,curve_adaptation=False)
 assert c.diagnostic['bend_reason']=='disabled'


def test_nonfinite_diagnostic_becomes_null_and_clock_reset_resnapshots():
 d=LaneDiagnostics();c,q,r=setup();events=[]
 c.diagnostic={'frame':100,'bad':float('nan')};send(d,c,q,r,events)
 assert events[-1][1]['geometry']['bad'] is None
 send(d,c,q,r,events,100,500_000_000)
 assert [n for n,_ in events][-2:]==['lane_centering_settings','lane_centering_diagnostic']
 json.dumps(events,allow_nan=False)
