"""Exercise diagnostics inside the real host curvature/limiter composition."""
import inspect,json,os,tempfile
from pathlib import Path
from openpilot.starpilot.lateral.tests import test_lane_host as host_test
from openpilot.starpilot.lateral.lane_diagnostics import LaneDiagnostics
from openpilot.common.swaglog import cloudlog


def test_real_host_with_diagnostics_enabled(tmp_path,monkeypatch):
 events=[]
 monkeypatch.setattr(cloudlog,'event',lambda name,**kw:events.append((name,kw)))
 namespace=dict(vars(host_test),LaneDiagnostics=LaneDiagnostics)
 source=inspect.getsource(host_test.probe).replace('control = Controls.__new__(Controls)','control = Controls.__new__(Controls)\n    control.lane_diagnostics = LaneDiagnostics()')
 exec(source,namespace)
 with tempfile.TemporaryDirectory(prefix='msgq_lane-host-',dir='/dev/shm') as ipc:
  monkeypatch.setenv('OPENPILOT_PREFIX',Path(ipc).name.removeprefix('msgq_'))
  for key in ('CEREAL_FAKE','CEREAL_FAKE_PREFIX','SIMULATION'):monkeypatch.delenv(key,raising=False)
  out=tmp_path/'host.json';namespace['probe'](out)
 records=json.loads(out.read_text())['records'];assert len(records)==1960
 for row in records:
  command,reference=row['host'],row['reference']
  assert command['actuators']['accel']==reference['actuators']['accel']
  assert command['latActive']==reference['latActive']
 data=[v for n,v in events if n=='lane_centering_diagnostic']
 assert data and any(v['reason']=='qualified' for v in data)
 assert all(set(('base','pre_rate','post_rate','final','measured','limited','applied'))<=set(v['host']) for v in data)
 assert any(n=='lane_centering_settings' for n,v in events)
 json.dumps(data,allow_nan=False)


def test_v5_real_host_resets_limits_and_original_message_preservation(tmp_path,monkeypatch):
 from openpilot.starpilot.lateral.lane_motion import Motion
 bad_motion=[]; original_valid=Motion.valid
 def checked(m):
  valid=original_valid(m)
  if not valid and len(bad_motion)<3: bad_motion.append(repr(m))
  return valid
 monkeypatch.setattr(Motion,"valid",checked)
 namespace=dict(vars(host_test))
 source=inspect.getsource(host_test.probe)
 source=source.replace('control.lane_centering_controller = LaneCenteringVersions()',
   'control.lane_centering_controller = LaneCenteringVersions()\n    control.lane_centering_controller.v5.clock = lambda: 0.')
 source=source.replace('  request = LaneCenteringRequest(',
   '  from openpilot.starpilot.lateral.lane_v5 import LaneCenteringV5Settings as LaneCenteringSettings\n  request = LaneCenteringRequest(')
 source=source.replace('LaneCenteringSettings(True, 0.0, 0.0, True), DT_CTRL)',
   'LaneCenteringSettings(True, 0.0, 0.0, True), DT_CTRL, version=5)')
 source=source.replace('      message("lateralDelay").lateralDelay = 0.1',
   '      if tick % 25 == 0: message("lateralDelay").lateralDelay = 0.1')
 source=source.replace('        event.modelV2 = model()',
   '        event.modelV2 = model()\n        event.modelV2.position.t = [x / 20. for x in event.modelV2.position.x]\n        event.modelV2.position.z = [0.] * len(event.modelV2.position.x)')
 exec(source,namespace)
 with tempfile.TemporaryDirectory(prefix='msgq_lane-host-',dir='/dev/shm') as ipc:
  monkeypatch.setenv('OPENPILOT_PREFIX',Path(ipc).name.removeprefix('msgq_'))
  monkeypatch.setenv('REPLAY','1')
  for key in ('CEREAL_FAKE','CEREAL_FAKE_PREFIX','SIMULATION'):monkeypatch.delenv(key,raising=False)
  out=tmp_path/'v5-host.json';namespace['probe'](out)
 records=json.loads(out.read_text())['records'];assert len(records)==1960
 qualified=[row for row in records if row['result'] and row['result']['reason']=='qualified']
 assert len(qualified)>500 and any(row['result']['correction']>0 for row in qualified), str(sorted({row['result']['reason'] for row in records if row['result']}))+'; '+str(bad_motion)
 for row in records:
  assert row['host']['actuators']['accel']==row['reference']['actuators']['accel']
  result=row['result']
  if result and row['phase'] in ('off','long_only','fault','disabled_host','override','invalid_model','clock_reset','invalid_request'):
   assert result['correction']==0
