from types import SimpleNamespace as NS
import numpy as np
from openpilot.cereal import messaging, log
from openpilot.starpilot.lateral.lane_path_wire import write_target
from openpilot.starpilot.lateral.tests.test_lane_v5 import controller, obs


def setup():
 c=controller();o=obs();r=c.update(o);event=messaging.new_message('starpilotLateralState',valid=True)
 event.starpilotLateralState.laneCentering.lateralActive=True
 write_target(event,c,r,model_ns=o.model_sample_id,control_ns=o.motion.timestamp_ns)
 state=event.starpilotLateralState
 class SM(dict): pass
 sm=SM(modelV2=o.model,carControl=NS(latActive=True),starpilotLateralState=state)
 sm.logMonoTime={k:o.motion.timestamp_ns for k in sm};sm.alive={k:True for k in sm};sm.valid=dict(sm.alive);sm.recv_frame={k:5 for k in sm}
 return c,event,sm


def test_exact_same_target_samples_and_wire_roundtrip():
 c,event,sm=setup()
 with log.Event.from_bytes(event.to_bytes()) as decoded:
  p=decoded.starpilotLateralState.lanePath
  np.testing.assert_allclose(p.y,c.target_path.target_y[::2],rtol=1e-6,atol=1e-8)
  assert p.modelMonoTime==c.target_path.frame and p.valid
  assert p.requestedCurvatureCorrection==np.float32(c.target_path.correction)


