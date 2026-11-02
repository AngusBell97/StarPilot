"""Small target-path attachment on the existing lateral feedback service."""
import numpy as np

VIEW_KEY = "LaneCenteringShowTargetPath"


def write_target(event, controller, result, *, model_ns, control_ns):
  target = getattr(controller, "target_path", None)
  z = getattr(controller, "target_z", None)
  if (target is None or result is None or result.reason != "qualified" or target.frame != model_ns or
      z is None or len(z) != len(target.x)):
    return
  state = event.starpilotLateralState.lanePath
  state.version = 1
  state.valid = True
  state.modelMonoTime = target.frame
  state.controlMonoTime = control_ns
  # Samples of the same fitted trajectory, not a curve reconstructed from the bar.
  cached = getattr(controller, "_target_wire_cache", None)
  if cached is None or cached[0] is not target or cached[1] is not z:
    ids = list(range(0, len(target.x), 2))
    if ids[-1] != len(target.x)-1: ids.append(len(target.x)-1)
    cached = (target, z, target.x[ids].tolist(), target.target_y[ids].tolist(), np.asarray(z)[ids].tolist())
    controller._target_wire_cache = cached
  state.x, state.y, state.z = cached[2:]
  state.requestedCurvatureCorrection = target.correction
