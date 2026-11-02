"""Bounded local stop-release evidence; never supplies controller authority."""
import math


def finite(value):
  return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


class StopReleaseDiagnostics:
  def __init__(self):
    self.drive = 0
    self.last_ns = None

  def sample(self, host, sm, *, now_ns, drive_id, native_target, native_stop, held):
    try:
      if type(now_ns) is not int or type(drive_id) is not int or not 0 < drive_id <= now_ns:
        return None
      if drive_id != self.drive:
        self.drive, self.last_ns = drive_id, None
      if self.last_ns is not None and 0 <= now_ns-self.last_ns < 1_000_000_000:
        return None
      car, control = sm['carState'], sm['carControl']
      if host is None or not car.standstill or not control.longActive:
        return None
      projector = host.mode.projector
      detector, launch = projector.stop_detector, projector.tesla_launch
      model = sm['modelV2']
      leads = []
      for lead in (sm['radarState'].leadOne, sm['radarState'].leadTwo):
        leads.append(dict(present=bool(lead.present), distance=finite(lead.dRel),
          speed=finite(lead.vLead), relative_speed=finite(lead.vRel), acceleration=finite(lead.aLeadK),
          probability=finite(lead.modelProb), lateral=finite(lead.yRel)))
      tail = list(model.velocity.x)[15:]
      self.last_ns = now_ns
      return dict(revision=1, now_ns=now_ns, drive_id=drive_id,
        model_ns=int(sm.logMonoTime['modelV2']), car_ns=int(sm.logMonoTime['carState']),
        radar_ns=int(sm.logMonoTime['radarState']),
        v_ego=finite(car.vEgo), v_raw=finite(car.vEgoRaw), gear=str(car.gearShifter),
        gas=bool(car.gasPressed), brake=bool(car.brakePressed),
        native_target=finite(native_target), native_stop=bool(native_stop), held=bool(held),
        committed=detector.committed, standstill_committed=detector.standstill_committed,
        distance=finite(detector.commit_distance_m), clear_since=detector.commit_clear_since_s,
        released_at_standstill=detector.released_at_standstill,
        model_stop=bool(model.action.shouldStop), model_accel=finite(model.action.desiredAcceleration),
        tail_min=finite(min(tail)) if tail else None, tail_end=finite(tail[-1]) if tail else None,
        tail_worst_drop=finite(min(b-a for a,b in zip(tail,tail[1:]))) if len(tail)>1 else None,
        lead_from_stop=launch.lead_from_stop, continuous_since=launch.continuous_since,
        minimum_gaps=launch.minimum_gaps, leads=leads,
        qualification=dict(projector.launch_diagnostic))
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      return None
