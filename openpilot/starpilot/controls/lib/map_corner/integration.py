"""Translate existing planner state into bounded local feature messages."""
import time

from .controller import Coordinator
from .runtime import Runtime
from .types import Fix, Frame
from .diagnostics import ParentDiagnostics


def stream_valid(sm,name):
  try: return bool(sm.valid[name] and sm.alive[name])
  except (KeyError,AttributeError,TypeError): return False


class MapFeature:
  def __init__(self):
    self.coordinator=Coordinator()
    self.runtime=None
    self.runtime_factory=Runtime
    self.clock=time.monotonic
    self.latest_result=None
    self.fix=None
    self.fix_id=None
    self.sequence=0
    self.status='Map corners off'
    self.diagnostics=ParentDiagnostics()

  def close(self):
    if self.runtime: self.runtime.close()
    self.runtime=None
    self.latest_result=None

  def update(self,vc,sm,toggles,speed,cruise,other,available,accel_button):
    now=self.clock()
    planner=vc.starpilot_planner
    parent=bool(getattr(toggles,'map_corner_speed',False) and toggles.curve_speed_controller)
    learn=bool(parent and getattr(toggles,'learn_corner_speeds',False))
    car=sm['carState']
    gas=bool(car.gasPressed)
    brake=bool(getattr(car,'brakePressed',False))
    auto=bool(sm['carControl'].longActive and not gas and not brake)
    events=sm['onroadEvents']
    fault=any(getattr(e,'immediateDisable',False) or getattr(e,'softDisable',False) for e in events)
    cancel=any(getattr(e,'userDisable',False) for e in events)
    cancel |= any(str(getattr(e,'type',''))=='cancel' and getattr(e,'pressed',False) for e in getattr(car,'buttonEvents',()))
    if self.coordinator.previous_auto and not auto and not (gas or brake or cancel): fault=True
    lead=bool(getattr(planner,'tracking_lead',False) or getattr(getattr(planner,'lead_one',None),'status',False))
    stop=bool(vc.forcing_stop or getattr(getattr(planner,'starpilot_cem',None),'stop_light_detected',False) or getattr(planner,'raw_model_stopped',False))
    blink=bool(car.leftBlinker or car.rightBlinker)
    confounded=bool(lead or stop or blink or vc.nav_turn_target>0)
    evidence=()
    visual_observable=bool(parent and not vc.csc_override and not lead and not blink and stream_valid(sm,'modelV2'))
    if visual_observable:
      try: evidence=vc.csc.corner_evidence(sm['modelV2'])
      except (AttributeError,KeyError,TypeError,ValueError): visual_observable=False
    result=self.latest_result
    result_fresh=bool(result and result.control_epoch==self.coordinator.control_epoch and
                      0<=now-result.now<=.5 and 0<=self.coordinator.odometer-result.odometer<=30)
    visual_for_control=bool(available and not vc.csc_override and visual_observable and result_fresh and result.visual_confirmed)
    decision=self.coordinator.step(now=now,speed=speed,acceleration=float(getattr(car,'aEgo',0)),cruise=cruise,
      visual=vc.csc_target,visual_valid=visual_for_control,other=other,automated=auto,parent=parent,learn=learn,
      available=available and not vc.forcing_stop,button=accel_button,gas=gas,brake=brake,cancel=cancel,
      confounded=confounded,fault=fault,result=result,visual_override=bool(vc.csc_override and accel_button))
    self.status=decision.status
    if not parent:
      self.close()
      return decision
    if self.runtime is None: self.runtime=self.runtime_factory()
    self.runtime.fence(self.coordinator.observation_fence,learn)
    service=getattr(planner,'gps_location_service','gpsLocationExternal')
    if stream_valid(sm,service):
      try:
        ident=int(sm.logMonoTime[service])
        if ident!=self.fix_id:
          g=sm[service]
          self.fix=Fix(now,ident,float(g.latitude),float(g.longitude),float(g.bearingDeg),float(g.horizontalAccuracy),float(g.speed),bool(g.hasFix))
          self.fix_id=ident
      except (KeyError,AttributeError,TypeError,ValueError): self.fix=None
    else: self.fix=None
    weather=getattr(planner,'starpilot_weather',None)
    updated=getattr(weather,'last_updated',None)
    clear=False
    if updated is not None and getattr(toggles,'weather_presets',False):
      try: clear=bool(0<=time.time()-updated.timestamp()<=900 and 800<=weather.weather_id<=803 and weather.is_daytime)
      except (AttributeError,ValueError,TypeError): pass
    self.sequence+=1
    frame=Frame(now,self.sequence,self.coordinator.control_epoch,self.coordinator.observation_fence,self.fix,
      speed,float(getattr(car,'aEgo',0)),self.coordinator.odometer,auto,learn,decision.owner,decision.intervention,
      decision.observation_id,confounded,fault,visual_observable,clear,evidence,decision.binding_key,
      decision.cap if auto and available and not fault and result_fresh and result and
      result.control_epoch==self.coordinator.control_epoch and not decision.skip_key else 0.0,
      min(cruise,other),confound_flags=int(lead)|int(stop)<<1|int(blink)<<2|int(vc.nav_turn_target>0)<<3)
    newest,overflow=self.runtime.exchange(frame)
    reason=('queue_full' if overflow else 'worker_failed' if self.runtime.failed else
      'result_missing' if result is None else 'result_epoch' if result.control_epoch!=self.coordinator.control_epoch else
      'result_expired' if not 0<=now-result.now<=.5 else
      'result_travel' if not 0<=self.coordinator.odometer-result.odometer<=30 else 'ready')
    self.runtime.record_diagnostics(self.diagnostics.update(frame,result,reason))
    if overflow or self.runtime.failed:
      self.coordinator.invalidate()
      self.runtime.fence(self.coordinator.observation_fence,False)
      self.latest_result=None
    elif newest is not None and (self.latest_result is None or newest.sequence>self.latest_result.sequence):
      self.latest_result=newest
    return decision
