"""Control-rate scalar arbitration. No file, map, process or database work."""
from dataclasses import dataclass
import math

DEAD_BAND=.3


def corner_positions(corner):
  """Each stored/observed third takes effect at the start of that third."""
  span=corner.exit-corner.entry
  return tuple(corner.entry+i*span/3 for i in range(3))


def approach_speed(point_speed,distance,speed,acceleration,uncertainty):
  # route_envelope inlines this formula to share reserve/common-point work.
  # Keep both bitwise-equivalent; the public-formula regression checks them.
  # Reserve distance to ramp from current acceleration to comfort braking.
  ramp=max(0,min(6,(max(-.6,acceleration)+.6)/.3))
  reserve=speed*ramp+max(0,acceleration)*ramp*ramp/2
  usable=max(0,distance-uncertainty-reserve)
  return math.sqrt(max(0,point_speed*point_speed+2*.6*usable))


def route_envelope(common,options,*,cruise,speed,acceleration,uncertainty,travelled=0,
                   skip_key='',allow_faster=False,admission=False):
  """MIN within a path, MAX across paths, retaining permissive empty paths."""
  common=tuple(sorted(common,key=lambda p:p.distance))
  if len(common)>24:
    common=common[:24]
    options=((),)
  if len(options)>32 or any(len(path)>128 for path in options): options=((),)
  common=tuple(p for p in common if p.key!=skip_key or not skip_key)
  common_keys={p.key for p in common if p.key}
  ramp=max(0,min(6,(max(-.6,acceleration)+.6)/.3))
  reserve=speed*ramp+max(0,acceleration)*ramp*ramp/2
  def allowances(points):
    values=[];priors=[]
    for point in points:
      limit=min(point.speed,point.ceiling)
      if not allow_faster: limit=min(limit,point.prior)
      if admission and point.admission_speed>0:
        limit=min(limit,point.admission_speed)
      # Inlined approach_speed: keep subtraction order and the formula above.
      usable=max(0,(point.distance-travelled)-uncertainty-reserve)
      prior=math.sqrt(max(0,point.prior*point.prior+2*.6*usable))
      value=prior if limit==point.prior else math.sqrt(max(0,limit*limit+2*.6*usable))
      values.append(value);priors.append(prior)
    return values,priors
  common_values,common_priors=allowances(common)
  paths=[]
  for option in options or ((),):
    option=tuple(p for p in option if p.key!=skip_key or not skip_key)
    points=common+option
    values,priors=allowances(option)
    values=common_values+values;priors=common_priors+priors
    value=min((cruise,*values));prior=min((cruise,*priors));key=''
    if values and value<cruise-DEAD_BAND:
      index=min(range(len(values)),key=values.__getitem__)
      proposed=points[index].key
      if proposed in common_keys and not any(abs(v-value)<=DEAD_BAND and points[i].key!=proposed for i,v in enumerate(values)):
        key=proposed
    paths.append((value,prior,key))
  raw=max(p[0] for p in paths);prior_raw=max(p[1] for p in paths)
  bindings={p[2] for p in paths if abs(p[0]-raw)<=DEAD_BAND}
  binding=next(iter(bindings)) if len(bindings)==1 else ''
  return raw,prior_raw,binding,tuple(p[0] for p in paths)


@dataclass(frozen=True)
class Decision:
  cap: float
  owner: str
  suppress_training: bool
  visual_nudge: bool
  skip_key: str
  intervention: bool
  observation_id: str
  status: str
  binding_key: str = ''


class Coordinator:
  def __init__(self):
    self.control_epoch=0
    self.observation_fence=0
    self.odometer=0.0
    self.previous_time=None
    self.previous_auto=False
    self.previous_parent=False
    self.previous_learn=False
    self.previous_owner='none'
    self.cap=0.0
    self.prior_cap=0.0
    self.binding_key=''
    self.key=''
    self.skip_key=''
    self.skip_until=0.0
    self.skip_odometer=0.0
    self.exclusion_until=0.0
    self.exclusion_odometer=0.0
    self.exclusion_key=''
    self.visual_frames=0
    self.faster_veto=False
    self.faster_latched=False
    self.observation_sequence=0

  def invalidate(self):
    self.control_epoch+=1
    self.observation_fence+=1

  def step(self,*,now,speed,acceleration,cruise,visual,visual_valid,other,automated,parent,learn,
           available,button,gas,brake,cancel,confounded,fault,result,visual_override=False):
    if not all(math.isfinite(v) for v in (now,speed,acceleration,cruise,visual,other)):
      self.invalidate()
      self.cap=0
      return Decision(0,'other',True,False,'',False,'','Map unavailable')
    dt=.05 if self.previous_time is None else max(0,min(.1,now-self.previous_time))
    gap=self.previous_time is not None and not 0<now-self.previous_time<=.25
    self.previous_time=now
    self.odometer+=max(0,speed)*dt
    was_owner=self.previous_owner
    was_auto=self.previous_auto
    parent_changed=parent!=self.previous_parent
    previous_epoch=self.control_epoch
    if automated!=was_auto or parent_changed:
      self.control_epoch+=1
    if (automated and not was_auto) or parent_changed or (self.previous_learn and not learn) or gap or fault:
      self.observation_fence+=1
    self.previous_auto=automated
    self.previous_parent=parent
    self.previous_learn=learn
    fresh=(result is not None and result.control_epoch==self.control_epoch and
           0<=now-result.now<=.5 and 0<=self.odometer-result.odometer<=30 and not gap)
    identity_fresh=(result is not None and result.control_epoch in (previous_epoch,self.control_epoch) and
                    0<=now-result.now<=.5 and 0<=self.odometer-result.odometer<=30 and not gap)
    if fresh and result.key and result.key!=self.key:
      self.key=result.key
      self.visual_frames=0
      self.faster_veto=False
      self.faster_latched=False
    if self.skip_key and (now>=self.skip_until or self.odometer-self.skip_odometer>=600 or
                         (self.skip_key!='@routes' and fresh and result.key and result.key!=self.skip_key and
                          all((p.key or result.key)!=self.skip_key for p in result.points)) or
                         (fresh and result.key==self.skip_key and result.progress>result.exit+10)):
      self.skip_key=''
    # Driver button bypasses this map bend only. Visual bypass is handled by
    # VCruise's existing explicit-override path, not by learned preferences.
    skipped_now=bool(parent and identity_fresh and result.key and ((gas and was_auto) or (button and (was_owner in ('map','visual','tie') or visual_override))))
    if skipped_now:
      aggregate=bool(getattr(result,'route_options',())) and not self.binding_key
      self.skip_key='@routes' if aggregate else self.binding_key if was_owner=='map' and self.binding_key else result.key
      self.skip_until=now+45
      self.skip_odometer=self.odometer
      self.control_epoch+=1
      self.cap=0
    if self.faster_latched and not visual_valid:
      self.faster_veto=True
    self.visual_frames=self.visual_frames+1 if visual_valid else 0
    if self.visual_frames>=10 and not self.faster_veto:
      self.faster_latched=True
    raw=cruise
    prior_raw=cruise
    raw_binding=''
    eligible=parent and available and automated and fresh and not skipped_now
    points=tuple(result.points) if eligible and self.skip_key!='@routes' else ()
    options=getattr(result,'route_options',()) if eligible and self.skip_key!='@routes' else ()
    if points or options:
      travelled=self.odometer-result.odometer
      # Legacy synthetic callers omit point keys; production always supplies them.
      if not options:
        from dataclasses import replace
        points=tuple(replace(p,key=p.key or result.key) for p in points)
      raw,prior_raw,raw_binding,_=route_envelope(points,options,cruise=cruise,speed=speed,
        acceleration=acceleration,uncertainty=result.uncertainty,travelled=travelled,skip_key=self.skip_key,
        allow_faster=visual_valid and self.faster_latched and not self.faster_veto)
    if not automated or skipped_now or (parent and not available):
      self.cap=0.0
      self.prior_cap=0.0
      self.binding_key=''
    elif eligible and raw<cruise-.01:
      if self.cap<=0 or raw<=self.cap: self.binding_key=raw_binding
      if self.cap<=0: self.cap=min(cruise,max(raw,speed))
      self.cap=max(raw,self.cap-1*dt) if raw<self.cap else min(raw,self.cap+.5*dt)
      if self.cap==raw: self.binding_key=raw_binding
    elif self.cap>0:
      self.cap=min(cruise,self.cap+.5*dt)
      if self.cap>=cruise-.01: self.cap=0
    if automated and not skipped_now and not (parent and not available):
      if eligible and prior_raw<cruise-.01:
        if self.prior_cap<=0: self.prior_cap=min(cruise,max(prior_raw,speed))
        self.prior_cap=max(prior_raw,self.prior_cap-dt) if prior_raw<self.prior_cap else min(prior_raw,self.prior_cap+.5*dt)
      elif self.prior_cap>0:
        self.prior_cap=min(cruise,self.prior_cap+.5*dt)
        if self.prior_cap>=cruise-.01: self.prior_cap=0
      if eligible and self.prior_cap>0 and not (visual_valid and self.faster_latched and not self.faster_veto):
        if self.cap>self.prior_cap: self.binding_key=''
        self.cap=min(self.cap,self.prior_cap) if self.cap>0 else self.prior_cap
    map_cap=self.cap if self.cap>0 else math.inf
    visual_cap=visual if visual>0 else math.inf
    owner='manual' if not automated else 'other'
    if automated and not confounded and not fault:
      if map_cap<min(visual_cap,other,cruise)-DEAD_BAND and speed>=map_cap-.5: owner='map'
      elif visual_cap<min(map_cap,other,cruise)-DEAD_BAND and speed>=visual_cap-.5: owner='visual'
      elif min(map_cap,visual_cap)<min(other,cruise)-DEAD_BAND: owner='tie'
    map_influenced=automated and map_cap<=min(visual_cap,other,cruise)+DEAD_BAND and map_cap<cruise-DEAD_BAND
    if map_influenced and (now>=self.exclusion_until or self.exclusion_key!=self.key):
      self.exclusion_until=now+45
      self.exclusion_odometer=self.odometer
      self.exclusion_key=self.key
    if fresh and result.key==self.exclusion_key and result.progress>result.exit+10:
      self.exclusion_until=min(self.exclusion_until,now+5)
    exclusion=(now<self.exclusion_until and self.odometer-self.exclusion_odometer<600) or map_influenced
    intervention=bool(was_auto and not automated and (gas or brake or cancel) and not fault)
    if gas and was_auto and not fault: intervention=True
    if intervention: self.observation_sequence+=1
    token=f'{self.observation_fence}:{self.observation_sequence}' if intervention else ''
    visual_nudge=was_owner=='visual' and not exclusion
    self.previous_owner=owner
    if self.cap>0 and owner=='map': status='Remembered corner' if fresh and result.remembered else 'Map corner'
    elif self.skip_key: status='Corner skipped'
    elif owner=='visual': status='Visual limit'
    elif not parent: status='Map corners off'
    else: status=result.status if fresh else 'Map unavailable'
    return Decision(self.cap,owner,exclusion,visual_nudge,self.skip_key,intervention,token,status,self.binding_key)
