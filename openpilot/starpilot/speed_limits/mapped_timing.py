"""SP-011 mapped timing using 7.0 cruise braking and command ownership.

Predictions integrate the selected speed-dependent brake curve and 7.0's cruise
jerk schedule. They are a comfort forecast, not a stopping-distance guarantee.
Only the existing v2 MapTracker can admit new map evidence.
"""
from dataclasses import dataclass, replace
from bisect import bisect_right
import hashlib
import math
import uuid
from openpilot.starpilot.longitudinal.set_speed_comfort import braking_lower_bound, CruiseTransition
from openpilot.starpilot.longitudinal.slc_coast import shaped_slc_floor
from openpilot.selfdrive.car.cruise import slc_target_kph

from openpilot.starpilot.common.slc_transition import MapApproach, LowerLimitDistance
from openpilot.starpilot.common.slc_warning import BoundaryWarningIdentity, SpeedChangeWarning, WarningBoundaryPin
from openpilot.starpilot.speed_limits import acceptance as acc, speed_domain as sd


def interpolate(x, xp, fp):
  index=min(max(bisect_right(xp,x)-1,0),len(xp)-2)
  t=min(max((x-xp[index])/(xp[index+1]-xp[index]),0.),1.)
  return fp[index]+t*(fp[index+1]-fp[index])

@dataclass(frozen=True)
class BrakingProfile:
  speeds: tuple[float,...] = (0.,70.)
  floors: tuple[float,...] = (-.5,-.5)
  limit: float | None = None
  jerk: float | None = None
  style: str = "standard"
  easing: bool = True

  def floor(self, speed):
    # More braking allowance does not guarantee a stronger actual request.
    floor = max(-3.5, min(-.1, interpolate(speed,self.speeds,self.floors)))
    return max(floor, -self.limit) if self.limit is not None else floor


def braking_distance(speed, target, acceleration, profile):
  if speed <= target + .05:return 0.
  if not all(math.isfinite(x) for x in (speed,target,acceleration)) or not 0<=target<speed<=70 or target<1:
    return 0.
  distance=0.;a=min(max(acceleration,-3.5),3.5)
  # One command/actuation allowance; the SLC ceiling bounds the brake curve only.
  # Continue measured acceleration during handover, including existing braking.
  delay=.6
  transition = CruiseTransition() if profile.jerk is not None else None
  for i in range(12000):
    # Same target-error clipping and speed-dependent jerk as get_cruise_accel.
    floor=profile.floor(speed)
    if profile.easing:floor=shaped_slc_floor(speed,target,floor,profile.style,approach=True)
    requested=max(target-speed,floor) if i*.05>=delay else a
    if transition is not None and i*.05 >= delay:
      requested=transition.shape(selected=target,speed=speed,desired=requested,previous=a,
        now_ns=1_000_000_000+i*50_000_000,enabled=True,response_time=.3)
    jerk=interpolate(speed,(0.,10.,25.,40.),(1.6,1.2,.8,.6))
    down_jerk=min(jerk,profile.jerk) if profile.jerk is not None else jerk
    next_a=min(max(requested,braking_lower_bound(a,.05,jerk,down_jerk)),a+jerk*.05)
    mean=next_a
    dt=min(.05,(speed-target-.05)/-mean) if mean<0 else .05
    next_speed=speed+mean*dt
    distance+=(speed+next_speed)*.5*dt
    speed,a=next_speed,next_a
    if speed<=target+.05+1e-8:return max(distance,0.)
  return math.inf


def profile_from_host(host, now_ns, personality, cp, *, traffic=False, enabled=False, previous=None, qualified=True, slc_limit=.8):
  from openpilot.starpilot.longitudinal.set_speed_comfort import DOWN_JERK
  jerk = DOWN_JERK if getattr(cp, "carFingerprint", None) in ("TESLA_MODEL_3", "TESLA_MODEL_Y") else None
  # Runtime freshness is not a different saved braking curve. Unknown inputs
  # prohibit a new lower preview; they must not manufacture a softer profile.
  if (qualified is not True or type(traffic) is not bool or type(slc_limit) not in (int,float) or
      not math.isfinite(slc_limit) or not .1 <= slc_limit <= 3.5):
    return None
  if host is None or not enabled:
    return BrakingProfile(floors=(-1.2,-1.2),jerk=jerk,easing=traffic is False,limit=slc_limit)
  from openpilot.starpilot.longitudinal.profile_document import BRAKING_SPEEDS_MPH, _NATIVE_ACCELERATION_SPEEDS_MS
  # Include every native/custom interpolation breakpoint; a regular grid can miss a trough.
  speeds=tuple(sorted({0.,70.,25.,*_NATIVE_ACCELERATION_SPEEDS_MS,*(v*.44704 for v in BRAKING_SPEEDS_MPH)}))
  floors=[];style="standard"
  for speed in speeds:
    legacy=host.sample(now_ns,personality,speed,cp,traffic_mode=traffic)
    selected=host.sample_selected(now_ns,personality,speed,cp,traffic_mode=traffic,legacy=legacy)
    magnitude=(selected.cruise_brake_magnitude if selected is not None else None)
    if magnitude is None:magnitude=legacy.cruise_brake_magnitude if legacy is not None else None
    if magnitude is None:
      if not getattr(host, "disabled", False):return None
      magnitude=1.2
    if previous is not None:magnitude=min(magnitude,previous.cruise_brake_magnitude)
    floors.append(-float(magnitude))
    style=getattr(selected,"braking_style",getattr(legacy,"slc_braking_style","standard"))
  return BrakingProfile(speeds,tuple(floors),limit=slc_limit,jerk=jerk,style=style if style in ("eco","standard","sport") else "standard",easing=traffic is False)


class MappedTiming:
  def __init__(self):
    self.scope=uuid.uuid4().hex
    self.approach=MapApproach()
    self.distance=LowerLimitDistance()
    self.warning=SpeedChangeWarning()
    self.pin=WarningBoundaryPin()
    self.profile=BrakingProfile()
    self.cache=None
    self.identity=''
    self.warning_identity=''
    self.warning_boundary=BoundaryWarningIdentity()
    self.upcoming=0.
    self.remaining=0.
    self.trigger=0.
    self.eta=math.inf
    self.current=False
    self.warning_event=None
    self.warning_pin={}
    self.source_key=None

  def project(self, observation, map_frame, *, now_ns, now_boot_ns, speed, acceleration, cluster_delta, settings, enabled, target_override=None):
    self.current=False
    self.identity='';self.warning_identity='';self.upcoming=0.;self.eta=math.inf
    if (not enabled or observation.kind is not acc.ObservationKind.VALID or observation.candidate is None or map_frame is None):
      self.approach.update(0,0,0,'',0,0,False)
      return observation
    source=(map_frame.producer_session,map_frame.generation,map_frame.source)
    if self.source_key is not None and source!=self.source_key:
      self.approach=MapApproach();self.distance=LowerLimitDistance();self.cache=None
    self.source_key=source
    self.current=True
    current=observation.candidate.speed_mps
    upcoming,distance=map_frame.next_speed_mps,map_frame.next_distance_m
    if (not all(math.isfinite(v) for v in (upcoming,distance,speed,acceleration,cluster_delta)) or
        not 0<=upcoming<=80 or not -200<=distance<=10000 or not 0<=speed<=70 or
        not isinstance(map_frame.road_name,str) or len(map_frame.road_name)>512):
      self.approach.update(0,0,0,'',0,0,False)
      return observation
    if upcoming<1:distance=0.
    if distance<0:
      # A negative boundary must not create a fresh forecast. Existing software
      # set speed remains governed by its acknowledgement and source history.
      return acc.Observation(acc.ObservationKind.UNKNOWN)
    raw_distance=distance
    now=now_ns/1e9
    lower = 1<=upcoming<current
    forecast_valid = isinstance(self.profile, BrakingProfile)
    if lower:
      distance=self.distance.update(current,upcoming,distance,map_frame.road_name,speed,now,map_frame.event_mono_ns)
      target=sd.adjusted_limit(upcoming,settings.offsets)
      remembered = type(target_override) in (int,float) and math.isfinite(target_override) and 8/3.6 <= target_override <= 145/3.6
      if remembered:
        target=target_override
      #7.0 adopts directly into the card-selected speed. Apply its exact
      # rounding; the obsolete separate-cap ego/cluster subtraction does not apply.
      target=slc_target_kph(target or upcoming, settings.metric)/3.6
      if forecast_valid:
        profile=replace(self.profile,limit=None) if remembered or not settings.custom.transition_offsets else self.profile
        key=(target,profile,settings.custom.lower_timing)
        old=self.cache
        near=old is None or distance<=old[1]+speed*(settings.custom.warning_lead+.5)
        if (old is None or near or now-old[0]>=.2 or key!=old[2] or abs(speed-old[3])>.25 or abs(acceleration-old[4])>.15):
          trigger=braking_distance(speed,target,acceleration,profile)
          trigger=max(0.,trigger+speed*settings.custom.lower_timing)
          self.cache=(now,trigger,key,speed,acceleration)
        trigger=self.cache[1]
        if not math.isfinite(trigger):trigger=distance+speed*(settings.custom.warning_lead+.5)
      else:
        self.cache=None
        trigger=0.  # No fresh forecast; an already active approach may continue.

    else:
      self.distance=LowerLimitDistance()
      trigger=settings.custom.higher_distance if upcoming>current else 0.
    selected=self.approach.update(current,upcoming,distance,map_frame.road_name,speed,trigger,True,
                                  identity_distance=raw_distance,
                                  allow_activate=forecast_valid if lower else 0<=now_boot_ns-map_frame.event_mono_ns<=250_000_000)
    self.remaining,self.trigger=distance,trigger
    self.upcoming=upcoming
    if upcoming>=1 and abs(current-upcoming)>=.1:
      ident=self.scope+repr(source)+':'+self.approach.identity+':'+str(upcoming)
      self.identity=hashlib.sha256(ident.encode()).hexdigest()
      key = ((map_frame.producer_session, map_frame.source, map_frame.way_id,
              map_frame.next_way_id, round(current, 3), round(upcoming, 3))
             if map_frame.way_id > 0 and map_frame.next_way_id > 0 else None)
      self.warning_identity=self.warning_boundary.update(self.identity, key=key, now=now,
                                                          distance=raw_distance, speed=speed)
      self.eta=(distance-trigger)/max(speed,.1) if not lower or forecast_valid else math.inf
    if self.approach.active and self.identity:
      return acc.Observation(acc.ObservationKind.VALID,acc.Candidate('map',
        acc.ObservationIdentity(acc.IdentityKind.GEOGRAPHIC,value=self.identity),selected))
    return observation

  def warn(self, now_ns, *, eligible, target_change, speed, settings):
    now=now_ns/1e9
    self.warning_event=self.warning.update(now,self.warning_identity,self.eta,target_change,
      eligible,settings.custom.warning_lead)
    self.warning_pin=self.pin.update(now,self.warning_event,self.warning_identity,self.remaining,speed,
                                    eligible)


class ShortMapZone:
  """Skip an intermediate zone only with two fresh, uniquely connected boundaries.

  The provider's facts remain unchanged. This consumer transforms the selected
  target path, retaining the prior zone through the known short interval. Any
  loss of route continuity releases the policy; it cannot invent map authority.
  """
  def __init__(self):
    self.checkpoint = None
    self.active = False

  def project(self, observation, frame, *, now_ns, speed, enabled, allow_start=True):
    self.active = False
    old = self.checkpoint
    fresh = (enabled and frame is not None and observation.kind is acc.ObservationKind.VALID and
             observation.candidate is not None and math.isfinite(speed) and 0 <= speed <= 70)
    if not fresh:
      if old is not None and (not enabled or not 0 <= now_ns-old['seen'] <= 1_000_000_000):
        self.checkpoint = None
      return observation, frame
    source = (frame.producer_session, frame.generation, frame.source)
    current = observation.candidate.speed_mps
    if old is not None:
      continuous = (source == old['source'] and 0 <= now_ns-old['seen'] <= 1_000_000_000 and
                    0 <= now_ns-old['start'] <= 600_000_000_000)
      before = (abs(current-old['current']) < .1 and abs(frame.next_speed_mps-old['short']) < .1 and
                frame.next_way_id == old['next_way'] and frame.following_way_id == old['end_way'] and
                abs(frame.following_speed_mps-old['following']) < .1 and
                0 <= frame.next_distance_m < frame.next_end_distance_m <= 2000)
      inside = (abs(current-old['short']) < .1 and abs(frame.next_speed_mps-old['following']) < .1 and
                frame.next_way_id == old['end_way'] and 0 <= frame.next_distance_m <= 2000)
      remaining = frame.next_end_distance_m if before else frame.next_distance_m
      progress = max(20., speed*max(0.,(now_ns-old['seen'])/1e9)+10.)
      length = frame.next_end_distance_m-frame.next_distance_m if before else old['length']
      still_short = speed > 1. and math.isfinite(length) and 0 < length/speed < 5.
      if not still_short or not continuous or not (before or inside) or not math.isfinite(remaining) or not old['remaining']-progress <= remaining <= old['remaining']+20.:
        self.checkpoint = old = None
    if old is None and allow_start and speed > 1.:
      length = frame.next_end_distance_m-frame.next_distance_m
      qualified = (all(math.isfinite(x) for x in (length,frame.next_distance_m,frame.next_end_distance_m,frame.next_speed_mps,frame.following_speed_mps)) and
        0 <= frame.next_distance_m < frame.next_end_distance_m <= 2000 and
        1 <= frame.next_speed_mps <= 80 and 1 <= frame.following_speed_mps <= 80 and
        abs(current-frame.next_speed_mps) >= .1 and abs(frame.next_speed_mps-frame.following_speed_mps) >= .1 and
        frame.next_way_id > 0 and frame.following_way_id > 0 and frame.next_way_id != frame.following_way_id and
        length/speed < 5.)
      if qualified:
        old = self.checkpoint = dict(source=source,current=current,short=frame.next_speed_mps,
          following=frame.following_speed_mps,next_way=frame.next_way_id,end_way=frame.following_way_id,
          start=now_ns,seen=now_ns,remaining=frame.next_end_distance_m,length=length)
        before = True
    if old is None:
      return observation, frame
    if before:old['length'] = frame.next_end_distance_m-frame.next_distance_m
    old['seen'] = now_ns
    old['remaining'] = frame.next_end_distance_m if before else frame.next_distance_m
    self.active = True
    upcoming = old['following'] if abs(old['following']-old['current']) >= .1 else 0.
    projected = replace(frame,speed_mps=old['current'],next_speed_mps=upcoming,
      next_distance_m=old['remaining'] if upcoming else 0.,next_end_distance_m=0.,following_speed_mps=0.,
      following_way_id=0,next_way_id=old['end_way'] if upcoming else 0)
    selected = replace(observation,candidate=replace(observation.candidate,speed_mps=old['current']))
    return selected, projected
