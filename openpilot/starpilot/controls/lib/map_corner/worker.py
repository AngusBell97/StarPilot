"""All map/SQLite/manual passage work lives outside the planner process."""
from dataclasses import replace
import math
import os
import secrets
import time

from .diagnostics import DiagnosticSnapshot,MAX_COUNT

from .types import PointCap, Result, Sample
from .controller import approach_speed, corner_positions, route_envelope
from .geometry import physical_speed


def visual_agrees(frame,corner,progress):
  if not frame.visual_valid or not frame.visual_profile: return False
  d=corner.apex-progress
  if d<0: d=0
  nearest=min(frame.visual_profile,key=lambda p:abs(p[0]-d))
  _,curvature,unclamped=nearest
  expected=corner.curvature[1]
  return bool(unclamped and abs(nearest[0]-d)<=20 and curvature*corner.turn>0 and
              .5<=abs(curvature)/expected<=2 and abs(curvature)>=.001)


class Engine:
  def __init__(self,provider,store=None):
    from .learning import Observer
    self.provider=provider
    self.store=store
    self.observer=Observer()
    self.session=secrets.token_hex(16)
    self.key=''
    self.fence=None
    self.last_frame=None
    self.preferences={}
    self.map_revision=''
    self.anticipated=set()
    self.last_match_revision=''
    self.stats={'frames':0,'qualified':0,'late':0,'commits':0,'rejected':0}
    self.late_points={}
    self.late_overflow=False
    self.command_history_valid=False
    self.previous_route=None
    self.previous_safe=False
    self.resuming=False
    self.anticipation_stamp=None
    self.continuation_deficit=0.0
    self.diagnostic_counts={}
    self.reset_reason='reset'
    self.provider_ms=0.0
    self.preview_ms=0.0
    self.preview_complete=-1

  def count_diagnostic(self,reason):
    self.diagnostic_counts[reason]=min(MAX_COUNT,self.diagnostic_counts.get(reason,0)+1)

  def step(self,frame,valid=lambda:True):
    started=time.perf_counter()
    self.provider_ms=self.preview_ms=0.0
    self.preview_complete=-1
    result=self._step(frame,valid)
    reason={'Road match contested':'contested','Corner acquired too late':'late_target',
      'Route preview unavailable':'preview_incomplete','No eligible map corner':'no_corner',
      'Map corner':'ready','Remembered corner':'ready'}.get(result.status,getattr(self.provider,'reason','unknown'))
    self.count_diagnostic(reason)
    observer=self.observer
    counters=dict(self.diagnostic_counts)
    counters.update(observer.rejections)
    tracker=getattr(self.provider,'tracker',None)
    diagnostic=DiagnosticSnapshot(sequence=frame.sequence,control_epoch=frame.control_epoch,
      observation_fence=frame.observation_fence,compute_ms=(time.perf_counter()-started)*1000,
      provider_ms=self.provider_ms,preview_ms=self.preview_ms,
      gps_age_ms=(frame.now-frame.fix.received_at)*1000 if frame.fix else -1,
      accuracy_m=frame.fix.accuracy if frame.fix else -1,control_reason=reason,
      control_trusted=getattr(tracker,'control_trusted',False),match_reason=getattr(tracker,'control_reason','unknown'),
      fork_free_metres=getattr(tracker,'fork_free_metres',0),
      learning_reason=observer.last_reason,learning_state='active' if observer.active else ('blocked' if observer.blocked else 'waiting'),
      memory_status='available' if self.store else 'unavailable',reset_reason=self.reset_reason,
      confounds=frame.confound_flags,faster_allowed=frame.clear_conditions,
      preview_complete=self.preview_complete,common_points=len(result.points),options=len(result.route_options),
      option_points=sum(map(len,result.route_options)),late_points=len(self.late_points),
      decodes=getattr(self.provider,'decode_count',0),reindexes=getattr(self.provider,'reindex_count',0),
      counters=tuple(sorted(counters.items())))
    return replace(result,diagnostics=diagnostic)

  def reset_command_history(self):
    self.previous_route=None
    self.previous_safe=False
    self.resuming=False
    self.anticipation_stamp=None
    self.continuation_deficit=0.0
    self.command_history_valid=False

  def _step(self,frame,valid=lambda:True):
    self.stats['frames']+=1
    blank=Result(frame.now,frame.sequence,frame.control_epoch,frame.odometer)
    self.resuming=False
    self.command_history_valid=bool(self.last_frame and frame.sequence==self.last_frame.sequence+1 and
      0<frame.now-self.last_frame.now<=.25 and self.fence==frame.observation_fence)
    if self.fence!=frame.observation_fence:
      self.reset_reason='input_fenced'
      self.count_diagnostic('input_fenced')
      self.reset_command_history()
      self.observer.reset()
      self.fence=frame.observation_fence
      self.anticipated.clear()
      self.late_points.clear()
      self.late_overflow=False
    if self.last_frame and (frame.sequence!=self.last_frame.sequence+1 or frame.now-self.last_frame.now>.25):
      self.reset_reason='sequence_gap'
      self.count_diagnostic('sequence_gap')
      previous=self.last_frame
      resumable=bool(self.previous_route and self.fence==frame.observation_fence and
        previous.control_epoch==frame.control_epoch and previous.automated and frame.automated and
        not previous.fault and not frame.fault and 0<frame.now-previous.now<=1 and
        0<=frame.odometer-previous.odometer<=30 and
        (previous.applied_map_cap>0 or self.previous_safe or self.bridge_valid(frame)))
      if resumable: self.resuming=True
      else: self.reset_command_history()
      self.observer.reset()
      self.anticipated.clear()
    self.last_frame=frame
    started=time.perf_counter()
    match=self.provider.update(frame.fix,frame.now)
    self.provider_ms=(time.perf_counter()-started)*1000
    contested=bool(getattr(getattr(self.provider,'tracker',None),'contested',False))
    trusted=getattr(getattr(self.provider,'tracker',None),'control_trusted',False) is True
    if match is None or (contested and not trusted):
      self.reset_reason='match_lost'
      self.reset_command_history()
      self.key=''
      self.observer.reset()
      self.anticipated.clear()
      return replace(blank,status='Road match contested' if contested else self.provider.status)
    if contested:
      # Control continuity is not permission to learn a preference at an
      # ambiguous identity. Reset passage history even when control is trusted.
      self.observer.reset()
      frame=replace(frame,confounded=True)
    self.stats['qualified']+=1
    progress=match.progress+max(0,frame.now-frame.fix.received_at)*frame.speed
    if getattr(self.provider,'tile',None) is not None:
      return self._route_step(frame,match,progress,blank,valid)
    self.reset_command_history()
    # Include the whole bend and its topology margin inside the loaded tiles.
    corners=[c for c in match.corners if c.exit+15>=progress and c.exit-progress<=1100]
    if not corners:
      self.key=''
      self.observer.reset()
      self.anticipated.clear()
      return replace(blank,status='No eligible map corner')
    corner=corners[0]
    if self.key!=corner.key:
      prior_preference=self.store.lookup(corner) if self.store else None
      proposed=prior_preference.profile if prior_preference else corner.prior
      ramp=max(0,min(6,(max(-.6,frame.acceleration)+.6)/.3))
      reserve=frame.speed*ramp+max(0,frame.acceleration)*ramp*ramp/2+match.uncertainty
      late=any(s-progress<max(0,frame.speed**2-min(v,p)**2)/1.2+reserve
               for s,v,p in zip(corner_positions(corner),proposed,corner.admission_prior or corner.prior))
      anticipated=corner.key in self.anticipated and match.revision==self.last_match_revision
      if late and not anticipated:
        self.stats['late']+=1
        self.observer.reset()
        return replace(blank,status='Corner acquired too late')
      self.key=corner.key
      self.preferences={}
      for c in corners[:8]:
        p=self.store.lookup(c) if self.store else None
        self.preferences[c.key]=p.profile if p else c.prior
    self.last_match_revision=match.revision
    self.anticipated={c.key for c in corners[:8]}
    confirmed=visual_agrees(frame,corner,progress)
    adopted=self.preferences.get(corner.key,corner.prior)
    owner=frame.owner
    if owner=='map' and frame.binding_key!=corner.key: owner='other'
    sample=Sample(frame.now,progress,frame.speed,frame.automated,owner,frame.intervention,
                  frame.confounded,frame.fault,confirmed,frame.clear_conditions,frame.learn,
                  self.session+frame.observation_id,frame.odometer)
    observed=self.observer.update(replace(corner,prior=adopted),sample)
    if observed and self.store and frame.learn and valid():
      try:
        receipt=self.store.observe(corner,observed.profile,observed.token,observed.faster_allowed,valid)
        self.stats['commits']+=1
        self.observer.last_reason=receipt if receipt in ('stored_applied','stored_unapplied','duplicate','geometry_mismatch') else 'unknown'
        self.count_diagnostic(self.observer.last_reason)
      except (OSError,ValueError):
        self.stats['rejected']+=1
        self.observer.last_reason='store_rejected'
        self.count_diagnostic('store_rejected')
    points=[]
    remembered=False
    for c in corners[:8]:
      profile=self.preferences.get(c.key,c.prior)
      # The visual match above describes this bend only. A visible first bend
      # must not authorize faster remembered speeds at a later hidden bend.
      if c.key!=corner.key: profile=tuple(min(v,p) for v,p in zip(profile,c.prior))
      remembered |= profile!=c.prior
      for i,s in enumerate(corner_positions(c)):
        if s>=progress-10:
          points.append(PointCap(max(0,s-progress),profile[i],c.prior[i],c.key,ceiling=physical_speed(c.curvature[i]),admission_speed=(c.admission_prior or c.prior)[i]))
    uncertainty=match.uncertainty+frame.speed*(max(0,frame.now-frame.fix.received_at)+.2)
    return replace(blank,key=corner.key,progress=progress,exit=corner.exit,points=tuple(points),
                   status='Remembered corner' if remembered else 'Map corner',remembered=remembered,
                   uncertainty=uncertainty,visual_confirmed=confirmed)

  def _route_step(self,frame,match,progress,blank,valid):
    from .routes import preview_routes
    reference=frame.speed
    if math.isfinite(frame.reference_speed) and frame.reference_speed>=0:
      reference=min(reference,frame.reference_speed)
    started=time.perf_counter()
    preview=preview_routes(self.provider.tile,match)
    self.preview_ms=(time.perf_counter()-started)*1000
    self.preview_complete=int(preview.complete)
    if match.revision!=self.last_match_revision:
      self.observer.reset()
      self.preferences={}
      self.anticipated.clear()
    self.last_match_revision=match.revision
    corners=[c for c in match.corners if c.exit+15>=progress and c.exit-progress<=1100 and c.exit<=preview.common_end]
    all_keys={c.key for c in corners}
    if len(corners)>8:
      corners=corners[:8]
      options=((),)
    else:
      extrapolated=progress-match.progress
      options=tuple(tuple(PointCap(max(0,p.distance-extrapolated),p.speed,p.prior,p.key,ceiling=p.ceiling,admission_speed=p.admission_speed) for p in path
                          if p.key not in all_keys and p.distance>=extrapolated-10) for path in preview.options)
    self.preferences={key:value for key,value in self.preferences.items() if key in all_keys}
    common=[];remembered=False
    for c in corners:
      if c.key not in self.preferences:
        p=self.store.lookup(c) if self.store else None
        self.preferences[c.key]=p.profile if p else None
      profile=self.preferences[c.key] or c.prior
      # Visual confirmation refers to the first common bend only.
      if c!=corners[0]: profile=tuple(min(v,p) for v,p in zip(profile,c.prior))
      remembered |= profile!=c.prior
      common.extend(PointCap(max(0,s-progress),profile[i],c.prior[i],c.key,ceiling=physical_speed(c.curvature[i]),admission_speed=(c.admission_prior or c.prior)[i])
                    for i,s in enumerate(corner_positions(c)) if s>=progress-10)
    uncertainty=match.uncertainty+frame.speed*(max(0,frame.now-frame.fix.received_at)+.2)
    targets=tuple(common)+tuple(p for path in options for p in path)
    seen={p.key for p in targets}
    self.late_points={key:stamp for key,stamp in self.late_points.items()
                      if key in seen or frame.now-stamp[0]<5 or frame.odometer-stamp[1]<80}
    for key in seen & self.late_points.keys(): self.late_points[key]=(frame.now,frame.odometer)
    common=tuple(p for p in common if p.key not in self.late_points)
    options=tuple(tuple(p for p in path if p.key not in self.late_points) for path in options)
    targets=common+tuple(p for path in options for p in path)
    late_count=len(self.late_points)
    bounded_options=len(options)<=32
    raw,_,_,_=route_envelope(tuple(common),options,cruise=math.inf,speed=frame.speed,
      acceleration=frame.acceleration,uncertainty=uncertainty,admission=True)
    commanded=bool(self.command_history_valid and frame.automated and not frame.fault and frame.applied_map_cap>0)
    old=self.previous_route
    history=bool(frame.automated and not frame.fault and (self.command_history_valid or self.resuming) and
                 old and old.control_epoch==frame.control_epoch and
                 0<frame.now-old.now<=(1 if self.resuming else .25) and 0<=frame.odometer-old.odometer<=30)
    oldraw=math.inf
    if history:
      oldraw,_,_,_=route_envelope(old.points,old.route_options,cruise=math.inf,speed=frame.speed,
        acceleration=frame.acceleration,uncertainty=uncertainty,travelled=frame.odometer-old.odometer,admission=True)
    direct=commanded and frame.applied_map_cap<=raw+.3
    allowance=max(0,.3-self.continuation_deficit)
    drop=max(0,oldraw-raw) if math.isfinite(oldraw) else math.inf
    # A safely previewed approach must be allowed to cross the current speed
    # before the next coordinator exchange can issue its first map command.
    supported=history and (commanded or self.resuming or self.previous_safe or self.bridge_valid(frame))
    continued=supported and drop<=allowance+1e-9
    anticipated=direct or continued
    if direct: self.continuation_deficit=0.0
    elif continued and raw<frame.speed: self.continuation_deficit=min(.3,self.continuation_deficit+drop)
    # Manual passage samples are learning evidence, not requests to start
    # automated braking. Do not discard the observed bend during the takeover.
    if frame.automated and raw<frame.speed and not anticipated:
      # Keep previously supported aggregate/common limits. A branch that becomes
      # binding too late remains permissive, even if its points were previewed.
      threshold=min(frame.speed,oldraw,frame.applied_map_cap or math.inf)-allowance if supported else frame.speed
      for p in targets:
        if approach_speed(min(p.speed,p.prior,p.admission_speed or p.prior),p.distance,frame.speed,frame.acceleration,uncertainty)<threshold:
          if p.key not in self.late_points:
            self.stats['late']+=1
            if len(self.late_points)>=256:
              self.late_overflow=True
              break
          self.late_points[p.key]=(frame.now,frame.odometer)
    if self.late_overflow:
      self.observer.reset()
      self.reset_command_history()
      return replace(blank,status='Route preview unavailable',route_options=((),))
    common=tuple(p for p in common if p.key not in self.late_points)
    options=tuple(dict.fromkeys(tuple(p for p in path if p.key not in self.late_points) for path in options))
    corner=corners[0] if corners and corners[0].key not in self.late_points else None
    confirmed=bool(corner and visual_agrees(frame,corner,progress))
    if corner:
      adopted=self.preferences.get(corner.key) or corner.prior
      owner=frame.owner if frame.binding_key==corner.key else 'other'
      sample=Sample(frame.now,progress,frame.speed,frame.automated,owner,frame.intervention,
        frame.confounded,frame.fault,confirmed,frame.clear_conditions,frame.learn,
        self.session+frame.observation_id,frame.odometer)
      observed=self.observer.update(replace(corner,prior=adopted),sample)
      if observed and self.store and frame.learn and valid():
        try:
          receipt=self.store.observe(corner,observed.profile,observed.token,observed.faster_allowed,valid)
          self.stats['commits']+=1
          self.observer.last_reason=receipt if receipt in ('stored_applied','stored_unapplied','duplicate','geometry_mismatch') else 'unknown'
          self.count_diagnostic(self.observer.last_reason)
        except (OSError,ValueError):
          self.stats['rejected']+=1
          self.observer.last_reason='store_rejected'
          self.count_diagnostic('store_rejected')
    else:
      self.observer.reset()
      self.observer.last_reason='no_unique_corner'
    self.key=corner.key if corner else preview.key
    usable=bool(common or any(options))
    status=('Remembered corner' if remembered else 'Map corner') if usable else (
      'Corner acquired too late' if seen & self.late_points.keys() else 'No eligible map corner')
    result=replace(blank,key=self.key,progress=progress,exit=corner.exit if corner else progress+1100,
      points=common,route_options=options,status=status,remembered=remembered and usable,
      uncertainty=uncertainty,visual_confirmed=confirmed)
    self.previous_route=result if usable else None
    published=raw
    # Deduplicating an over-capacity option list can change the envelope's
    # fallback; recalculate that case as well as newly rejected late targets.
    if len(self.late_points)!=late_count or not bounded_options:
      published,_,_,_=route_envelope(common,options,cruise=math.inf,speed=frame.speed,
        acceleration=frame.acceleration,uncertainty=uncertainty,admission=True)
    self.previous_safe=published>=reference
    if frame.automated and not frame.fault and (commanded or self.previous_safe):
      self.anticipation_stamp=(frame.now,frame.odometer)
    if not usable: self.reset_command_history()
    return result

  def bridge_valid(self,frame):
    stamp=self.anticipation_stamp
    return bool(stamp and 0<=frame.now-stamp[0]<=1 and 0<=frame.odometer-stamp[1]<=30)


def demote():
  """Run before heavy imports in both child and supervisor thread."""
  if os.name!='posix': raise OSError('Linux worker required')
  os.sched_setscheduler(0,os.SCHED_OTHER,os.sched_param(0))
  # Comma realtime tasks use big cores4/5; map/DB work stays on little cores.
  cpus=set(range(min(4,os.cpu_count() or 1)))
  try: os.sched_setaffinity(0,cpus)
  except OSError:
    # Container test hosts can restrict CPUs; never put the child on its sole
    # inherited RT core when no permitted background core is available.
    cpus=set(os.sched_getaffinity(0))-{4,5}
    if not cpus: raise
    os.sched_setaffinity(0,cpus)
  os.setpriority(os.PRIO_PROCESS,0,10)


def process_frame(engine,frame,flags,clock=time.monotonic):
  """Production dequeue step with an injectable clock for exact scheduling tests."""
  wall_now=clock()
  fenced=frame.observation_fence!=flags[2]
  if wall_now-frame.now>.5 or fenced:
    engine.count_diagnostic('input_fenced' if fenced else 'input_expired')
    engine.observer.reset()
    if fenced or not engine.last_frame or not 0<=wall_now-engine.last_frame.now<=1:
      engine.reset_command_history()
      engine.last_frame=None
    return None
  valid=lambda: bool(flags[0] and flags[1] and flags[2]==frame.observation_fence)
  try: result=engine.step(frame,valid)
  except Exception:
    engine.count_diagnostic('worker_error')
    engine.observer.reset()
    engine.reset_command_history()
    engine.last_frame=None
    result=Result(frame.now,frame.sequence,frame.control_epoch,frame.odometer,status='Map unavailable',
      diagnostics=DiagnosticSnapshot(sequence=frame.sequence,control_epoch=frame.control_epoch,
        observation_fence=frame.observation_fence,control_reason='worker_error',
        memory_status='available' if engine.store else 'unavailable',
        counters=tuple(sorted((engine.diagnostic_counts | engine.observer.rejections).items()))))
  return replace(result,diagnostics=replace(result.diagnostics,input_age_ms=(wall_now-frame.now)*1000))


def worker_main(inputs,outputs,flags,root,map_root,parent_pid,clock=None):
  import ctypes
  import queue
  import signal
  import sqlite3
  import time
  clock=time.monotonic if clock is None else clock
  store=None
  try:
    demote()
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(1,signal.SIGTERM,0,0,0)!=0 or os.getppid()!=parent_pid: return
    from .snapshot_provider import SnapshotProvider
    from .store import CornerStore
    # A quick OFF/ON can overlap the previous worker's bounded shutdown.
    deadline=time.monotonic()+3
    while flags[0]:
      try:
        store=CornerStore(root)
        break
      except BlockingIOError:
        if time.monotonic()>=deadline: break
        time.sleep(.05)
      except (OSError,sqlite3.Error,ValueError): break
    engine=Engine(SnapshotProvider(map_root),store)
    while flags[0]:
      try: frame=inputs.get(timeout=.25)
      except queue.Empty: continue
      if frame is None: continue  # supervisor's background feeder warm-up
      result=process_frame(engine,frame,flags,clock)
      if result is None: continue
      try: outputs.put_nowait(result)
      except queue.Full:
        engine.count_diagnostic('output_full')
        try: outputs.get_nowait()
        except queue.Empty: pass
        try: outputs.put_nowait(result)
        except queue.Full: pass
  finally:
    if store: store.close()
