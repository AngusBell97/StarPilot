"""Distance-aligned manual passages; disengagement by itself is not a label."""
from collections import deque
import math
import secrets
import statistics

from .types import Observation


class Observer:
  def __init__(self):
    self.last_reason='waiting'
    self.completed={}
    self.rejections={}
    self.reset()

  def reset(self):
    self.corner=None
    self.history=deque(maxlen=80)
    self.previous=None
    self.active=False
    self.blocked=False
    self.started=0.0
    self.bins=[[],[],[]]
    self.last_progress=-math.inf
    self.faster=True
    self.token=''

  def reject(self,reason):
    self.last_reason=reason
    self.rejections[reason]=min(2147483647,self.rejections.get(reason,0)+1)
    self.active=False
    self.blocked=True
    self.bins=[[],[],[]]

  def update(self,corner,sample):
    if corner is None or not sample.learn:
      self.reset()
      return None
    if not all(math.isfinite(v) for v in (sample.now,sample.progress,sample.speed)) or not 0<=sample.speed<=55:
      self.reject('invalid sample')
      return None
    if self.corner is None or self.corner.key!=corner.key:
      self.reset()
      self.corner=corner
    elif self.previous and sample.progress<self.previous.progress-30:
      self.reset()
      self.corner=corner
    previous=self.previous
    self.previous=sample
    if previous and (not 0<sample.now-previous.now<=.25 or sample.progress<previous.progress-2):
      self.reject('gap or reverse')
    if sample.fault or sample.confounded:
      self.reject('confounded')
    if self.blocked: return None
    if sample.automated:
      if self.active:
        self.reject('reengaged')
        return None
      self.history.append((sample.now,sample.owner))
      while self.history and sample.now-self.history[0][0]>4: self.history.popleft()
      return None
    if not self.active:
      if not sample.intervention: return None
      recent=[(t,owner) for t,owner in self.history if sample.now-t<=1.1]
      owners={owner for _,owner in self.history}
      if not recent or sample.now-recent[0][0]<1 or recent[-1][0]<sample.now-.25 or any(o!='map' for _,o in recent) or 'visual' in owners or 'tie' in owners:
        self.reject('ownership')
        return None
      if sample.progress>corner.entry-5:
        self.reject('late takeover')
        return None
      done=self.completed.get(corner.key)
      if done and (sample.now-done[0]<120 or sample.odometer-done[1]<600):
        self.reject('repeat passage')
        return None
      self.active=True
      self.started=sample.now
      self.token=sample.observation_id or secrets.token_hex(16)
      # SP-031 user decision: weather is not a learning gate.
      self.faster=True
    if corner.entry<=sample.progress<=corner.exit:
      self.faster &= sample.visual_valid
    if sample.now-self.started>45:
      self.reject('timeout')
      return None
    if corner.entry<=sample.progress<=corner.exit and sample.progress-self.last_progress>=2:
      index=min(2,int((sample.progress-corner.entry)/max(1,corner.exit-corner.entry)*3))
      if len(self.bins[index])<96:
        self.bins[index].append((sample.progress,sample.speed))
      self.last_progress=sample.progress
    if sample.progress>corner.exit+10:
      if not all(len(b)>=3 and b[-1][0]-b[0][0]>=4 for b in self.bins):
        self.reject('incomplete profile')
        return None
      profile=tuple(statistics.median(v for _,v in b) for b in self.bins)
      if not all(5<=v<=40 for v in profile):
        self.reject('speed outside scope')
        return None
      # No label for a lateral-only takeover or an unchanged speed profile.
      if max(abs(v-p) for v,p in zip(profile,corner.prior))<.5:
        self.reject('no speed change')
        return None
      self.completed[corner.key]=(sample.now,sample.odometer)
      if len(self.completed)>64: self.completed.pop(next(iter(self.completed)))
      self.last_reason='observation_ready'
      result=Observation(corner,profile,self.token,bool(self.faster))
      self.active=False
      self.blocked=True
      return result
    return None
