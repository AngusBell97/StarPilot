"""Parked local UI actions. This directory is outside Params and logger backups."""
import sqlite3
import hashlib
import json
from pathlib import Path

from .store import CornerStore,MEMORY_ROOT


class MemorySettings:
  def __init__(self,is_onroad,root=MEMORY_ROOT):
    self.is_onroad=is_onroad
    self.root=root
    self.rows=[]
    self.stats={'corners':0,'bytes':0,'observations':0}
    self.status='Select to inspect local memory'
    self.generation=None

  def snapshot(self):
    return hashlib.sha256(json.dumps((self.generation,self.rows,self.stats),sort_keys=True).encode()).hexdigest().encode()

  def action(self,action,key='',*,expected=None):
    if self.is_onroad():
      self.status='Park to manage corner memory'
      return False
    if action=='refresh' and not (Path(self.root)/'corners.sqlite').exists():
      self.rows=[]
      self.generation=None
      self.stats={'corners':0,'bytes':0,'observations':0}
      self.status='0 corners, 0.0 MiB'
      return True
    try:
      with CornerStore(self.root,open_database=action!='clear' or expected is not None,check_integrity=False) as store:
        if self.is_onroad():
          self.status='Park to manage corner memory'
          return False
        if expected is not None:
          self.rows=store.recent();self.stats=store.stats();self.generation=store.generation
          if self.snapshot()!=expected:
            self.status='Memory changed; refresh before retrying'
            return False
        if action=='clear': store.clear()
        elif action=='forget': store.forget(key)
        elif action=='undo': store.undo()
        elif action!='refresh': raise ValueError('Unknown memory action')
        self.rows=store.recent()
        self.stats=store.stats()
        self.generation=store.generation
      self.status=f"{self.stats['corners']} corners, {self.stats['bytes']/1048576:.1f} MiB"
      if self.stats.get('full'): self.status='Memory full: '+self.status
      return True
    except BlockingIOError:
      self.status='Memory busy; wait for drive shutdown'
    except (OSError,sqlite3.Error,ValueError):
      self.status='Memory unavailable; no change confirmed'
    return False

  def labels(self,metric=False):
    factor=3.6 if metric else 2.2369362921
    unit='km/h' if metric else 'mph'
    result={}
    for row in self.rows:
      direction=('N','NE','E','SE','S','SW','W','NW')[round(row['heading']/45)%8]
      speed='/'.join(str(round(v*factor)) for v in row['profile'])
      label=f"{row['key'][:6]} {direction} {speed} {unit} · {row['count']} passes · {row['reason']}"
      result[label]=row['key']
    return result
