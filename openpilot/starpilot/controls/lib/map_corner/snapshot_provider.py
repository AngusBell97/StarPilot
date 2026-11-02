"""SP-034 geometry from the same content-addressed map package selected by 7.0.

Selection and tile verification run only in the map worker. No legacy directory
fallback, network fetch, selector mutation, or planner-thread disk access.
"""
import hashlib
from pathlib import Path
import re
import stat
import time

from openpilot.starpilot.maps.storage import offline_root
from openpilot.starpilot.maps.shadow_lifecycle import _selected_receipt, _open_regular
from .provider import OfflineProvider

class SnapshotProvider(OfflineProvider):
  def __init__(self,root=None):
    self.package_root=offline_root() if root is None else Path(root)
    super().__init__(self.package_root)
    self.generation=None
    self.entries={}
    self.checked=None

  def _select(self,now):
    if self.checked is not None and 0<=now-self.checked<1.:return bool(self.generation)
    self.checked=now
    old=self.generation
    try:
      if not stat.S_ISDIR(self.package_root.lstat().st_mode):raise ValueError('Invalid map root')
      selected=_selected_receipt(self.package_root)
      if selected is None:raise ValueError('No selected package')
      generation,receipt,_=selected
      if generation==old:return True
      entries={}
      for entry in receipt['tiles']:
        relative=Path(entry['path'])
        if (type(entry.get('size')) is not int or not 0<entry['size']<=73<<20 or
            type(entry.get('sha256')) is not str or re.fullmatch('[0-9a-f]{64}',entry['sha256']) is None or
            relative.is_absolute() or len(relative.parts)!=3 or '..' in relative.parts or
            relative.as_posix()!=entry['path'] or '\\' in entry['path'] or entry['path'] in entries):
          raise ValueError('Invalid tile receipt')
        entries[entry['path']]=entry
      self.root=self.package_root/'generations'/generation
      self.generation=generation;self.entries=entries
    except (OSError,ValueError,KeyError,TypeError):
      self.generation=None;self.entries={}
    if old!=self.generation:
      self.tile=None;self.path=None;self.stamp=None;self.decoded={};self.bad_decodes={}
      self.failed_paths=None;self.failed_stamp=None;self.failed_position=None;self.retry_after=0.
      self.tracker.reset()
    return bool(self.generation)

  def _stamp(self,path):
    relative=path.relative_to(self.root)
    current=self.package_root
    for part in ('generations',self.generation,*relative.parts[:-1]):
      current=current/part
      if not stat.S_ISDIR(current.lstat().st_mode):raise ValueError('Invalid tile directory')
    entry=self.entries.get(relative.as_posix())
    info=path.lstat()
    if entry is None or not stat.S_ISREG(info.st_mode) or info.st_size!=entry['size']:
      raise ValueError('Tile does not match selected receipt')
    return info.st_size,info.st_mtime_ns,info.st_ino

  def _read_bytes(self,path):
    self._stamp(path)
    with _open_regular(path,max_bytes=16<<20) as stream:raw=stream.read((16<<20)+1)
    entry=self.entries[path.relative_to(self.root).as_posix()]
    if len(raw)!=entry['size'] or hashlib.sha256(raw).hexdigest()!=entry['sha256']:
      raise ValueError('Tile checksum mismatch')
    return raw

  def update(self,fix,now=None):
    now=time.monotonic() if now is None else now
    if not self._select(now):
      self.tracker.reset();self.reason='tile_missing';self.status='No selected offline map package'
      return None
    return super().update(fix,now)
