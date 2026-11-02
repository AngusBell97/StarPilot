"""Private bounded preferences. Only worker/parked management opens this store."""
from contextlib import contextmanager
from pathlib import Path
import hashlib
import hmac
import math
import os
import secrets
import shutil
import sqlite3
import struct
import time

from .types import MPH, PRIOR_VERSION, Preference

MEMORY_ROOT = Path('/data/starpilot/corner_memory')
DB_LIMIT = 64*1024*1024
JOURNAL_RESERVE = 8*1024*1024


class MemoryFull(OSError):
  pass


def _pack(profile):
  if len(profile)!=3 or not all(math.isfinite(v) and 5<=v<=40 for v in profile):
    raise ValueError('Invalid corner speed profile')
  return struct.pack('<3d',*profile)


def _unpack(profile):
  return struct.unpack('<3d',profile)


class CornerStore:
  """Lifetime lock: worker closes on offroad before parked actions are reachable.

  No competing connection may unlink or modify the database while this object
  exists. clear() preserves the lock inode, replaces only its own closed DB.
  """
  def __init__(self,root=MEMORY_ROOT,max_corners=10000,open_database=True,check_integrity=True):
    self.root=Path(root)
    if self.root.is_symlink(): raise OSError('Invalid memory directory')
    self.root.mkdir(parents=True,exist_ok=True,mode=0o700)
    os.chmod(self.root,0o700)
    self.path=self.root/'corners.sqlite'
    self.max_corners=min(10000,max_corners)
    self.frozen=False
    self.check_integrity=check_integrity
    self.conn=None
    self.lock_fd=os.open(self.root/'memory.lock',os.O_CREAT|os.O_RDWR|getattr(os,'O_NOFOLLOW',0),0o600)
    try:
      import fcntl
      fcntl.flock(self.lock_fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
      if open_database: self._open()
    except BaseException:
      if self.conn: self.conn.close()
      os.close(self.lock_fd)
      self.lock_fd=None
      raise

  def _open(self):
    if self.path.is_symlink(): raise OSError('Invalid memory file')
    if self.path.exists() and self.path.stat().st_size>DB_LIMIT: raise MemoryFull('Memory full')
    self.conn=sqlite3.connect(self.path,timeout=0,isolation_level=None)
    self.conn.execute('PRAGMA journal_mode=DELETE')
    self.conn.execute('PRAGMA synchronous=FULL')
    self.conn.execute('PRAGMA secure_delete=ON')
    self.conn.execute('PRAGMA foreign_keys=ON')
    page_size=self.conn.execute('PRAGMA page_size').fetchone()[0]
    self.conn.execute(f'PRAGMA max_page_count={DB_LIMIT//page_size}')
    if self.check_integrity and self.conn.execute('PRAGMA quick_check').fetchone()[0]!='ok': raise OSError('Memory unavailable')
    version=self.conn.execute('PRAGMA user_version').fetchone()[0]
    if version not in (0,1): raise OSError('Unsupported memory version')
    self.conn.executescript('''
      CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY,value TEXT NOT NULL);
      CREATE TABLE IF NOT EXISTS corners(
        key TEXT PRIMARY KEY CHECK(length(key)=64), revision TEXT NOT NULL CHECK(length(revision)<=80),
        prior BLOB NOT NULL CHECK(length(prior)=24), profile BLOB NOT NULL CHECK(length(profile)=24),
        shape BLOB NOT NULL CHECK(length(shape)<=768), heading INTEGER NOT NULL, direction INTEGER NOT NULL,
        count INTEGER NOT NULL, reason TEXT NOT NULL CHECK(length(reason)<64), day INTEGER NOT NULL,
        version TEXT NOT NULL, active INTEGER NOT NULL DEFAULT 1, family TEXT NOT NULL);
      CREATE TABLE IF NOT EXISTS observations(
        id INTEGER PRIMARY KEY, key TEXT NOT NULL REFERENCES corners(key) ON DELETE CASCADE,
        token TEXT NOT NULL UNIQUE CHECK(length(token)=64), profile BLOB NOT NULL CHECK(length(profile)=24),
        before BLOB NOT NULL CHECK(length(before)=24), before_count INTEGER NOT NULL,
        before_reason TEXT NOT NULL CHECK(length(before_reason)<64), faster INTEGER NOT NULL,
        applied INTEGER NOT NULL);
      CREATE INDEX IF NOT EXISTS by_corner ON observations(key,id);
      PRAGMA user_version=1;
    ''')
    for key in ('salt','generation'):
      self.conn.execute('INSERT OR IGNORE INTO meta VALUES (?,?)',(key,secrets.token_hex(32)))
    self.salt=bytes.fromhex(self.conn.execute("SELECT value FROM meta WHERE key='salt'").fetchone()[0])
    self.generation=self.conn.execute("SELECT value FROM meta WHERE key='generation'").fetchone()[0]
    os.chmod(self.path,0o600)

  def _key(self,key):
    return hmac.new(self.salt,key.encode(),hashlib.sha256).hexdigest()

  @contextmanager
  def transaction(self,valid=lambda:True):
    if self.frozen: raise MemoryFull('Learning paused: memory unavailable')
    total=sum(p.stat().st_size for p in self.root.iterdir() if p.is_file())
    if total>DB_LIMIT+JOURNAL_RESERVE or shutil.disk_usage(self.root).free<JOURNAL_RESERVE:
      self.frozen=True
      raise MemoryFull('Memory full')
    if not valid(): raise ValueError('Observation cancelled')
    self.conn.execute('BEGIN IMMEDIATE')
    try:
      generation=self.conn.execute("SELECT value FROM meta WHERE key='generation'").fetchone()[0]
      if generation!=self.generation: raise ValueError('Memory changed')
      yield
      if not valid(): raise ValueError('Observation cancelled')
      self.conn.execute('COMMIT')
    except BaseException as error:
      if self.conn.in_transaction: self.conn.execute('ROLLBACK')
      if isinstance(error,sqlite3.Error): self.frozen=True
      raise

  def lookup(self,corner):
    row=self.conn.execute('SELECT profile,count,reason,revision,version,active FROM corners WHERE key=?',(self._key(corner.key),)).fetchone()
    # A local map window cannot prove that a whole way's other bends changed.
    # Exact key/revision/version equality is the recall authority; active is UI
    # information only and must not suppress a later exact geometry match.
    if row is None or row[3]!=corner.revision or row[4]!=PRIOR_VERSION: return None
    profile=_unpack(row[0])
    _pack(profile)
    return Preference(profile,row[1],row[2])

  def observe(self,corner,profile,token,faster_allowed,valid=lambda:True):
    packed=_pack(profile)
    prior=_pack(corner.prior)
    if len(token)>128 or not token: raise ValueError('Invalid observation token')
    if len(corner.shape)>96 or any(abs(v)>32767 for p in corner.shape for v in p): raise ValueError('Invalid corner shape')
    key=self._key(corner.key)
    token=self._key(token)
    with self.transaction(valid):
      if self.conn.execute('SELECT 1 FROM observations WHERE token=?',(token,)).fetchone(): return 'duplicate'
      row=self.conn.execute('SELECT profile,count,reason,revision FROM corners WHERE key=?',(key,)).fetchone()
      if row and row[3]!=corner.revision: return 'geometry_mismatch'
      if row is None:
        if self.conn.execute('SELECT COUNT(*) FROM corners').fetchone()[0]>=self.max_corners: raise MemoryFull('Memory full')
        shape=struct.pack('<'+'h'*(len(corner.shape)*2),*(v for p in corner.shape for v in p))
        self.conn.execute('INSERT INTO corners VALUES (?,?,?,?,?,?,?,?,?,?,?,1,?)',
                          (key,corner.revision,prior,prior,shape,corner.heading,corner.direction,0,'Unconfirmed',int(time.time()//86400),PRIOR_VERSION,
                           self._key(f'{corner.way_id}:{corner.direction}')))
        old,count,reason=corner.prior,0,'Unconfirmed'
      else:
        old,count,reason=_unpack(row[0]),row[1],row[2]
      history=self.conn.execute('SELECT profile,faster,applied FROM observations WHERE key=? ORDER BY id DESC LIMIT 2',(key,)).fetchall()
      previous=_unpack(history[0][0]) if history else None
      large_slower=[a<b-8*MPH-.001 for a,b in zip(profile,old)]
      slower_bins=[a<b-.5*MPH and (not large_slower[i] or (previous is not None and abs(previous[i]-a)<=MPH))
                   for i,(a,b) in enumerate(zip(profile,old))]
      slower=any(slower_bins)
      large_faster=any(a>b+8*MPH+.001 for a,b in zip(profile,old))
      faster=bool(faster_allowed and not any(a<b-.5*MPH for a,b in zip(profile,old)) and not large_faster and any(a>b+.5*MPH for a,b in zip(profile,old)))
      consistent=faster and len(history)==2 and all(h[1] and not h[2] and max(abs(a-b) for a,b in zip(profile,_unpack(h[0])))<=1*MPH for h in history)
      adopted=old
      new_reason='Review needed' if any(large_slower) or large_faster else 'Slower-only evidence' if not faster_allowed else 'Awaiting repeated faster evidence'
      if slower:
        adopted=tuple(max(target,value-2*MPH) if slower_bins[i] else value for i,(target,value) in enumerate(zip(profile,old)))
        new_reason='Provisional slower preference'
      elif consistent:
        adopted=tuple(min(target,value+MPH) if target>value+.5*MPH else value for target,value in zip(profile,old))
        new_reason='Repeated faster preference'
      changed=adopted!=old
      self.conn.execute('INSERT INTO observations(key,token,profile,before,before_count,before_reason,faster,applied) VALUES(?,?,?,?,?,?,?,?)',
                        (key,token,packed,_pack(old),count,reason,int(faster),int(changed)))
      self.conn.execute('DELETE FROM observations WHERE key=? AND id NOT IN (SELECT id FROM observations WHERE key=? ORDER BY id DESC LIMIT 8)',(key,key))
      self.conn.execute('UPDATE corners SET profile=?,count=?,reason=?,day=? WHERE key=?',(_pack(adopted),min(count+1,1000000),new_reason,int(time.time()//86400),key))

    # A receipt is returned only after transaction.__exit__ commits successfully.
    return 'stored_applied' if changed else 'stored_unapplied'

  def recent(self,limit=20):
    rows=self.conn.execute('SELECT key,profile,count,reason,heading,direction,active FROM corners ORDER BY day DESC,rowid DESC LIMIT ?', (min(100,max(1,limit)),)).fetchall()
    return [dict(key=r[0],profile=_unpack(r[1]),count=r[2],reason=r[3] if r[6] else 'Map changed',heading=r[4],direction=r[5]) for r in rows]

  def mark_map(self,way_id,direction,corners):
    family=self._key(f'{way_id}:{direction}')
    keys={self._key(c.key) for c in corners}
    rows=self.conn.execute('SELECT key,active FROM corners WHERE family=?',(family,)).fetchall()
    changes=[(int(key in keys),key) for key,active in rows if bool(active)!=(key in keys)]
    if changes:
      with self.transaction(): self.conn.executemany('UPDATE corners SET active=? WHERE key=?',changes)

  def forget(self,key):
    # UI supplies already HMAC'd key; internal callers may supply raw key.
    with self.transaction():
      self.conn.execute('DELETE FROM corners WHERE key IN (?,?)',(key,self._key(key)))

  def undo(self):
    with self.transaction():
      row=self.conn.execute('SELECT id,key,before,before_count,before_reason FROM observations WHERE applied=1 ORDER BY id DESC LIMIT 1').fetchone()
      if row:
        self.conn.execute('UPDATE corners SET profile=?,count=?,reason=? WHERE key=?',(row[2],row[3],row[4],row[1]))
        self.conn.execute('DELETE FROM observations WHERE key=? AND id>=?',(row[1],row[0]))

  def stats(self):
    count=self.conn.execute('SELECT COUNT(*) FROM corners').fetchone()[0]
    return {'corners':count,
            'observations':self.conn.execute('SELECT COUNT(*) FROM observations').fetchone()[0],
            'bytes':sum(p.stat().st_size for p in self.root.iterdir() if p.is_file()),'full':self.frozen or count>=self.max_corners}

  def clear(self):
    if self.conn: self.conn.close()
    self.conn=None
    # Lock inode remains locked. No other store can race unlink/recreation.
    for name in ('corners.sqlite','corners.sqlite-journal','corners.sqlite-wal','corners.sqlite-shm','corners.backup','corners.tmp'):
      (self.root/name).unlink(missing_ok=True)
    fd=os.open(self.root,os.O_RDONLY|getattr(os,'O_DIRECTORY',0))
    try: os.fsync(fd)
    finally: os.close(fd)
    self.frozen=False
    self._open()
    if self.stats()['corners']!=0: raise OSError('Clear failed')

  def close(self):
    if self.conn: self.conn.close(); self.conn=None
    if self.lock_fd is not None: os.close(self.lock_fd); self.lock_fd=None

  def __enter__(self): return self
  def __exit__(self,*_args): self.close()
