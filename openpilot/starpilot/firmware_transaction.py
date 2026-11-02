"""SP-032 firmware barrier prototype. NOT integrated or installation-qualified.

The bootstrapped updater must hold reader() across EVERY firmware read/write,
reset/recovery and native-child startup validation. The installer must not replace
product files until writer() has durably recorded its intent. A native child may
continue supplying parked telemetry only after completing its startup checks.
A timeout, installer death or reboot never clears pending intent.

The existing installer remains responsible for authorization, identity/parked
guards, backup, exact payload allowlist, monitoring and independent verification.
finish() requires its fresh complete source/image verification callback.
"""
import contextlib
import fcntl
import json
import os
from pathlib import Path
import re
import stat

class BarrierError(RuntimeError):pass

def require(ok,message):
  if not ok:raise BarrierError(message)

def identity(info):return info.st_dev,info.st_ino

def valid_hex(value,length):
  return type(value) is str and re.fullmatch("[0-9a-f]{"+str(length)+"}",value) is not None

def intent_bytes(token,manifest):
  require(valid_hex(token,32) and valid_hex(manifest,64),"Invalid transaction identity")
  return (json.dumps(dict(version=1,token=token,manifest=manifest),sort_keys=True,separators=(",",":"))+"\n").encode()

class FirmwareBarrier:
  def __init__(self,directory,*,owner_uid):
    self.directory=Path(directory).absolute()
    self.owner_uid=owner_uid

  def _directory(self):
    require(self.directory.resolve()==self.directory,"Noncanonical barrier path")
    fd=os.open(self.directory,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW|os.O_CLOEXEC)
    try:
      info=os.fstat(fd)
      require(info.st_uid==self.owner_uid and stat.S_IMODE(info.st_mode)==0o700,"Untrusted barrier directory")
      return fd
    except BaseException:
      os.close(fd);raise

  @contextlib.contextmanager
  def _lock(self,exclusive):
    directory=self._directory()
    lock=None
    try:
      lock=os.open("mutex",os.O_RDWR|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=directory)
      info=os.fstat(lock)
      require(stat.S_ISREG(info.st_mode) and info.st_nlink==1 and info.st_uid==self.owner_uid and
              stat.S_IMODE(info.st_mode)==0o600,"Untrusted barrier mutex")
      try:
        fcntl.flock(lock,(fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH)|fcntl.LOCK_NB)
      except BlockingIOError:
        yield None
        return
      def check():
        require(identity(os.stat(self.directory,follow_symlinks=False))==identity(os.fstat(directory)),"Barrier directory replaced")
        require(identity(os.stat("mutex",dir_fd=directory,follow_symlinks=False))==identity(info),"Barrier mutex replaced")
      check()
      yield directory,check,lock
    finally:
      if lock is not None:os.close(lock)
      os.close(directory)

  @staticmethod
  def _pending(directory):
    try:
      os.stat("pending.json",dir_fd=directory,follow_symlinks=False)
      return True
    except FileNotFoundError:return False

  @contextlib.contextmanager
  def reader(self):
    with self._lock(False) as held:
      if held is None:
        yield False
        return
      directory,check,lock=held
      check()
      # Any existing intent, even malformed or unreadable, blocks the updater.
      if self._pending(directory):
        yield False
      else:
        lease=ReaderLease(lock,check)
        try:yield lease
        finally:lease.closed=True
      check()

  @contextlib.contextmanager
  def writer(self,token,manifest,*,resume=False,admit=None):
    intended=intent_bytes(token,manifest)
    with self._lock(True) as held:
      require(held is not None,"Firmware reader/writer is active")
      directory,check,lock=held
      # The composed adapter supplies live-contract and ordinary fresh guards.
      # Validate under the exclusive lease BEFORE recording durable write intent.
      if admit is not None:
        require(admit() is True,"Live firmware writer admission failed")
        check()
      if resume:
        require(self._pending(directory),"No pending transaction to resume")
      else:
        require(not self._pending(directory),"Unresolved firmware transaction")
        fd=os.open("pending.json",os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_CLOEXEC,0o600,dir_fd=directory)
        try:
          with os.fdopen(fd,"wb",closefd=False) as stream:
            stream.write(intended);stream.flush();os.fsync(fd)
        finally:os.close(fd)
        os.fsync(directory)
      lease=WriterLease(directory,check,intended,self.owner_uid)
      lease.check()
      try:yield lease
      finally:lease.closed=True

class WriterLease:
  def __init__(self,directory,check,intended,owner_uid):
    self.directory=directory;self.check_lock=check;self.intended=intended
    self.owner_uid=owner_uid;self.closed=False

  def check(self):
    require(not self.closed,"Closed firmware transaction")
    self.check_lock()
    fd=os.open("pending.json",os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=self.directory)
    try:
      info=os.fstat(fd)
      require(stat.S_ISREG(info.st_mode) and info.st_nlink==1 and
              info.st_uid in (0,self.owner_uid) and stat.S_IMODE(info.st_mode)==0o600,"Untrusted pending transaction")
      require(os.read(fd,4097)==self.intended,"Foreign or changed firmware transaction")
    finally:os.close(fd)

  def finish(self,verify_complete_release):
    self.check()
    # Caller supplies the maintained adapter's complete, fresh source/image
    # verifier. Returning a truthy receipt is insufficient; require explicit True.
    require(verify_complete_release() is True,"Complete release verification failed")
    self.check()
    os.unlink("pending.json",dir_fd=self.directory)
    os.fsync(self.directory)
    self.closed=True


class ReaderLease:
  def __init__(self,descriptor,check):
    self._descriptor=descriptor;self._check=check;self.closed=False

  @property
  def descriptor(self):
    require(not self.closed,"Closed firmware reader")
    self._check()
    # pass_fds transfers this same open-file description to the native child.
    # No unlock call: its inherited reference must survive a parent crash.
    return self._descriptor
