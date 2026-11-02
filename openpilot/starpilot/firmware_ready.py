"""SP-032 native startup acknowledgement. Caller holds the firmware read lease."""
import os
import secrets
import select
import socket
import time

PREFIX = "STARPILOT_PANDAD_READY_"

class StartupError(RuntimeError):pass

class NativeStartup:
  def __init__(self, lease_fd):
    self.parent,self.child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    self.lease_fd=lease_fd
    self.nonce=secrets.token_hex(16)
    self.spawned=False

  @property
  def pass_fds(self):
    return self.child.fileno(),self.lease_fd

  def environment(self,environment):
    env={k:v for k,v in environment.items() if not k.startswith(PREFIX)}
    env.update({PREFIX+"FD":str(self.child.fileno()),PREFIX+"LEASE_FD":str(self.lease_fd),PREFIX+"NONCE":self.nonce})
    return env

  def child_started(self):
    self.child.close()
    self.spawned=True

  def wait(self,process,stop_requested,*,timeout=15.0):
    if not self.spawned or timeout <= 0:
      raise StartupError("Invalid native startup wait")
    deadline=time.monotonic()+timeout
    expected=f"SP032_FW_READY_V1 {self.nonce} {process.pid}\n".encode()
    while True:
      if stop_requested():raise StartupError("Native startup cancelled")
      if process.poll() is not None:raise StartupError("Native child exited before readiness")
      remaining=deadline-time.monotonic()
      if remaining <= 0:raise StartupError("Native readiness timeout")
      readable,_,_=select.select([self.parent],[],[],min(.1,remaining))
      if not readable:continue
      raw=self.parent.recv(257)
      if raw!=expected:raise StartupError("Invalid native readiness acknowledgement")
      if stop_requested() or process.poll() is not None:
        raise StartupError("Native readiness lost")
      return

  def close(self):
    self.child.close()
    self.parent.close()

  def __enter__(self):return self
  def __exit__(self,*args):self.close()
