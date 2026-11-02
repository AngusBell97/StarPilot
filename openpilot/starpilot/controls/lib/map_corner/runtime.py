"""Nonblocking transport; process creation/reaping occurs in a background thread."""
import queue
import threading
import time


class Runtime:
  def __init__(self,root='/data/starpilot/corner_memory',map_root=None):
    self.root=root
    self.map_root=map_root
    self.inputs=None
    self.outputs=None
    self.flags=None
    self.stopped=False
    self.failed=False
    self.worker_pid=None
    self.diagnostic_snapshot=None
    self.thread=threading.Thread(target=self._supervise,name='map-corner-supervisor',daemon=True)
    self.thread.start()

  def _supervise(self):
    process=None
    inputs=outputs=None
    emitter=None
    try:
      import multiprocessing
      import os
      from .worker import demote,worker_main
      from .diagnostics import DiagnosticEmitter
      demote()
      def emit_log(payload):
        # Logging dependencies/failures must never stop the worker starting.
        from openpilot.common.swaglog import cloudlog
        cloudlog.info("map_corner_diagnostics "+payload)
      emitter=DiagnosticEmitter(emit_log)
      context=multiprocessing.get_context('spawn')
      inputs=context.Queue(maxsize=64)
      outputs=context.Queue(maxsize=4)
      inputs.cancel_join_thread()
      outputs.cancel_join_thread()
      flags=context.RawArray('q',[1,0,0])
      # Queue starts its feeder on first put. Start it here so it inherits this
      # background thread's priority/affinity rather than the realtime caller's.
      inputs.put_nowait(None)
      process=context.Process(target=worker_main,args=(inputs,outputs,flags,self.root,self.map_root,os.getpid()),daemon=True)
      process.start()
      self.worker_pid=process.pid
      self.flags=flags
      self.inputs=inputs
      self.outputs=outputs
      while not self.stopped and process.is_alive():
        try: emitter.emit(self.diagnostic_snapshot,time.monotonic())
        except Exception: pass  # Telemetry cannot kill transport or control.
        time.sleep(.05)
      flags[0]=0
      process.join(timeout=1)
      if process.is_alive():
        process.terminate()
        process.join(timeout=1)
      self.failed=not self.stopped
    except Exception:
      self.failed=True
    finally:
      self.inputs=None
      self.outputs=None
      self.flags=None
      if process and process.is_alive(): process.terminate()
      if inputs: inputs.close()
      if outputs: outputs.close()
      if self.failed and not self.stopped and emitter is not None:
        from dataclasses import replace
        from .diagnostics import DiagnosticSnapshot
        # At most one final failure record, respecting the same rate/byte gate.
        time.sleep(max(0,min(1,emitter.last+1-time.monotonic())))
        if not self.stopped:
          snapshot=self.diagnostic_snapshot or DiagnosticSnapshot()
          try: emitter.emit(replace(snapshot,parent_reason='worker_failed'),time.monotonic())
          except Exception: pass

  def record_diagnostics(self,snapshot):
    self.diagnostic_snapshot=snapshot

  def exchange(self,frame):
    flags=self.flags
    inputs,outputs=self.inputs,self.outputs
    if flags is None or inputs is None or outputs is None: return None,False
    flags[1]=int(frame.learn)
    flags[2]=frame.observation_fence
    try: inputs.put_nowait(frame)
    except (queue.Full,ValueError,OSError):
      return None,True
    newest=None
    for _ in range(4):
      try: newest=outputs.get_nowait()
      except (queue.Empty,ValueError,OSError): break
    return newest,False

  def fence(self,value,learn):
    flags=self.flags
    if flags is not None:
      flags[1]=int(learn)
      flags[2]=value

  def close(self):
    self.stopped=True
    flags=self.flags
    if flags is not None: flags[0]=0
