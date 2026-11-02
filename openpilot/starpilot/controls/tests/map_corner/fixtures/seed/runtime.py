"""Nonblocking transport; process creation/reaping occurs in a background thread."""
import queue
import threading
import time


class Runtime:
  def __init__(self,root='/data/starpilot/corner_memory',map_root='/data/media/0/osm/offline'):
    self.root=root
    self.map_root=map_root
    self.inputs=None
    self.outputs=None
    self.flags=None
    self.stopped=False
    self.failed=False
    self.worker_pid=None
    self.thread=threading.Thread(target=self._supervise,name='map-corner-supervisor',daemon=True)
    self.thread.start()

  def _supervise(self):
    process=None
    inputs=outputs=None
    try:
      import multiprocessing
      import os
      from .worker import demote,worker_main
      demote()
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
