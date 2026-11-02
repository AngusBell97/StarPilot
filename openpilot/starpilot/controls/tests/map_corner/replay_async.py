"""Deterministic single-worker FIFO schedule; fixed motion, not vehicle simulation."""
from collections import deque
from dataclasses import replace

class VirtualWorker:
  def __init__(self,engine,process,cost=lambda frame:0):
    self.engine=engine;self.process=process;self.cost=cost
    self.now=0.;self.inputs=deque();self.outputs=deque();self.busy=None
    self.flags=[1,1,1];self.dropped_outputs=0;self.expired=0

  def advance(self,until):
    while True:
      if self.busy is not None:
        finish,result=self.busy
        if finish>until:break
        self.now=finish;self.busy=None
        if result is not None:
          if len(self.outputs)>=4:self.outputs.popleft();self.dropped_outputs+=1
          self.outputs.append(result)
      if not self.inputs:break
      frame=self.inputs.popleft()
      self.now=max(self.now,frame.now)
      result=self.process(self.engine,frame,self.flags,clock=lambda:self.now)
      if result is None:self.expired+=1
      self.busy=(self.now+max(0,self.cost(frame)),result)
    self.now=max(self.now,until)

  def exchange(self,frame):
    self.advance(frame.now)
    self.flags[1]=int(frame.learn);self.flags[2]=frame.observation_fence
    if len(self.inputs)>=64:return None,True
    self.inputs.append(frame)
    newest=None
    while self.outputs:newest=self.outputs.popleft()
    self.advance(frame.now)
    return newest,False
