"""The pair partner process: its job loop and modeld's handle on it (no GPU or camera needed)."""
import os
from multiprocessing.connection import Connection
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.starpilot.models import pair_partner
from openpilot.starpilot.models.pair_partner import FrameSource, PartnerError, RemotePartner, serve


class Client:
  """A camerad stream: frame ids arrive in order."""

  def __init__(self, ids):
    self.ids, self.frame_id = list(ids), -1

  def recv(self, timeout_ms=100):
    if not self.ids:
      return None
    self.frame_id = self.ids.pop(0)
    return SimpleNamespace(frame=self.frame_id)


def test_frame_source_reads_up_to_the_frame_modeld_chose():
  source = FrameSource({"main": Client([5, 6, 7, 8])})
  assert source.frame("main", 7).frame == 7
  assert source.frame("main", 7).frame == 7          # already read: kept
  assert source.frame("main", 6) is None             # gone past it
  assert source.frame("main", 9, timeout_s=0.01) is None


class Conn:
  def __init__(self, messages):
    self.inbox, self.sent = list(messages), []

  def recv(self):
    if not self.inbox:
      raise EOFError
    return self.inbox.pop(0)

  def poll(self, timeout=0):
    return bool(self.inbox)

  def send(self, message):
    self.sent.append(message)


class Model:
  vision_input_names = ("img", "big_img")

  def __init__(self, fail=False):
    self.runs, self.fail = [], fail

  def run(self, bufs, transforms, inputs):
    if self.fail:
      raise RuntimeError("gpu fault")
    self.runs.append((bufs["img"].frame, bufs["big_img"].frame, inputs["x"]))
    return {"plan": np.ones(3)}


def test_serve_runs_the_newest_job_and_reports_what_it_skipped():
  conn = Conn([("job", 1, (5, 5), {}, {"x": 1}), ("job", 2, (6, 6), {}, {"x": 2}), ("job", 3, (9, 9), {}, {"x": 3})])
  model = Model()
  frames = FrameSource({"main": Client([5, 6, 7]), "extra": Client([5, 6, 7])})
  assert serve(conn, model, frames, extra=True) == 0
  assert model.runs == []                             # job 3 is the newest; its frame 9 never came
  assert [m[:2] for m in conn.sent] == [("skip", 1), ("skip", 2), ("skip", 3)]
  assert conn.sent[-1][2] == "camera frame not available"

  conn = Conn([("job", 1, (6, 6), {}, {"x": 1})])
  model = Model()
  serve(conn, model, FrameSource({"main": Client([5, 6]), "extra": Client([5, 6])}), extra=True)
  assert model.runs == [(6, 6, 1)] and conn.sent[0][0] == "out" and conn.sent[0][1] == 1

  conn = Conn([("job", 1, (5, 5), {}, {"x": 1})])
  assert serve(conn, Model(fail=True), FrameSource({"main": Client([5])}), extra=False) == 1
  assert conn.sent[0][0] == "error" and "gpu fault" in conn.sent[0][2]


class Proc:
  def __init__(self):
    self.killed, self.waited = False, 0

  def wait(self, timeout=None):
    self.waited += 1

  def kill(self):
    self.killed = True


def remote():
  child = {}

  def popen(command, pass_fds, close_fds, env):
    child["command"], child["env"] = command, env
    child["conn"] = Connection(os.dup(pass_fds[0]))
    return Proc()

  partner = RemotePartner("polaris", "v15", "/data/models/p.pkl", "a" * 64, 1344, 760, main_wide_camera=False,
                          use_extra_client=True, popen=popen)
  return partner, child


def test_remote_partner_start_jobs_and_late_results():
  partner, child = remote()
  command = child["command"]
  assert command[1:3] == ["-m", "openpilot.starpilot.models.pair_partner"] and "--extra" in command and "--main-wide" not in command
  assert command[command.index("--core") + 1] == str(pair_partner.PARTNER_CORE) == "5"
  assert command[command.index("--priority") + 1] == "50"
  # the partner's comma-GPU context stays below modeld's, whatever modeld's own environment says
  assert child["env"]["QCOM_PRIORITY"] == str(pair_partner.PARTNER_GPU_PRIORITY) == "12"
  assert pair_partner.PRIMARY_GPU_PRIORITY < pair_partner.PARTNER_GPU_PRIORITY
  child["conn"].send(("ready", {"loadS": 3.6}))
  assert partner.wait_ready(1.0)["loadS"] == 3.6

  seq = partner.submit((7, 7), {"img": np.eye(3)}, {"x": np.zeros(2)})
  job = child["conn"].recv()
  assert job[0] == "job" and job[1] == seq and job[2] == (7, 7)
  assert partner.collect(seq, 0.01) == (None, None, "late")
  # the late answer arrives during the next frame and is dropped; the next frame's answer is used
  child["conn"].send(("out", seq, {"plan": 1}, 40.0))
  seq2 = partner.submit((8, 8), {}, {})
  child["conn"].send(("out", seq2, {"plan": 2}, 37.0))
  assert partner.collect(seq2, 1.0) == ({"plan": 2}, 37.0, "")
  seq3 = partner.submit((9, 9), {}, {})
  child["conn"].send(("skip", seq3, "camera frame not available"))
  assert partner.collect(seq3, 1.0) == (None, None, "camera frame not available")
  child["conn"].send(("error", seq3 + 1, "gpu fault"))
  with pytest.raises(PartnerError, match="gpu fault"):
    partner.collect(seq3 + 1, 1.0)
  partner.close()
  received = []
  try:
    while child["conn"].poll(0.5):
      received.append(child["conn"].recv())
  except EOFError:   # modeld's end closed after "stop"
    pass
  assert [m[1] for m in received if m[0] == "job"] == [seq2, seq3] and received[-1] == ("stop",)


def test_remote_partner_that_fails_to_start_or_dies():
  partner, child = remote()
  child["conn"].send(("error", 0, "camera stream main unavailable"))
  with pytest.raises(PartnerError, match="camera stream main unavailable"):
    partner.wait_ready(1.0)
  partner, child = remote()
  with pytest.raises(PartnerError, match="did not start"):
    partner.wait_ready(0.01)
  partner, child = remote()
  child["conn"].close()
  with pytest.raises(PartnerError, match="ended"):
    partner.collect(1, 0.5)
