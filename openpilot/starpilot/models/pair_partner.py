"""A Model Laboratory pair partner in its own process on the comma's GPU, beside a big model on the Chestnut.

Inside modeld's one thread a comma-GPU partner can only run after the big model's USB work, which does not fit the
50 ms frame. In its own process it runs in parallel: modeld sends each frame's job (camera frame ids, warp
transforms, model inputs) at the start of the frame; the partner reads the same camera frames from camerad, runs,
and sends its outputs back. On the comma GPU, modeld's work (the big model's camera warp) runs at a higher priority
than the partner's, so the GPU interrupts the partner for it and the big model never waits.

Measured on the comma (5 Nov 2026, real stack, forced onroad, car off, RDF V4 + lightspeed): modeld 40.0 ms per
frame (max 44.7) against 38.6 ms for lightspeed alone, the partner's outputs ready 39.4 ms into the frame, no late
or dropped frames; UI (20 fps) and driver monitoring (+0.9 ms) unchanged. With both at GPU priority 12 and the job
sent after the warp instead, the outputs landed 44.5-46.3 ms into the frame.

The partner runs on a big core at a real-time priority below every control process (card, selfdrived, plannerd,
radard), so they always preempt it, and never on the isolated cores 6 (camerad) and 7 (modeld).

  modeld side:   RemotePartner(...).wait_ready(); submit(); collect(); close()
  partner side:  python -m openpilot.starpilot.models.pair_partner --fd N ...   (started by RemotePartner)
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
from multiprocessing.connection import Connection
from pathlib import Path

# Big cores 4 and 5, whichever is free (camerad keeps 6, modeld 7). With the car on, selfdrived alone takes ~28 % of
# core 5 and card shares core 4: RDF V4 on core 5 alone took 50 ms, on core 4 alone 42 ms (5 Nov 2026, parked, car on).
PARTNER_CORES = (4, 5)
PARTNER_RT_PRIORITY = 50    # below plannerd/radard (51) and card/selfdrived (53): they always run first
WARMUP_RUNS = 20            # dummy frames before the first real one: GPU clocks and caches up, first-run costs paid
DIAG_GPU_EVERY = 20         # frames between reads of the GPU clock for the diagnostics
START_TIMEOUT_S = 60.0
# Comma GPU (KGSL) context priorities: lower is higher, and with preemption on the GPU interrupts a lower ring (of
# four) for a higher one. launch_env.sh puts every model at 12 so the UI (8) interrupts them. With a partner process,
# modeld's context goes to 11 (the UI's ring, so it never interrupts the UI) and the partner stays at 12.
PRIMARY_GPU_PRIORITY = 11
PARTNER_GPU_PRIORITY = 12


class PartnerError(RuntimeError):
  pass


class PartnerDriver:
  """The pair's partner process as modeld's spare driving model, so the comma holds one copy of the small model
  instead of two (memory: a real drive ran the comma to 94 % with both). If the Chestnut fails, modeld drives on
  with it: the partner computes complete outputs every frame."""

  chestnut = False
  vision_input_names = ("img", "big_img")
  WAIT_S = 0.1

  def __init__(self, partner: "RemotePartner", desire_len: int):
    import numpy as np
    self.partner = partner
    self.model_id = partner.model_id
    self.npy = {"desire": np.zeros(desire_len, dtype=np.float32)}   # modeld reads the desire width
    self.lat_delay = 0.0
    self.frame_drop_ratio = 0.0
    self.frame_ids: tuple[int, int] | None = None                   # modeld sets them each frame
    self._last: dict | None = None

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    if self.frame_ids is None:
      raise PartnerError("camera frame ids unavailable")
    seq = self.partner.submit(self.frame_ids, transforms, inputs)
    if after_enqueue is not None:
      after_enqueue()
    outputs, _, why = self.partner.collect(seq, self.WAIT_S)
    if outputs is None:
      # one missed frame reuses the previous outputs; a second in a row is a failure modeld handles
      if self._last is None:
        raise PartnerError(f"partner process {why}")
      outputs, self._last = self._last, None
      return outputs
    self._last = outputs
    return outputs

  def get_action_from_model(self, outputs, previous, lat_action_t, long_action_t, v_ego):
    return self.partner.get_action_from_model(outputs, previous, lat_action_t, long_action_t, v_ego)


class RemotePartner:
  """modeld's handle on the partner process. Quacks enough like a model for ModelPair (model_id, chestnut,
  get_action_from_model); `remote` tells the pair to send jobs instead of calling run()."""

  chestnut = False
  remote = True
  vision_input_names = ("img", "big_img")

  def __init__(self, model_id: str, behavior_version: str, artifact: Path, sha256: str, cam_w: int, cam_h: int, *,
               main_wide_camera: bool, use_extra_client: bool, cores: tuple[int, ...] = PARTNER_CORES, priority: int = PARTNER_RT_PRIORITY,
               gpu_priority: int = PARTNER_GPU_PRIORITY, popen=subprocess.Popen):
    self.model_id, self.behavior_version = model_id, behavior_version
    self.lat_delay = 0.0
    self.frame_drop_ratio = 0.0
    self.seq = 0
    self.load_s: float | None = None
    self.info: dict = {}
    self.diag: dict = {}
    parent, child = socket.socketpair()
    self.conn = Connection(parent.detach())
    command = [sys.executable, "-m", "openpilot.starpilot.models.pair_partner", "--fd", str(child.fileno()),
               "--parent", str(os.getpid()), "--model-id", model_id, "--version", behavior_version, "--artifact", str(artifact),
               "--sha256", sha256, "--camera", f"{cam_w}x{cam_h}", "--cores", ",".join(str(c) for c in cores),
               "--priority", str(priority)]
    if main_wide_camera:
      command.append("--main-wide")
    if use_extra_client:
      command.append("--extra")
    try:
      # modeld's own GPU priority (PRIMARY_GPU_PRIORITY) must not pass on to the partner
      env = {**os.environ, "QCOM_PRIORITY": str(gpu_priority)}
      self.proc = popen(command, pass_fds=(child.fileno(),), close_fds=True, env=env)
    finally:
      child.close()

  def wait_ready(self, timeout_s: float = START_TIMEOUT_S) -> dict:
    if not self.conn.poll(timeout_s):
      self.close()
      raise PartnerError(f"partner process did not start within {timeout_s:.0f} s")
    message = self.conn.recv()
    if message[0] != "ready":
      self.close()
      raise PartnerError(f"partner process failed to start: {message[-1]}")
    self.load_s = message[1].get("loadS")
    self.info = message[1]
    return message[1]

  def submit(self, frame_ids: tuple[int, int], transforms: dict, inputs: dict) -> int:
    """Queue this frame for the partner; returns the job number collect() waits for."""
    self.seq += 1
    self.conn.send(("job", self.seq, tuple(int(f) for f in frame_ids), transforms, inputs))
    return self.seq

  def collect(self, seq: int, timeout_s: float) -> tuple[dict | None, float | None, str]:
    """(outputs, partner run ms, "") for job `seq`, or (None, None, why) when it did not arrive in time or was
    skipped. Older results that arrive late are dropped. Raises PartnerError when the partner failed. The partner's
    timing breakdown for that frame is left in self.diag (or {})."""
    deadline = time.monotonic() + max(0.0, timeout_s)
    while True:
      remaining = deadline - time.monotonic()
      try:
        if not self.conn.poll(max(0.0, remaining)):
          return None, None, "late"
        message = self.conn.recv()
      except (EOFError, OSError) as error:
        raise PartnerError(f"partner process ended: {error or 'connection closed'}") from None
      kind = message[0]
      if kind == "error":
        raise PartnerError(message[-1])
      if message[1] < seq:
        continue
      if kind == "out":
        self.diag = message[4] if len(message) > 4 else {}
        return message[2], message[3], ""
      return None, None, message[2] if kind == "skip" else f"unexpected partner message {kind}"

  def get_action_from_model(self, outputs, previous, lat_action_t, long_action_t, v_ego):
    from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS
    from openpilot.starpilot.models.runner import action_from_outputs
    return action_from_outputs(outputs, self.behavior_version, previous, lat_action_t, long_action_t, v_ego,
                               LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS)

  def close(self) -> None:
    try:
      self.conn.send(("stop",))
    except (OSError, ValueError):
      pass
    try:
      self.proc.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
      self.proc.kill()
      self.proc.wait(timeout=1.0)
    except Exception:  # noqa: BLE001 - closing never fails the drive
      pass
    try:
      self.conn.close()
    except OSError:
      pass


# ---- the partner process -----------------------------------------------------------------------------------------

def _check_parent(parent: int) -> None:
  # No PR_SET_PDEATHSIG: it fires when the *thread* that started us ends, and modeld starts the partner from its
  # loader thread. modeld going away closes the socket instead, which ends serve() (EOF).
  if os.getppid() != parent:
    sys.exit(0)


def _parse(argv=None):
  parser = argparse.ArgumentParser()
  parser.add_argument("--fd", type=int, required=True)
  parser.add_argument("--parent", type=int, required=True)
  parser.add_argument("--model-id", required=True)
  parser.add_argument("--version", required=True)
  parser.add_argument("--artifact", type=Path, required=True)
  parser.add_argument("--sha256", required=True)
  parser.add_argument("--camera", required=True)
  parser.add_argument("--cores", default=",".join(str(c) for c in PARTNER_CORES))
  parser.add_argument("--priority", type=int, default=PARTNER_RT_PRIORITY)
  parser.add_argument("--main-wide", action="store_true")
  parser.add_argument("--extra", action="store_true")
  return parser.parse_args(argv)


class FrameSource:
  """The partner's own camerad clients, read up to the frames modeld chose."""

  def __init__(self, clients: dict):
    self.clients = clients
    self.latest = {name: (-1, None) for name in clients}

  def frame(self, name: str, wanted: int, timeout_s: float = 0.03):
    client = self.clients[name]
    frame_id, buf = self.latest[name]
    deadline = time.monotonic() + timeout_s
    while frame_id < wanted and time.monotonic() < deadline:
      received = client.recv(10)
      if received is not None:
        frame_id, buf = client.frame_id, received
    self.latest[name] = (frame_id, buf)
    return buf if frame_id == wanted else None


def _release_load_memory() -> None:
  """Hand the model load's temporary buffers (the file read, the unpickled arena) back to the system: the comma has
  little memory to spare during a drive."""
  import ctypes
  import gc
  gc.collect()
  try:
    ctypes.CDLL("libc.so.6").malloc_trim(0)
  except (OSError, AttributeError):
    pass


def _rss_mb() -> float | None:
  try:
    with open("/proc/self/status") as status:
      return next(round(int(line.split()[1]) / 1024, 1) for line in status if line.startswith("VmRSS:"))
  except (OSError, StopIteration, ValueError):
    return None


def _preemptions() -> int:
  """This process's involuntary context switches so far: how often something else took its CPU."""
  try:
    with open("/proc/self/status") as status:
      return next(int(line.split()[1]) for line in status if line.startswith("nonvoluntary_ctxt_switches"))
  except (OSError, StopIteration, ValueError):
    return 0


def _gpu_mhz() -> float | None:
  try:
    return round(int(Path("/sys/class/kgsl/kgsl-3d0/gpuclk").read_text()) / 1e6, 1)
  except (OSError, ValueError):
    return None


def warm_up(model, runs: int = WARMUP_RUNS) -> None:
  """Run dummy frames through the model, then clear its recurrent state, so the first real frames are not slow."""
  import numpy as np
  if runs <= 0:
    return
  frames = {name: np.zeros(model.frame_buf_size, dtype=np.uint8) for name in model.vision_input_names}
  transforms = dict.fromkeys(model.vision_input_names, np.eye(3, dtype=np.float32))
  inputs = {"desire_pulse": np.zeros(len(model.npy["desire"]), dtype=np.float32)}
  for _ in range(runs):
    model.run(frames, transforms, inputs)
  model._reset_state()


def serve(conn, model, frames: FrameSource, extra: bool) -> int:
  """Run jobs until modeld says stop or goes away. Only the newest waiting job runs; older ones are skipped.
  Each answer carries where the frame's time went (diag), so a short real-drive run is enough to see why it is slow."""
  count = 0
  gpu_mhz = _gpu_mhz()
  parse_ms = [0.0]
  parse = getattr(model, "_parse", None)
  if parse is not None:
    def timed_parse(outputs):   # decoding the outputs on the CPU, timed apart from the GPU wait
      started = time.perf_counter()
      try:
        return parse(outputs)
      finally:
        parse_ms[0] = (time.perf_counter() - started) * 1e3
    model._parse = timed_parse
  while True:
    try:
      message = conn.recv()
      while message[0] == "job" and conn.poll(0):
        newer = conn.recv()
        conn.send(("skip", message[1], "partner behind"))
        message = newer
    except (EOFError, OSError):
      return 0
    if message[0] != "job":
      return 0
    _, seq, (main_id, extra_id), transforms, inputs = message
    received = time.perf_counter()
    preempted = _preemptions()
    buf_main = frames.frame("main", main_id)
    buf_extra = frames.frame("extra", extra_id) if extra else buf_main
    got_frames = time.perf_counter()
    if buf_main is None or buf_extra is None:
      conn.send(("skip", seq, "camera frame not available"))
      continue
    bufs = {name: buf_extra if "big" in name else buf_main for name in model.vision_input_names}
    start = time.perf_counter()
    sent = []
    try:
      outputs = model.run(bufs, transforms, inputs, lambda: sent.append(time.perf_counter()))
    except Exception as error:  # noqa: BLE001 - reported to modeld, which stops the partner
      conn.send(("error", seq, f"{type(error).__name__}: {error}"[:300]))
      return 1
    done = time.perf_counter()
    count += 1
    if count % DIAG_GPU_EVERY == 0:
      gpu_mhz = _gpu_mhz()
    sent_at = sent[0] if sent else start
    diag = {"framesMs": round((got_frames - received) * 1e3, 2), "sendMs": round((sent_at - start) * 1e3, 2),
            "waitMs": round((done - sent_at) * 1e3, 2), "parseMs": round(parse_ms[0], 2),
            "preempted": _preemptions() - preempted, "gpuMhz": gpu_mhz}
    conn.send(("out", seq, outputs, round((done - start) * 1e3, 2), diag))


def main(argv=None) -> int:
  args = _parse(argv)
  conn = Connection(args.fd)
  try:
    _check_parent(args.parent)
    cores = [int(core) for core in args.cores.split(",")]
    os.sched_setaffinity(0, set(cores))   # off modeld's core 7 while loading too
    start = time.monotonic()
    os.environ["GMMU"] = "0"
    os.environ.setdefault("AM_POWER_LIMIT", "100")
    from openpilot.starpilot.models.runtime_bridge.runtime import prepare
    prepare("driving")
    from msgq.visionipc import VisionIpcClient
    from openpilot.cereal.visionipc import VisionStreamType   # where modeld imports it from
    from openpilot.common.realtime import config_realtime_process
    from openpilot.starpilot.models.runner import load_verified_model
    cam_w, cam_h = (int(value) for value in args.camera.split("x"))
    model, _ = load_verified_model(cam_w, cam_h, args.artifact, args.version, False, args.model_id, args.sha256)
    main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if args.main_wide else VisionStreamType.VISION_STREAM_NARROW_ROAD
    clients = {"main": VisionIpcClient("camerad", main_stream, False)}
    if args.extra:
      clients["extra"] = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
    deadline = time.monotonic() + 10.0
    for name, client in clients.items():
      while not client.connect(False):
        if time.monotonic() > deadline:
          raise PartnerError(f"camera stream {name} unavailable")
        time.sleep(0.05)
    config_realtime_process(cores, args.priority)   # also turns the garbage collector off for the run
    warm_up(model)
    _release_load_memory()
    conn.send(("ready", {"pid": os.getpid(), "loadS": round(time.monotonic() - start, 1), "cores": cores, "priority": args.priority,
                         "desireLen": int(model.npy["desire"].shape[-1]), "rssMb": _rss_mb()}))
  except Exception as error:  # noqa: BLE001 - reported to modeld, which drives without the pair
    try:
      conn.send(("error", 0, f"{type(error).__name__}: {error}"[:300]))
    except OSError:
      pass
    return 1
  return serve(conn, model, FrameSource(clients), args.extra)


if __name__ == "__main__":
  sys.exit(main())
