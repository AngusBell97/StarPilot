"""Model Laboratory pairs in modeld: steering from one model and speed from the other.

Pairs are two small models, or one big and one small (two big ones do not fit one Chestnut's 50 ms frame). With a
pair switched on, modeld loads the pair's models instead of the Active ones (`pair_selection`): the big one on the
Chestnut as the running model, else the lateral small one; the other model is the partner. With the big model on
the Chestnut, the small partner runs on the comma's GPU in its own process (pair_partner.py), in parallel with the
big model: modeld sends it each frame's job at the start of the frame and collects its outputs by a deadline (a late
frame reuses the partner's previous outputs once). The comma GPU interrupts the partner for the big model's camera
warp, because modeld's GPU work runs at a higher priority than the partner's (pair_partner.PRIMARY_GPU_PRIORITY). Everything on the Chestnut does not fit: the
Chestnut needs ~13-15 ms to compute even a small model (measured 5 Nov 2026). Other placements (small+small) run the
partner in modeld's thread as before. Mode "drive" (default) composes the outputs; "shadow" keeps a log-only test
path that Galaxy no longer offers.

A timing guard stops the partner for the rest of the session when it threatens the 20 Hz frame budget, and any
partner error stops it too; the primary then carries on alone. A primary error is modeld's to handle (it falls
back to the small model as before).
"""
from __future__ import annotations

import json
import os
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openpilot.cereal import log
from openpilot.starpilot.models.laboratory_outputs import compose_model_outputs, hybrid_action_values

MODES = ("shadow", "drive")
STATUS_PATH = Path(os.getenv("STARPILOT_MODEL_PAIR_STATUS", "/dev/shm/starpilot_model_pair.json"))
# the last run's summary survives the comma powering down after a drive
LAST_RUN_PATH = Path(os.getenv("STARPILOT_MODEL_PAIR_LAST_RUN", "/data/models/v26/laboratory_last_run.json"))
PERSIST_PERIOD_S = 30.0
STATUS_SCHEMA = 1

# Guard: the frame is 50 ms and lightspeed alone takes 41-43 ms (p99) on the road, with ~7 ms spare.
SLOW_FRAME_S = 0.047        # a pair frame slower than this risks the next camera frame
SLOW_WINDOW = 40            # frames (2 s)
SLOW_LIMIT = 4              # slow frames allowed in the window before the partner stops
DROP_LIMIT = 0.02           # modeld's filtered frame-drop ratio that stops the partner
WARMUP_FRAMES = 10          # frames before the guard judges (modeld also ignores its first 10 frames' drops)
PARTNER_DEADLINE_S = 0.045  # a partner process's outputs must arrive within this of the pair frame's start
# A partner process starts by probing: it runs every frame but the primary drives and modeld never waits for it.
# It takes over (its role) only after PROBE_FRAMES answers in a row each ready within PROBE_READY_S, and only while
# the car is not steering itself. If it then runs late it goes back to probing (at once), at most MAX_DEMOTIONS times.
PROBE_FRAMES = 40
PROBE_READY_S = 0.042
MAX_DEMOTIONS = 3
PROBE_LIMIT_FRAMES = 20 * 60 * 5   # a partner never on time in 5 minutes of probing stops (its timing stays recorded)
STATUS_PERIOD_S = 1.0
LOG_PERIOD_S = 10.0


@dataclass(frozen=True)
class PairPlan:
  mode: str
  lateral_id: str
  longitudinal_id: str
  partner_id: str
  partner_role: str          # "lateral" or "longitudinal": what the partner decides in drive mode


def pair_kind_error(lateral_big: bool, longitudinal_big: bool) -> str:
  """Pairs are two small models, or one big and one small; two big models do not fit one Chestnut's frame."""
  return "Choose two small models, or one big and one small" if lateral_big and longitudinal_big else ""


def pair_selection(config: dict, entries: dict) -> dict | None:
  """The models modeld loads for an enabled pair, as Active-model overrides: big+small loads the big one (Chestnut)
  and the small one; small+small loads the lateral one as the running model. None for single-model driving."""
  if not isinstance(config, dict) or config.get("enabled") is not True:
    return None
  lateral, longitudinal = config.get("lateralModel"), config.get("longitudinalModel")
  if lateral not in entries or longitudinal not in entries or lateral == longitudinal:
    return None
  big = [mid for mid in (lateral, longitudinal) if entries[mid].get("uses_external_gpu")]
  small = [mid for mid in (lateral, longitudinal) if not entries[mid].get("uses_external_gpu")]
  if len(big) > 1:
    return None
  return {"big": big[0] if big else "", "small": small[0]}


def plan_pair(config: dict, primary_id: str) -> tuple[PairPlan | None, str]:
  """Which partner to load for this configuration and running model; (None, reason) when no pair runs."""
  if not isinstance(config, dict) or config.get("enabled") is not True:
    return None, "off"
  mode = config.get("mode", "drive")
  lateral, longitudinal = config.get("lateralModel"), config.get("longitudinalModel")
  if mode not in MODES or not isinstance(lateral, str) or not isinstance(longitudinal, str) or not lateral or not longitudinal:
    return None, "The model pair setting is invalid"
  if lateral == longitudinal:
    return None, "Choose two different models"
  if primary_id not in (lateral, longitudinal):
    return None, f"The pair must include the model that is running ({primary_id})"
  partner = longitudinal if lateral == primary_id else lateral
  return PairPlan(mode, lateral, longitudinal, partner, "lateral" if partner == lateral else "longitudinal"), ""


LOCAL_VARIANTS = "local_variants.json"
LOCAL_BUILDS = "local_builds"   # comma-GPU builds made on this comma (kept out of the catalog's model folders)


def local_build_path(model_id: str, root: Path, kind: str) -> Path:
  """Where a build made on this comma lives: a Chestnut ("amd") build beside the catalog model, a tuned comma-GPU
  ("tuned", BEAM-searched kernels) build under <root>/local_builds."""
  if kind == "amd":
    from openpilot.starpilot.models.manager import artifact_path
    return artifact_path(model_id, root, "amd")
  return root / LOCAL_BUILDS / f"{model_id}_driving_{kind}_tinygrad.pkl"


def local_variant_ids(root: Path) -> set[str]:
  """Models with a Chestnut build made on this comma."""
  try:
    return {mid for mid, row in json.loads((root / LOCAL_VARIANTS).read_text()).items() if isinstance(row, dict) and "amd" in row}
  except (OSError, ValueError, AttributeError):
    return set()


def local_variant(model_id: str, root: Path, *, verify: bool = True, kind: str = "amd") -> dict | None:
  """A build of a small model made on this comma (not published in the catalog), a Chestnut one ("amd") or a tuned
  comma-GPU one ("tuned"): {"artifact_sha256", "artifact_size", "behavior_version", ...} from
  <root>/local_variants.json when its file is present (and, with verify, its digest matches)."""
  import hashlib
  try:
    entry = json.loads((root / LOCAL_VARIANTS).read_text())[model_id][kind]
    path = local_build_path(model_id, root, kind)
    if path.is_symlink() or not path.is_file() or path.stat().st_size != entry["artifact_size"]:
      return None
    if verify:
      with path.open("rb") as stream:
        if hashlib.file_digest(stream, "sha256").hexdigest() != entry["artifact_sha256"]:
          return None
    return {**entry, "path": path}
  except (OSError, ValueError, KeyError, TypeError):
    return None


def partner_artifact(partner_id: str, *, root: Path | None = None, chestnut_ok: bool = True) -> tuple[Path, str, str, bool]:
  """(path, behavior version, sha256, on_chestnut) for the partner; raises when it is not installed and verified."""
  from openpilot.starpilot.models import manager
  from openpilot.starpilot.models.catalog import DEFAULT_SMALL, DEFAULT_SMALL_SHA256
  base = manager.ROOT if root is None else root
  entries = manager.catalog(base)
  row = entries.get(partner_id)
  if row is None:
    raise ValueError(f"Unknown partner model {partner_id}")
  if row.get("uses_external_gpu"):
    path = manager.verified_artifact(partner_id, entries, base)
    if path is None:
      raise ValueError(f"{row.get('name', partner_id)} is not downloaded")
    return path, row["version"], row["artifact_sha256"], True
  amd = manager.verified_artifact(partner_id, entries, base, "amd") if chestnut_ok else None
  if amd is not None:
    return amd, row["version"], manager.artifact_entry(partner_id, entries, "amd")["artifact_sha256"], True
  local = local_variant(partner_id, base) if chestnut_ok else None
  if local is not None and local.get("behavior_version") == row["version"]:
    return local["path"], row["version"], local["artifact_sha256"], True
  # on the comma GPU, a tuned build made on this comma wins once registered (it is registered only after a parked
  # check shows it faster with outputs matching the catalog build closely)
  tuned = local_variant(partner_id, base, kind="tuned")
  if tuned is not None and tuned.get("behavior_version") == row["version"]:
    return tuned["path"], row["version"], tuned["artifact_sha256"], False
  path = manager.verified_artifact(partner_id, entries, base)
  if path is not None:
    return path, row["version"], row["artifact_sha256"], False
  if partner_id == DEFAULT_SMALL and (shipped := manager.shipped_default()) is not None:
    return shipped, row["version"], DEFAULT_SMALL_SHA256, False
  raise ValueError(f"{row.get('name', partner_id)} is not downloaded")


class PairGuard:
  """Decides when the partner must stop to protect the frame budget."""

  def __init__(self):
    self.frames = 0
    self.slow = deque(maxlen=SLOW_WINDOW)

  def check(self, run_s: float, frame_drop_ratio: float, late: bool = False) -> str:
    self.frames += 1
    # warm-up frames take hundreds of ms while the GPUs clock up; keep them out of the slow window
    if self.frames <= WARMUP_FRAMES:
      return ""
    self.slow.append(run_s > SLOW_FRAME_S or late)
    if frame_drop_ratio > DROP_LIMIT:
      return f"frames dropped ({frame_drop_ratio:.0%}) while the pair ran"
    if sum(self.slow) > SLOW_LIMIT:
      return f"pair too slow: {sum(self.slow)} of the last {len(self.slow)} frames over {SLOW_FRAME_S * 1e3:.0f} ms or late"
    return ""


def _percentile(values, q: float) -> float | None:
  return round(float(np.percentile(np.asarray(values), q)), 2) if len(values) else None


class ModelPair:
  """Drop-in for the driving model in modeld's loop (run, get_action_from_model, chestnut, npy, ...)."""

  def __init__(self, primary, partner, plan: PairPlan, *, now: Callable[[], float] = time.perf_counter,
               status_path: Path | None = None, report: Callable[[str, dict], None] | None = None, keep_partner: bool = False):
    self.primary = primary
    self.partner = partner
    self.plan = plan
    self.keep_partner = keep_partner   # the partner process is modeld's spare too: stopping the pair leaves it running
    self.now = now
    self.status_path = STATUS_PATH if status_path is None else status_path
    self.report = report
    self.guard = PairGuard()
    self.partner_active = True
    self.stop_reason = ""
    self.partner_device = "chestnut" if partner.chestnut else "comma"
    # a partner in its own process (pair_partner.RemotePartner) gets jobs instead of run() calls
    self.remote = bool(getattr(partner, "remote", False))
    self.frame_ids: tuple[int, int] | None = None   # modeld sets the camera frame ids before each run
    self.partner_image = "own process" if self.remote else ""
    self._late: deque[bool] = deque(maxlen=600)
    self._sent_ms: deque[float] = deque(maxlen=600)    # job sent, ms after the pair frame started
    self._ready_ms: deque[float] = deque(maxlen=600)   # partner outputs in hand, ms after the pair frame started
    # the partner process's own breakdown of each frame (pair_partner.serve diag)
    self._diag: dict[str, deque] = {key: deque(maxlen=600) for key in ("framesMs", "sendMs", "waitMs", "parseMs", "preempted",
                                                                        "gpuBusyPct")}
    self._gpu_mhz: float | None = None
    self._reused = False
    self.probing = self.remote          # see PROBE_FRAMES
    self.lat_active = False             # modeld sets it each frame: no handover while the car steers itself
    self.demotions = 0
    self._probe: deque[bool] = deque(maxlen=PROBE_FRAMES)
    self._probe_ready_ms: deque[float] = deque(maxlen=600)
    self._pending: dict[int, float] = {}   # probing: job number -> when its frame started
    self._probe_frames = 0
    self._demote_now = ""
    self.share_warp = bool(partner.chestnut and primary.chestnut and getattr(primary, "camera_warp_descriptor", None) is not None and
                           primary.camera_warp_descriptor == getattr(partner, "camera_warp_descriptor", None))
    self.lat_delay = 0.0
    self.frame_drop_ratio = 0.0
    self._outputs: dict[str, dict | None] = {"primary": None, "partner": None}
    self._run_ms = deque(maxlen=600)
    self._partner_ms = deque(maxlen=600)
    self._lat_diff = deque(maxlen=600)
    self._long_diff = deque(maxlen=600)
    self._frames = 0
    self._last_status = 0.0
    self._last_log = 0.0
    self._last_persist = 0.0
    self.last_run_path = LAST_RUN_PATH if status_path is None else status_path.with_name("last_run.json")

  # modeld reads these from the driving model
  vision_input_names = ("img", "big_img")

  @property
  def chestnut(self) -> bool:
    return self.primary.chestnut

  @property
  def model_id(self) -> str:
    return self.primary.model_id

  @property
  def npy(self):
    return self.primary.npy

  @property
  def driving(self) -> bool:
    return self.partner_active and self.plan.mode == "drive" and not self.probing

  def __getattr__(self, name):
    # anything else modeld or a hook reads (input_queues, last_warp, ...) is the primary's
    primary = self.__dict__.get("primary")
    if primary is None:
      raise AttributeError(name)
    return getattr(primary, name)

  def _stop_partner(self, reason: str) -> None:
    if not self.partner_active:
      return
    self.partner_active = False
    self.stop_reason = reason
    self._outputs["partner"] = None
    # a partner process that is also modeld's spare driving model stays up (idle) for a Chestnut failure
    if self.remote and not self.keep_partner:
      try:
        self.partner.close()
      except Exception:  # noqa: BLE001 - closing never fails the drive
        pass
    if self.report is not None:
      self.report("modelPairStopped", {"reason": reason, **self.summary()})
    self.write_status(force=True)

  def _primary_image(self):
    """The camera image the primary uploaded to the Chestnut this frame, when the partner can read it directly."""
    if not (self.share_warp and self.partner.chestnut and self.primary.chestnut):
      return None
    image = getattr(self.primary, "input_queues", {}).get("mapped_gpu_image")
    accepts = getattr(self.partner, "accepts_device_image", None)
    return image if accepts is not None and accepts(image) else None

  def _run_partner(self, bufs, transforms, inputs) -> None:
    t = self.now()
    try:
      if (image := self._primary_image()) is not None:
        self.partner_image = "primary's Chestnut upload"
        self._outputs["partner"] = self.partner.run_on_device_image(image, inputs)
      elif self.share_warp:
        self.partner_image = "shared warp"
        self._outputs["partner"] = self.partner.run(bufs, transforms, inputs, shared_warp=self.primary.last_warp)
      else:
        self.partner_image = "own warp"
        self._outputs["partner"] = self.partner.run(bufs, transforms, inputs)
    except Exception as error:  # noqa: BLE001 - any partner failure stops the partner, never the drive
      self._stop_partner(f"partner failed: {type(error).__name__}: {error}"[:300])
      return
    self._partner_ms.append((self.now() - t) * 1e3)

  def _run_remote(self, bufs, transforms, inputs, after_enqueue) -> tuple[dict, dict | None, bool]:
    """Primary on the Chestnut, partner in its own process on the comma GPU: (primary outputs, partner outputs, late)."""
    job = {}

    def send_job():
      if self.partner_active and self.frame_ids is not None:
        try:
          job["seq"] = self.partner.submit(self.frame_ids, transforms, inputs)
          self._sent_ms.append((self.now() - start) * 1e3)
        except Exception as error:  # noqa: BLE001
          self._stop_partner(f"partner process unreachable: {error}"[:300])

    start = self.now()
    # at once: the GPU runs the big model's camera warp ahead of the partner (higher priority), so nothing waits
    send_job()
    primary_out = self.primary.run(bufs, transforms, inputs, after_enqueue)
    if not self.partner_active:
      return primary_out, None, False
    if "seq" not in job:
      self._stop_partner("camera frame ids unavailable for the partner process")
      return primary_out, None, False
    if self.probing:
      self._pending[job["seq"]] = start
      self._probe_frames += 1
      self._probe_answers()
      if self.probing and self._probe_frames >= PROBE_LIMIT_FRAMES:
        self._stop_partner(f"never on time in {PROBE_LIMIT_FRAMES // 1200} minutes of probing")
      return primary_out, None, False
    try:
      outputs, partner_ms, _why = self.partner.collect(job["seq"], PARTNER_DEADLINE_S - (self.now() - start))
    except Exception as error:  # noqa: BLE001 - a failed partner stops, the primary drives on
      self._stop_partner(f"partner failed: {error}"[:300])
      return primary_out, None, False
    if outputs is not None:
      diag = getattr(self.partner, "diag", None) or {}
      for key, values in self._diag.items():
        if diag.get(key) is not None:
          values.append(diag[key])
      self._gpu_mhz = diag.get("gpuMhz", self._gpu_mhz)
      self._ready_ms.append((self.now() - start) * 1e3)
      self._partner_ms.append(partner_ms)
      self._late.append(False)
      self._reused = False
      return primary_out, outputs, False
    self._late.append(True)
    # one late frame reuses the partner's previous outputs (50 ms old); two in a row hand the role back to the primary
    # until the partner qualifies again, so the two models never alternate frame by frame
    if self._reused or self._outputs["partner"] is None:
      self._demote_now = "two partner frames late in a row"
      return primary_out, None, True
    self._reused = True
    return primary_out, self._outputs["partner"], True

  def _probe_answers(self) -> None:
    """Probing: score every answer that has come in (ready within PROBE_READY_S of its frame's start, or not) without
    waiting, and hand over once PROBE_FRAMES in a row were on time while the car is not steering itself."""
    try:
      answers = self.partner.drain()
    except Exception as error:  # noqa: BLE001
      self._stop_partner(f"partner failed: {error}"[:300])
      return
    for seq, outputs, diag, _why in answers:
      for missed in [s for s in self._pending if s < seq]:   # never answered: skipped as stale by the partner
        self._pending.pop(missed)
        self._probe.append(False)
      started = self._pending.pop(seq, None)
      if started is None:
        continue
      ready = (diag.get("doneAt", float("inf")) - started) if outputs is not None else float("inf")
      if ready != float("inf"):
        self._probe_ready_ms.append(ready * 1e3)
        for key, values in self._diag.items():
          if diag.get(key) is not None:
            values.append(diag[key])
        self._gpu_mhz = diag.get("gpuMhz", self._gpu_mhz)
      self._probe.append(ready <= PROBE_READY_S)
    if len(self._probe) == PROBE_FRAMES and all(self._probe) and not self.lat_active:
      self.probing = False
      self._pending.clear()
      self.guard = PairGuard()
      if self.report is not None:
        self.report("modelPairDriving", {"demotions": self.demotions, **self.summary()})
      self.write_status(force=True)

  def _demote(self, reason: str) -> None:
    """Driving and late: the primary takes the partner's role back at once, and the partner probes again."""
    self.probing = True
    self.demotions += 1
    self._probe_frames = 0
    self.stop_reason = f"probing again: {reason}"[:300]
    self._outputs["partner"] = None
    self._probe.clear()
    self.guard = PairGuard()
    if self.report is not None:
      self.report("modelPairProbing", {"reason": reason, "demotions": self.demotions, **self.summary()})
    self.write_status(force=True)

  def run(self, bufs, transforms, inputs, after_enqueue: Callable[[], None] | None = None) -> dict:
    for model in (self.primary, self.partner):
      model.lat_delay, model.frame_drop_ratio = self.lat_delay, self.frame_drop_ratio
    if self.remote:
      t = self.now()
      primary_out, partner_out, late = self._run_remote(bufs, transforms, inputs, after_enqueue)
      self._outputs["primary"] = primary_out
      if self.partner_active:
        self._outputs["partner"] = partner_out
      return self._finish(t, primary_out, late)
    t = self.now()
    overlap = self.partner_active and not self.partner.chestnut and self.primary.chestnut
    queued: list = []   # a Chestnut partner's work queued right behind the primary's on the Chestnut

    def enqueued():
      if after_enqueue is not None:
        after_enqueue()
      if overlap and self.partner_active:
        self._run_partner(bufs, transforms, inputs)
      elif self.partner_active and self.partner.chestnut and self.primary.chestnut:
        self._queue_partner(inputs, queued)

    self._outputs["partner"] = None
    primary_out = self.primary.run(bufs, transforms, inputs, enqueued)
    self._outputs["primary"] = primary_out
    if queued:
      self._read_queued_partner(*queued[0])
    elif self.partner_active and not overlap:
      self._run_partner(bufs, transforms, inputs)
    return self._finish(t, primary_out, False)

  def _queue_partner(self, inputs, queued: list) -> None:
    """Both models on the Chestnut, the partner reading the primary's image: queue the partner's policy as soon as the
    primary's is queued, so the Chestnut runs them back to back while the comma waits for and reads the primary's
    outputs, instead of a second send and wait after them. Falls back to _run_partner when it cannot."""
    enqueue = getattr(self.partner, "enqueue_on_device_image", None)
    if enqueue is None or (image := self._primary_image()) is None:
      return
    t = self.now()
    try:
      queued.append((enqueue(image, inputs), t))
    except Exception as error:  # noqa: BLE001 - any partner failure stops the partner, never the drive
      self._stop_partner(f"partner failed: {type(error).__name__}: {error}"[:300])

  def _read_queued_partner(self, output_tensors, t: float) -> None:
    try:
      self.partner_image = "primary's Chestnut upload"
      self._outputs["partner"] = self.partner.read_queued(output_tensors)
    except Exception as error:  # noqa: BLE001
      self._stop_partner(f"partner failed: {type(error).__name__}: {error}"[:300])
      return
    self._partner_ms.append((self.now() - t) * 1e3)

  def _finish(self, t: float, primary_out: dict, late: bool) -> dict:
    run_s = self.now() - t
    self._run_ms.append(run_s * 1e3)
    self._frames += 1
    reason = self.guard.check(run_s, self.frame_drop_ratio, late) if self.partner_active else ""
    if self.partner_active and self._demote_now:
      reason, self._demote_now = self._demote_now, ""
    if self.partner_active and reason:
      if self.remote and not self.probing and self.demotions < MAX_DEMOTIONS:
        self._demote(reason)
      else:
        self._stop_partner(reason)
    partner_out = self._outputs["partner"]
    if not self.driving or partner_out is None:
      return primary_out
    lateral, longitudinal = ((partner_out, primary_out) if self.plan.partner_role == "lateral" else (primary_out, partner_out))
    try:
      return compose_model_outputs(lateral, longitudinal)
    except Exception as error:  # noqa: BLE001
      self._stop_partner(f"outputs could not be combined: {error}"[:300])
      return primary_out

  def get_action_from_model(self, model_output, prev_action, lat_action_t, long_action_t, v_ego):
    primary_out, partner_out = self._outputs["primary"], self._outputs["partner"]
    primary_action = self.primary.get_action_from_model(primary_out if primary_out is not None else model_output,
                                                        prev_action, lat_action_t, long_action_t, v_ego)
    if partner_out is None or not self.partner_active:
      self.write_status()
      return primary_action
    try:
      partner_action = self.partner.get_action_from_model(partner_out, prev_action, lat_action_t, long_action_t, v_ego)
    except Exception as error:  # noqa: BLE001
      self._stop_partner(f"partner action failed: {error}"[:300])
      return primary_action
    self._lat_diff.append(abs(partner_action.desiredCurvature - primary_action.desiredCurvature))
    self._long_diff.append(abs(partner_action.desiredAcceleration - primary_action.desiredAcceleration))
    self.write_status()
    if not self.driving:
      return primary_action
    lateral, longitudinal = ((partner_action, primary_action) if self.plan.partner_role == "lateral" else
                             (primary_action, partner_action))
    try:
      return log.ModelDataV2.Action(**hybrid_action_values(lateral, longitudinal))
    except Exception as error:  # noqa: BLE001
      self._stop_partner(f"actions could not be combined: {error}"[:300])
      return primary_action

  def summary(self) -> dict:
    return {
      "frames": self._frames,
      "runMsP50": _percentile(self._run_ms, 50), "runMsP95": _percentile(self._run_ms, 95), "runMsMax": _percentile(self._run_ms, 100),
      "partnerMsP50": _percentile(self._partner_ms, 50), "partnerMsP95": _percentile(self._partner_ms, 95),
      "curvatureDiffP50": _percentile(self._lat_diff, 50), "curvatureDiffP95": _percentile(self._lat_diff, 95),
      "accelDiffP50": _percentile(self._long_diff, 50), "accelDiffP95": _percentile(self._long_diff, 95),
      "frameDropRatio": round(float(self.frame_drop_ratio), 4),
      "lateFrames": int(sum(self._late)),
      "jobSentMsP50": _percentile(self._sent_ms, 50),
      "partnerReadyMsP50": _percentile(self._ready_ms, 50), "partnerReadyMsP95": _percentile(self._ready_ms, 95),
      **{f"partner{key[0].upper()}{key[1:]}P50": _percentile(values, 50) for key, values in self._diag.items()},
      **{f"partner{key[0].upper()}{key[1:]}P95": _percentile(values, 95) for key, values in self._diag.items()},
      "partnerGpuMhz": self._gpu_mhz,
      "probing": self.probing, "demotions": self.demotions,
      "probeOnTime": f"{sum(self._probe)}/{len(self._probe)}",
      "probeReadyMsP50": _percentile(self._probe_ready_ms, 50), "probeReadyMsP95": _percentile(self._probe_ready_ms, 95),
    }

  def write_status(self, force: bool = False) -> None:
    now = time.monotonic()
    if not force and now - self._last_status < STATUS_PERIOD_S:
      return
    self._last_status = now
    state = (("probing" if self.probing else "driving" if self.driving else "testing") if self.partner_active else "stopped")
    status = {"state": state, "mode": self.plan.mode, "lateralModel": self.plan.lateral_id,
              "longitudinalModel": self.plan.longitudinal_id, "primary": self.primary.model_id,
              "partner": self.plan.partner_id, "partnerDevice": self.partner_device, "sharedWarp": self.share_warp,
              "partnerImage": self.partner_image, "partnerProcess": self.remote,
              "reason": self.stop_reason, **self.summary()}
    write_status(status, self.status_path)
    if force or now - self._last_persist >= PERSIST_PERIOD_S:
      self._last_persist = now
      write_status(status, self.last_run_path)
    if self.report is not None and now - self._last_log >= LOG_PERIOD_S:
      self._last_log = now
      self.report("modelPair", {"state": state, **self.summary()})


def write_status(status: dict, path: Path | None = None) -> None:
  """Volatile status for Galaxy; written atomically, never raises."""
  target = STATUS_PATH if path is None else path
  try:
    tmp = target.with_name(target.name + ".tmp")
    tmp.write_text(json.dumps({"schemaVersion": STATUS_SCHEMA, "wallTime": time.time(), **status}))
    os.replace(tmp, target)
  except OSError:
    pass


def read_status(path: Path | None = None, *, max_age_s: float = 5.0) -> dict | None:
  """The pair's status: the live one, else the last drive's saved summary (fresh False); None when neither exists."""
  candidates = [path] if path is not None else [STATUS_PATH, LAST_RUN_PATH]
  status = None
  for candidate in candidates:
    try:
      status = json.loads(candidate.read_text())
      break
    except (OSError, ValueError):
      continue
  if status is None:
    return None
  if not isinstance(status, dict) or status.get("schemaVersion") != STATUS_SCHEMA:
    return None
  fresh = time.time() - float(status.get("wallTime", 0)) <= max_age_s
  return {**status, "fresh": fresh}


def _is_big(model_id: str) -> bool:
  from openpilot.starpilot.models import manager
  return bool(manager.catalog(manager.ROOT).get(model_id, {}).get("uses_external_gpu"))


def start_partner(model_id: str, cam_w: int, cam_h: int, streams: tuple[bool, bool], remote=None):
  """Start the small model's process on the comma GPU and wait until it is ready; raises when it cannot start."""
  path, version, sha256, _ = partner_artifact(model_id, chestnut_ok=False)
  if remote is None:
    from openpilot.starpilot.models.pair_partner import RemotePartner as remote
  partner = remote(model_id, version, path, sha256, cam_w, cam_h, main_wide_camera=streams[0], use_extra_client=streams[1])
  try:
    partner.wait_ready()
  except Exception:
    partner.close()
    raise
  return partner


def start_partner_spare(model_id: str, cam_w: int, cam_h: int, streams: tuple[bool, bool], *, remote=None,
                        report: Callable[[str], None] | None = None):
  """For a big + small pair: the small model's process, as modeld's spare driving model too (pair_partner.PartnerDriver),
  so modeld need not load its own copy. None when it cannot start; modeld then loads its usual spare."""
  from openpilot.starpilot.models.pair_partner import PartnerDriver
  try:
    partner = start_partner(model_id, cam_w, cam_h, streams, remote)
    return PartnerDriver(partner, int(getattr(partner, "info", {}).get("desireLen", 8)))
  except Exception as error:  # noqa: BLE001 - modeld falls back to loading its own small model
    if report is not None:
      report(f"pair partner process could not start as the spare: {error}"[:300])
    return None


def setup_pair(primary, small_model, cam_w: int, cam_h: int, *, config: dict | None = None, load=None,
               report: Callable[[str, dict], None] | None = None, streams: tuple[bool, bool] | None = None,
               remote=None):
  """Wrap the loaded primary in a ModelPair when the saved pair asks for one; returns (model, reason)."""
  from openpilot.starpilot.models.laboratory import configuration
  from openpilot.starpilot.models.manager import ROOT
  from openpilot.starpilot.models.runner import CatalogModelState

  config = configuration(ROOT) if config is None else config
  primary_id = getattr(primary, "model_id", "")
  plan, reason = plan_pair(config, primary_id)
  if plan is None:
    if reason != "off":
      write_status({"state": "refused", "mode": config.get("mode", "drive"), "lateralModel": config.get("lateralModel", ""),
                    "longitudinalModel": config.get("longitudinalModel", ""), "primary": primary_id, "reason": reason})
    else:
      write_status({"state": "off", "primary": primary_id})
    return primary, reason
  try:
    if not isinstance(primary, CatalogModelState):
      raise ValueError("the running model cannot be paired")
    small_beside_chestnut = bool(primary.chestnut and streams is not None and not _is_big(plan.partner_id))
    if small_beside_chestnut and plan.partner_id in local_variant_ids(ROOT):
      # a Chestnut build of the small model registered on this comma (registered only after bench_egpu showed the pair
      # fits the frame): both models on the Chestnut, the partner reading the primary's image, queued behind it
      try:
        path, version, sha256, on_chestnut = partner_artifact(plan.partner_id, chestnut_ok=True)
        if on_chestnut:
          if load is None:
            from openpilot.starpilot.models.runner import load_verified_model as load
          partner, _ = load(cam_w, cam_h, path, version, True, plan.partner_id, sha256)
          return ModelPair(primary, partner, plan, report=report), ""
      except Exception as error:  # noqa: BLE001 - the comma-GPU partner below is the fallback
        if report is not None:
          report("modelPairChestnutPartnerFailed", {"reason": f"{type(error).__name__}: {error}"[:300]})
    if small_beside_chestnut:
      # the small partner beside a Chestnut primary: its own process on the comma GPU, in parallel. modeld started it
      # already as its spare driving model (start_partner_spare) when it could: one copy of the small model, not two.
      spare = getattr(small_model, "partner", None)
      if spare is not None and getattr(small_model, "model_id", None) == plan.partner_id:
        return ModelPair(primary, spare, plan, report=report, keep_partner=True), ""
      partner = start_partner(plan.partner_id, cam_w, cam_h, streams, remote)
      return ModelPair(primary, partner, plan, report=report), ""
    # elsewhere: a Chestnut build of the partner wins when the primary is on the Chestnut; else the loaded small model is reused
    path, version, sha256, on_chestnut = partner_artifact(plan.partner_id, chestnut_ok=bool(primary.chestnut))
    if on_chestnut and not primary.chestnut:
      raise ValueError("the Chestnut is not running, so the big model cannot join the pair")
    if (not on_chestnut and small_model is not None and small_model is not primary and
        getattr(small_model, "model_id", None) == plan.partner_id and isinstance(small_model, CatalogModelState)):
      partner = small_model
    else:
      if load is None:
        from openpilot.starpilot.models.runner import load_verified_model as load
      partner, _ = load(cam_w, cam_h, path, version, on_chestnut, plan.partner_id, sha256)
    # the pair writes its status from its first frames, so a loader that timed out never reports a pair that is not running
    return ModelPair(primary, partner, plan, report=report), ""
  except Exception as error:  # noqa: BLE001 - a pair that cannot load never stops the drive
    reason = f"partner could not load: {error}"[:300]
    write_status({"state": "refused", "mode": plan.mode, "lateralModel": plan.lateral_id, "longitudinalModel": plan.longitudinal_id,
                  "primary": primary_id, "partner": plan.partner_id, "reason": reason})
    return primary, reason
