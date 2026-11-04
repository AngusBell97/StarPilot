"""Model Laboratory pairs in modeld: steering from one model and speed from the other.

Pairs are two small models, or one big and one small (two big ones do not fit one Chestnut's 50 ms frame). With a
pair switched on, modeld loads the pair's models instead of the Active ones (`pair_selection`): the big one on the
Chestnut as the running model, else the lateral small one; the other model is the partner. A big partner would run
on the Chestnut after the primary, sharing its warped camera image when the models allow it. A small partner runs on
the Chestnut when its AMD variant is installed and the primary is there, else on the comma's GPU, overlapping the
Chestnut's compute when the primary is on the Chestnut. Mode "drive" (default) composes the outputs; "shadow" keeps
a log-only test path that Galaxy no longer offers.

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
WARMUP_FRAMES = 20          # frames before the guard judges (first runs are slower)
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

  def check(self, run_s: float, frame_drop_ratio: float) -> str:
    self.frames += 1
    self.slow.append(run_s > SLOW_FRAME_S)
    if self.frames <= WARMUP_FRAMES:
      return ""
    if frame_drop_ratio > DROP_LIMIT:
      return f"frames dropped ({frame_drop_ratio:.0%}) while the pair ran"
    if sum(self.slow) > SLOW_LIMIT:
      return f"pair too slow: {sum(self.slow)} of the last {len(self.slow)} frames over {SLOW_FRAME_S * 1e3:.0f} ms"
    return ""


def _percentile(values, q: float) -> float | None:
  return round(float(np.percentile(np.asarray(values), q)), 2) if len(values) else None


class ModelPair:
  """Drop-in for the driving model in modeld's loop (run, get_action_from_model, chestnut, npy, ...)."""

  def __init__(self, primary, partner, plan: PairPlan, *, now: Callable[[], float] = time.perf_counter,
               status_path: Path | None = None, report: Callable[[str, dict], None] | None = None):
    self.primary = primary
    self.partner = partner
    self.plan = plan
    self.now = now
    self.status_path = STATUS_PATH if status_path is None else status_path
    self.report = report
    self.guard = PairGuard()
    self.partner_active = True
    self.stop_reason = ""
    self.partner_device = "chestnut" if partner.chestnut else "comma"
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
    return self.partner_active and self.plan.mode == "drive"

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
    if self.report is not None:
      self.report("modelPairStopped", {"reason": reason, **self.summary()})
    self.write_status(force=True)

  def _run_partner(self, bufs, transforms, inputs) -> None:
    t = self.now()
    try:
      if self.share_warp:
        self._outputs["partner"] = self.partner.run(bufs, transforms, inputs, shared_warp=self.primary.last_warp)
      else:
        self._outputs["partner"] = self.partner.run(bufs, transforms, inputs)
    except Exception as error:  # noqa: BLE001 - any partner failure stops the partner, never the drive
      self._stop_partner(f"partner failed: {type(error).__name__}: {error}"[:300])
      return
    self._partner_ms.append((self.now() - t) * 1e3)

  def run(self, bufs, transforms, inputs, after_enqueue: Callable[[], None] | None = None) -> dict:
    for model in (self.primary, self.partner):
      model.lat_delay, model.frame_drop_ratio = self.lat_delay, self.frame_drop_ratio
    t = self.now()
    overlap = self.partner_active and not self.partner.chestnut and self.primary.chestnut

    def enqueued():
      if after_enqueue is not None:
        after_enqueue()
      if overlap and self.partner_active:
        self._run_partner(bufs, transforms, inputs)

    self._outputs["partner"] = None
    primary_out = self.primary.run(bufs, transforms, inputs, enqueued)
    self._outputs["primary"] = primary_out
    if self.partner_active and not overlap:
      self._run_partner(bufs, transforms, inputs)
    run_s = self.now() - t
    self._run_ms.append(run_s * 1e3)
    self._frames += 1
    if self.partner_active and (reason := self.guard.check(run_s, self.frame_drop_ratio)):
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
    }

  def write_status(self, force: bool = False) -> None:
    now = time.monotonic()
    if not force and now - self._last_status < STATUS_PERIOD_S:
      return
    self._last_status = now
    state = ("driving" if self.driving else "testing") if self.partner_active else "stopped"
    status = {"state": state, "mode": self.plan.mode, "lateralModel": self.plan.lateral_id,
              "longitudinalModel": self.plan.longitudinal_id, "primary": self.primary.model_id,
              "partner": self.plan.partner_id, "partnerDevice": self.partner_device, "sharedWarp": self.share_warp,
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


def setup_pair(primary, small_model, cam_w: int, cam_h: int, *, config: dict | None = None, load=None,
               report: Callable[[str, dict], None] | None = None):
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
    if small_model is not None and small_model is not primary and getattr(small_model, "model_id", None) == plan.partner_id \
       and isinstance(small_model, CatalogModelState):
      partner = small_model
    else:
      if load is None:
        from openpilot.starpilot.models.runner import load_verified_model as load
      path, version, sha256, on_chestnut = partner_artifact(plan.partner_id, chestnut_ok=bool(primary.chestnut))
      if on_chestnut and not primary.chestnut:
        raise ValueError("the Chestnut is not running, so the big model cannot join the pair")
      partner, _ = load(cam_w, cam_h, path, version, on_chestnut, plan.partner_id, sha256)
    # the pair writes its status from its first frames, so a loader that timed out never reports a pair that is not running
    return ModelPair(primary, partner, plan, report=report), ""
  except Exception as error:  # noqa: BLE001 - a pair that cannot load never stops the drive
    reason = f"partner could not load: {error}"[:300]
    write_status({"state": "refused", "mode": plan.mode, "lateralModel": plan.lateral_id, "longitudinalModel": plan.longitudinal_id,
                  "primary": primary_id, "partner": plan.partner_id, "reason": reason})
    return primary, reason
