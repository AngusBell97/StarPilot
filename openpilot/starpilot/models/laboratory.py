from pathlib import Path

from openpilot.starpilot.models.catalog import model_entries, BUNDLED_CURRENT, GENERATION
from openpilot.starpilot.models.manager import (
  ModelError, ModelManager, artifact_entry, artifact_path, atomic_json, catalog, preferences, read_json, state_lock,
)
from openpilot.starpilot.models.model_pair import MODES, read_status

KEYS = {"enabled", "lateralModel", "longitudinalModel"}


def configuration(root: Path) -> dict:
  default = {"enabled": False, "lateralModel": "", "longitudinalModel": "", "mode": "shadow"}
  try:
    saved = read_json(root / "laboratory.json", 4096)
    validate_configuration(saved, root)
    return {"mode": "shadow", **saved}
  except (OSError, ValueError, TypeError):
    return default


def validate_configuration(value: dict, root: Path | None = None) -> None:
  """A pair of any two catalog models; "mode" is shadow (log only, the default) or drive."""
  entries = model_entries(root)
  if (not isinstance(value, dict) or not KEYS <= set(value) <= KEYS | {"mode"} or type(value["enabled"]) is not bool or
      value.get("mode", "shadow") not in MODES):
    raise ModelError("Invalid laboratory configuration")
  for key in ("lateralModel", "longitudinalModel"):
    mid = value[key]
    if not isinstance(mid, str) or (mid and (mid not in entries or mid == BUNDLED_CURRENT)):
      raise ModelError("Choose catalog models for the laboratory pair")
  if value["enabled"] and (not value["lateralModel"] or not value["longitudinalModel"] or
                           value["lateralModel"] == value["longitudinalModel"]):
    raise ModelError("Choose two different models")


def running_models(owner: ModelManager) -> tuple[str, str, str]:
  """(model modeld runs now, Active Small, Active Big): Big when a Chestnut is present and one is chosen."""
  prefs = preferences(owner.root)
  small, big = prefs["small"], prefs["big"]
  return (big if owner.gpu_present() and big else small), small, big


class ModelLaboratory:
  def __init__(self, manager: ModelManager, *, runtime_supported: bool = True):
    self.manager = manager
    self.runtime_supported = runtime_supported

  @property
  def runtime_unavailable_reason(self) -> str:
    return "" if self.runtime_supported else "Pair inference is not available in this build yet."

  def snapshot(self) -> dict:
    owner = self.manager
    with owner.lock:
      entries = catalog(owner.root)
      config = configuration(owner.root)
      parked, gpu = owner.parked(), owner.gpu_present()
      job = owner._job_status()
      running, active_small, active_big = running_models(owner)
      models = []
      for mid, row in entries.items():
        small = not row.get("uses_external_gpu", False)
        eligible = small and row.get("model_lab_eligible") is True
        artifact = artifact_entry(mid, entries, "amd") if eligible else {}
        published = "artifact_sha256" in artifact
        amd_installed, checking = owner._snapshot_installed(mid, row, "amd") if published else (False, False)
        installed, verifying = owner._snapshot_installed(mid, row)
        if not small:
          state, reason = "unsupported", "Big models run on the Chestnut already and need no eGPU variant."
        elif not eligible:
          state, reason = "unsupported", "This model is not available for combining."
        elif not published:
          state, reason = "unpublished", "A Chestnut version for combining has not been published for this model."
        elif checking:
          state, reason = "checking", "Checking the downloaded Chestnut version."
        elif not amd_installed:
          state, reason = "missing", "Download the Chestnut version to run this model on the Chestnut in a pair."
        else:
          state, reason = "installed", "Download verified. In a pair this model runs on the Chestnut."
        models.append({"value": mid, "label": row["name"], "version": row["version"], "series": row.get("series", ""),
                       "small": small, "modelSize": "small" if small else "big", "modelLabEligible": eligible,
                       "modelLabArtifactAvailable": published, "modelLabArtifactInstalled": amd_installed, "checking": checking,
                       "modelLabStatus": state, "modelLabReason": reason,
                       "installed": installed, "pairable": installed or verifying or amd_installed,
                       "placement": "chestnut" if not small or amd_installed else "comma",
                       "artifactSize": artifact.get("artifact_size", 0)})
      status = read_status()
      live = status is not None and status.get("fresh") and status.get("state") in ("testing", "driving")
      runtime = {"requested": config["enabled"], "active": bool(live), "state": (status or {}).get("state", "off"),
                 "lateralModel": (status or {}).get("lateralModel", "") if live else "",
                 "longitudinalModel": (status or {}).get("longitudinalModel", "") if live else "",
                 "error": (status or {}).get("reason", "") if status and status.get("state") in ("refused", "stopped") else "",
                 "health": "active" if live else "unavailable", "stats": status or {}}
      return {"schemaVersion": 1, "chestnutReady": gpu, "isOnroad": not parked, "configuration": config,
              "configurationError": "",
              "runningModel": running, "activeSmallModel": active_small, "activeBigModel": active_big,
              "runtimeSupported": self.runtime_supported, "runtimeUnavailableReason": self.runtime_unavailable_reason,
              "runtime": runtime,
              "download": {"model": job.get("model", ""), "progress": job.get("progress", ""),
                           "downloading": job["downloading"], "jobId": job.get("jobId", ""),
                           "variant": job.get("variant", "standard")},
              "summary": {"catalog": len(models), "eligible": sum(row["modelLabEligible"] is True for row in models),
                          "ready": sum(row["modelLabArtifactInstalled"] is True for row in models),
                          "published": sum(row["modelLabArtifactAvailable"] is True for row in models),
                          "pairable": sum(row["pairable"] is True for row in models),
                          "declaredSize": sum(int(row["artifactSize"]) for row in models)},
              "models": models, "manifest": {"version": GENERATION},
              "capabilities": {"configure": parked, "download": parked,
                               "delete": parked, "cancel": True, "refresh": parked}}

  def action(self, action: str, payload: dict) -> dict:
    owner = self.manager
    if not isinstance(payload, dict):
      raise ModelError("Invalid laboratory request")
    with owner.lock:
      owner._require_parked()
      if action == "download":
        if set(payload) != {"model"} or not isinstance(payload["model"], str):
          raise ModelError("Unknown laboratory model")
        return owner.action("download", {"model": payload["model"], "variant": "amd", "allowGpuWithoutGpu": True})
      if action not in ("configure", "delete"):
        raise ModelError("Unknown laboratory action")
      with state_lock(owner.root):
        owner._require_parked()
        if owner._job_busy():
          raise ModelError("Wait for the model download to finish")
        if action == "configure":
          validate_configuration(payload, owner.root)
          if payload["enabled"]:
            if not self.runtime_supported:
              raise ModelError(self.runtime_unavailable_reason)
            entries = catalog(owner.root)
            for role in ("lateralModel", "longitudinalModel"):
              mid = payload[role]
              # installed, or still being checked after its download: modeld verifies the digest again when it loads
              if not any(any(owner._snapshot_installed(mid, entries[mid], variant)) for variant in ("standard", "amd")):
                raise ModelError(f"Download {entries[mid]['name']} before pairing it")
            _, small, big = running_models(owner)
            if not {small, big} & {payload["lateralModel"], payload["longitudinalModel"]}:
              raise ModelError("The pair must include your Active Small or Active Big model")
            owner._require_parked()
          atomic_json(owner.root / "laboratory.json", {"mode": "shadow", **payload})
          message = "Model Laboratory configuration saved"
        else:
          if set(payload) != {"model"} or not isinstance(payload["model"], str):
            raise ModelError("Unknown laboratory model")
          mid = payload["model"]
          destination = artifact_path(mid, owner.root, "amd")
          config = configuration(owner.root)
          if config["enabled"] and mid in (config["lateralModel"], config["longitudinalModel"]):
            raise ModelError("Disable the laboratory pair before deleting either eGPU variant")
          destination.unlink(missing_ok=True)
          owner.checked.pop(f"{mid}:amd", None)
          message = "eGPU variant deleted"
      return {**self.snapshot(), "message": message}
