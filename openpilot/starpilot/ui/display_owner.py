"""Parked, source-bound editor for the opt-in native display preferences."""

from collections.abc import Callable
from dataclasses import replace
import math

from openpilot.starpilot.ui.display_preferences import (
  AUTO, GPU_LOADING, BOOLEAN_KEYS, BRIGHTNESS, CUSTOM_KEYS, KEYS, MASTER, PIXEL_SHIFT, PIXEL_SHIFT_DESCRIPTION, SCREEN_OFF,
  SCREEN_OFF_DESCRIPTION, SLEEP_EYES, SLEEP_EYES_DESCRIPTION, SLEEP_EYES_PARKED, SLEEP_EYES_PARKED_DESCRIPTION, STANDALONE_KEYS,
  STATIC_DIM, STATIC_DIM_DESCRIPTION, default_value, read_choice,
)
from openpilot.starpilot.ui.feature_settings_state import FeatureRow, FeatureSettingsRequest, FeatureSettingsState
from openpilot.starpilot.ui.presentation import Profile


LABELS = {
  MASTER: "Custom Display Settings",
  "ScreenBrightness": "Parked Brightness",
  "ScreenBrightnessOnroad": "Driving Brightness",
  "ScreenTimeout": "Parked Screen Timeout",
  "ScreenTimeoutOnroad": "Driving Interaction Timeout",
  SCREEN_OFF: "Turn Off Comma Screen",
  SLEEP_EYES: "Sleep Eyes",
  SLEEP_EYES_PARKED: "Sleep Eyes Only When Parked",
  GPU_LOADING: "Keep Screen On While eGPU Loads",
  PIXEL_SHIFT: "Pixel Shift",
  STATIC_DIM: "Dim Static Elements",
}
STANDALONE_REASONS = {
  SCREEN_OFF: SCREEN_OFF_DESCRIPTION,
  SLEEP_EYES: SLEEP_EYES_DESCRIPTION,
  SLEEP_EYES_PARKED: SLEEP_EYES_PARKED_DESCRIPTION,
  GPU_LOADING: "Keep the driving screen awake while an eGPU model loads. Manual Screen Off still takes priority.",
  PIXEL_SHIFT: PIXEL_SHIFT_DESCRIPTION,
  STATIC_DIM: STATIC_DIM_DESCRIPTION,
}
AUTO_PREFIX = "display:auto:"
SCREEN_OFF_NEEDED_FOR_EYES = "Turn on Turn Off Comma Screen to use"
EYES_NEEDED_FOR_PARKED = "Turn on Sleep Eyes to use"
# Sleep Eyes replaces Turn Off Comma Screen's black screen, so it needs that option on; Only When Parked needs both.
EYES_REQUIRES = {SLEEP_EYES: (SCREEN_OFF,), SLEEP_EYES_PARKED: (SCREEN_OFF, SLEEP_EYES)}


def eyes_blocker(key: str, saved: dict) -> str:
  if key not in EYES_REQUIRES:
    return ""
  screen_off = saved[SCREEN_OFF]
  if not (screen_off.readable and screen_off.valid and screen_off.value == 1):
    return SCREEN_OFF_NEEDED_FOR_EYES
  if SLEEP_EYES in EYES_REQUIRES[key]:
    eyes = saved[SLEEP_EYES]
    if not (eyes.readable and eyes.valid and eyes.value == 1):
      return EYES_NEEDED_FOR_PARKED
  return ""


BRIGHTNESS_CHOICES = ("Auto",) + tuple(f"{number}%" for number in range(5, 101))
TIMEOUT_CHOICES = tuple(f"{number} s" for number in range(5, 61, 5))


def _display(key: str, value: int | None, valid: bool, unsupported: bool) -> str:
  if unsupported:
    return "Saved Off (unsupported)"
  if not valid or value is None:
    return "Invalid saved choice"
  if key in BOOLEAN_KEYS:
    return "On" if value else "Off"
  if key in BRIGHTNESS:
    return "Auto" if value == AUTO else str(value)
  return str(value)


class DisplayOwner:
  def __init__(self, params, parked: Callable[[], bool], *, gpu_capable=None):
    from openpilot.common.hardware import HARDWARE
    self.gpu_capable = HARDWARE.get_device_type() in ("tici", "mici") if gpu_capable is None else bool(gpu_capable)
    self.params = params
    self.parked = parked

  def snapshot(self, profile: Profile) -> FeatureSettingsState:
    large = profile == Profile.LARGE
    parked = self.parked()
    saved = {key: read_choice(self.params, key, large=large) for key in KEYS}
    master = saved[MASTER]
    rows = []
    for key in KEYS:
      if key == GPU_LOADING and not self.gpu_capable:
        continue
      choice = saved[key]
      valid = choice.valid
      value = _display(key, choice.value, valid, choice.unsupported)
      if not choice.readable:
        reason = "Saved choice cannot be read"
      elif choice.unsupported:
        reason = "Choose Auto before enabling custom display"
      elif not valid:
        reason = "Choose default to repair"
      elif key in STANDALONE_KEYS:
        reason = STANDALONE_REASONS[key]
      elif key != MASTER and master.value != 1:
        reason = "Saved; custom display is off"
      elif key == "ScreenTimeoutOnroad":
        reason = "Returns to driving view; turns off screen if Turn Off Comma Screen is on"
      else:
        reason = ""
      boolean = key in BOOLEAN_KEYS
      repair = "Off" if boolean else "Auto" if key in BRIGHTNESS else "30 s" if key == "ScreenTimeout" else \
        "10 s" if large else "5 s"
      dependencies = (tuple((name, saved[name].raw) for name in EYES_REQUIRES[key]) if key in EYES_REQUIRES else
                      () if key in STANDALONE_KEYS else tuple((name, saved[name].raw) for name in CUSTOM_KEYS) if key == MASTER else
                      ((MASTER, master.raw),))
      editable = parked and choice.readable and (boolean or master.valid and master.value == 1 or not valid)
      if blocker := eyes_blocker(key, saved):
        # Greyed out; a saved On stays visible (and inactive) until its requirement is met again.
        editable = False
        reason = blocker
      numeric = valid and not boolean and choice.value != AUTO
      rows.append(FeatureRow(key, LABELS[key], value, source=choice.raw,
                             choices=("Off", "On") if boolean else
                                     (("Auto",) if numeric else BRIGHTNESS_CHOICES) if key in BRIGHTNESS else (),
                             step=(1.0 if key in BRIGHTNESS else 5.0) if valid and not boolean else 0.0,
                             minimum=5.0, maximum=100.0 if key in BRIGHTNESS else 60.0,
                             unit=("%" if key in BRIGHTNESS else "s") if valid and not boolean else "",
                             available=editable, reason=reason,
                             repair_value=repair if not valid and choice.readable else "",
                             dependencies=dependencies))
      if key in BRIGHTNESS and valid and choice.value != AUTO:
        rows.append(FeatureRow(AUTO_PREFIX + key, "Use Auto " + LABELS[key], "Fixed level saved",
                               source=choice.raw, available=editable,
                               repair_value="Auto", dependencies=((MASTER, master.raw),)))
    defaults = {key: ('Off' if key in BOOLEAN_KEYS else 'Auto' if key in BRIGHTNESS else str(default_value(key, large=large)))
                for key in KEYS}
    rows = tuple(replace(row, default_value=defaults[row.key]) if row.key in defaults else row for row in rows)
    return FeatureSettingsState(page="display", title="Display", subtitle="Saved brightness and interaction timing.",
                                rows=tuple(rows), parked=parked)

  def apply(self, request: FeatureSettingsRequest) -> bool:
    key = request.key.removeprefix(AUTO_PREFIX)
    if key not in KEYS or not self.parked() or key == GPU_LOADING and not self.gpu_capable:
      return False
    if request.key.startswith(AUTO_PREFIX) and key not in BRIGHTNESS:
      return False
    if key in BOOLEAN_KEYS:
      if request.value not in ("Off", "On"):
        return False
      desired = int(request.value == "On")
    elif key in BRIGHTNESS:
      if request.value == "Auto":
        desired = AUTO
      else:
        try:
          number = float(request.value.removesuffix("%"))
        except ValueError:
          return False
        if not math.isfinite(number) or not number.is_integer() or not 5 <= number <= 100:
          return False
        desired = int(number)
    else:
      try:
        number = float(request.value.removesuffix(" s"))
      except ValueError:
        return False
      if not math.isfinite(number) or not number.is_integer() or int(number) % 5 or not 5 <= number <= 60:
        return False
      desired = int(number)
    for final in (False, True):
      if not self.parked():
        return False
      observed = read_choice(self.params, key, large=True)
      if not observed.readable or observed.raw != request.expected:
        return False
      if key in EYES_REQUIRES and desired == 1:
        expected = dict(request.dependencies)
        if set(expected) != set(EYES_REQUIRES[key]):
          return False
        current = {name: read_choice(self.params, name, large=True) for name in KEYS}
        if any(current[name].raw != expected[name] for name in EYES_REQUIRES[key]) or eyes_blocker(key, current):
          return False
      # Turning the master off stays available even when other saved choices
      # are corrupt or have changed while the user was viewing the page.
      if key not in STANDALONE_KEYS and not (key == MASTER and desired == 0):
        expected = dict(request.dependencies)
        required = set(CUSTOM_KEYS) if key == MASTER else {MASTER}
        if set(expected) != required:
          return False
        for name in required:
          source = read_choice(self.params, name, large=True)
          if not source.readable or source.raw != expected[name] or (key == MASTER and not source.valid):
            return False
          if key != MASTER and observed.valid and (not source.valid or source.value != 1):
            return False
      if not final:
        continue
    try:
      if key in BOOLEAN_KEYS:
        self.params.put_bool(key, bool(desired), block=True)
      else:
        self.params.put(key, desired, block=True)
    except (OSError, KeyError, TypeError, ValueError):
      return False
    result = read_choice(self.params, key, large=True)
    return result.readable and result.valid and result.value == desired
