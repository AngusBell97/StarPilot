"""Lane version 10 (lane-led centring) Galaxy controls: every slider from the plan, saved under LaneCenteringV10* keys.

The root lane page shows the master switch, pause on signal, the lane offset and links to six groups. Each group is
its own page. Values are stored in SI; percentages and the bias limit are scaled for display only.
"""
from dataclasses import replace
import json
import math

from openpilot.starpilot.lateral.lane_v10_settings import RANGES
from openpilot.starpilot.lateral.lane_v10 import LeverLearner
from openpilot.starpilot.lateral.lane_v10_adapter import LEVER_STATE_KEY, V10_BOOLS, V10_DEFAULTS, V10_MAP, V10_VERSION
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

KEYS = frozenset(V10_DEFAULTS)
VERSION = "LaneCenteringVersion"
SELECTED = str(V10_VERSION).encode()
PAUSE = V10_MAP["pause_on_signal"]
OFFSET = V10_MAP["offset_m"]

# field -> (label, unit, display scale, step in display units, help)
SPECS = {
  "offset_m": ("Lane offset", "m", 1, .01, "Shift the target left or right of the geometric lane centre, in metres. Positive is right (two-line road only). Saved separately for v10."),
  "recovery_time": ("Recovery time", "s", 1, .1, "Time for a 30 cm offset to come within 10 cm of the target. Shorter is more assertive; the limits below still apply."),
  "damping": ("Damping", "", 1, .05, "Damping ratio of the recovery. 1.0 is critical: fastest without overshoot. Below 1 is quicker with a small overshoot; above 1 is slower and never overshoots."),
  "preview_factor": ("Preview", "x delay", 1, .1, "Where the road curvature feedforward is read, as a multiple of the measured steering delay. Raise it a little if the car runs wide entering bends."),
  "axle_to_camera_m": ("Camera ahead of axle", "m", 1, .1, "Distance from the rear axle to the camera. The lane heading is seen at the camera but the car turns about the axle; without this the car settles inside every bend. With learning on this is the starting value at 18 m/s and the learner finds the speed dependence; off, it is used as is at every speed. Raise it if bends are still cut, lower it if the car runs wide."),
  "max_added_accel": ("Max added acceleration", "m/s2", 1, .05, "Lateral acceleration the controller may add above following the road, for the correction and the learned bias together."),
  "max_jerk": ("Max jerk", "m/s3", 1, .05, "How fast the controller's own contribution may change. The road's own curvature changes are followed separately."),
  "gain_floor_speed": ("Gain floor speed", "m/s", 1, .5, "Below this speed the feedback gains stop rising, so low-speed corrections stay gentle."),
  "adopt_confidence": ("Adopt confidence", "%", 100, 1, "Lane-line probability a line needs, for the switch-hold time, before it is used."),
  "drop_confidence": ("Drop confidence", "%", 100, 1, "Probability below which a used line is dropped, after the switch-hold time. Keep it below the adopt confidence."),
  "max_line_std": ("Max line uncertainty", "m", 1, .05, "Line position uncertainty above which a line is not used."),
  "edge_std": ("Max edge uncertainty", "m", 1, .05, "Road-edge uncertainty above which an edge is not tracked. Edges never define the centre in v10; they are kept for a future clearance guard."),
  "switch_hold": ("Switch hold", "s", 1, .05, "How long a change in the usable lines must persist before it is adopted or dropped. Prevents flapping between sources."),
  "min_width": ("Minimum lane width", "m", 1, .1, "Narrowest two-line gap accepted as a lane."),
  "max_width": ("Maximum lane width", "m", 1, .1, "Widest two-line gap accepted as a lane."),
  "innovation_limit_m": ("Innovation limit", "m", 1, .01, "Largest move of the tracked centre accepted from one model frame. Larger changes are spread over several frames."),
  "innovation_limit_deg": ("Heading innovation limit", "deg", 1, .1, "Largest change of the tracked lane heading accepted from one model frame."),
  "car_width": ("Car width", "m", 1, .05, "Body width without mirrors. With the clearances below it sets how close to a line or edge the car may be held."),
  "line_clearance": ("Line clearance", "m", 1, .05, "Minimum gap between the car's side and a lane line. With one line usable the car holds its position and follows the line's direction; it is pushed away only when closer than this."),
  "edge_clearance": ("Edge clearance", "m", 1, .05, "Minimum gap between the car's side and a road edge (kerb, grass). Edges never attract the car; they only push."),
  "fade_in": ("Fade in", "s", 1, .1, "Time for the lane controller to take over from the model once the lines are trusted."),
  "fade_out": ("Fade out", "s", 1, .1, "Time to hand back to the model when trust is lost or a hand-back applies."),
  "model_divergence_m": ("Model divergence", "m", 1, .05, "If the model's own path leaves the tracked lane centre by more than this one second ahead, the model drives (a junction, split or obstacle)."),
  "hold_through_gap": ("Hold through gap", "s", 1, .05, "How long the tracked centre is carried on the car's own motion without new lines before fading to the model."),
  "min_speed": ("Minimum speed", "m/s", 1, .5, "Below this speed the model drives."),
  "bias_time": ("Bias time", "s", 1, .5, "Time constant of the slow bias that removes a steady lean (camera offset, camber)."),
  "bias_limit_accel": ("Bias limit", "m/s2", 1, .05, "Largest steady lateral acceleration the bias may hold to cancel a lean (camber, camera offset). As a curvature this is larger at low speed, where camber needs it."),
  "tau_steer": ("Steering response", "s", 1, .01, "First-order steering response time assumed by the delay predictor."),
  "slew_time": ("Setting change time", "s", 1, .1, "A changed slider takes effect gradually over this time, so adjusting while driving cannot itself cause a jerk."),
}
BOOL_SPECS = {
  "pause_on_signal": ("Pause on signal", "Hand steering to the model while a turn signal is on, as during a lane change."),
  "lever_learn": ("Learn camera lever", "Learn the camera's effective distance ahead of the axle from steady bends while driving, starting from the slider above and remembered between drives. Off uses the slider as a constant."),
  "bias_enabled": ("Learn bias", "Let the slow bias remove a steady lean. Off keeps the bias at zero."),
  "bias_persist": ("Keep bias between drives", "Carry the learned bias across engagements instead of starting from zero each time."),
  "edge_guard": ("Guard against road edges", "Keep the edge clearance from detected road edges as well as lines. Off: only lines push."),
}
GROUPS = {
  "lane/recovery": ("Recovery", "How fast and how firmly the lane centre is regained.", ("recovery_time", "damping", "preview_factor", "axle_to_camera_m", "lever_learn")),
  "lane/limits": ("Limits", "Comfort bounds on what the controller adds.", ("max_added_accel", "max_jerk", "gain_floor_speed")),
  "lane/trust": ("Trust", "Which lines are used and how a change of source is absorbed.",
                 ("adopt_confidence", "drop_confidence", "max_line_std", "edge_std", "switch_hold", "min_width", "max_width", "innovation_limit_m", "innovation_limit_deg")),
  "lane/oneline": ("One line and clearance", "With one line the car holds its position and follows the line; lines and edges keep a minimum clearance.",
                   ("car_width", "line_clearance", "edge_clearance", "edge_guard")),
  "lane/handover": ("Hand-over", "When and how fast the lane controller and the model exchange control.",
                    ("fade_in", "fade_out", "model_divergence_m", "hold_through_gap", "min_speed")),
  "lane/bias": ("Bias learning", "The slow term that removes a steady lean.", ("bias_enabled", "bias_time", "bias_limit_accel", "bias_persist", "tau_steer", "slew_time")),
}
PAGES = tuple(GROUPS)
TITLES = {page: spec[0] for page, spec in GROUPS.items()}


def _number_row(owner, field, allowed):
  key = V10_MAP[field]
  label, unit, scale, step, help_text = SPECS[field]
  low, high = RANGES[field]
  value, source, valid = owner._value(key)
  try:
    number = float(value)
    valid = valid and math.isfinite(number) and low <= number <= high
  except (TypeError, ValueError):
    number, valid = 0., False
  shown = round(number * scale, 4)
  return FeatureRow(key, label, (str(int(shown)) if float(shown).is_integer() and scale != 1 else str(shown)) if valid else "Invalid saved value",
                    source, available=allowed and valid, minimum=round(low * scale, 6), maximum=round(high * scale, 6),
                    step=step if valid else 0., unit=unit, display_unit=unit,
                    default_value=str(round(V10_DEFAULTS[key] * scale, 4)), reason=help_text if valid else "Invalid saved value",
                    dependencies=((VERSION, SELECTED),))


def _bool_row(owner, field, allowed):
  label, help_text = BOOL_SPECS[field]
  return replace(owner._bool_row(V10_MAP[field], label, allowed, help_text), dependencies=((VERSION, SELECTED),))


def rows(owner, page, allowed):
  if page == "lane":
    result = [owner._bool_row("LaneCentering", "Enable Lane Centering", allowed,
                              "v10 drives from the lane lines when they are trusted and hands to the model when they are not."),
              _bool_row(owner, "pause_on_signal", allowed), _number_row(owner, "offset_m", allowed)]
    if owner._raw("LaneCentering") is None:
      result[0] = replace(result[0], value="Off", default_value="Off")
    result[0] = replace(result[0], dependencies=((VERSION, SELECTED),))
    for group_page, (title, blurb, _) in GROUPS.items():
      result.append(FeatureRow("", title, blurb, page=group_page, available=True))
    return result
  if page not in GROUPS:
    return []
  _, _, fields = GROUPS[page]
  result = [_bool_row(owner, f, allowed) if f in BOOL_SPECS else _number_row(owner, f, allowed) for f in fields]
  if page == "lane/recovery":
    result.append(FeatureRow("", "Learned camera lever", _learned_lever_text(owner), available=False))
  return result


def _learned_lever_text(owner) -> str:
  """Read-only: what the learner currently holds, at 12, 18 and 24 m/s, with the slider as its prior."""
  try:
    raw = owner._raw(LEVER_STATE_KEY)
    prior = float(owner._value(V10_MAP["axle_to_camera_m"])[0])
  except Exception:
    return "Unavailable"
  learner = LeverLearner()
  if not raw:
    return "Nothing learned yet"
  try:
    ok = learner.load(json.loads(raw))
  except (ValueError, TypeError):
    ok = False
  if not ok or learner.n <= 0.0:
    return "Nothing learned yet"
  levers = " / ".join(f"{learner.lever(v, prior):.2f}" for v in (12.0, 18.0, 24.0))
  return f"{levers} m at 12 / 18 / 24 m/s, from {learner.n / 20.0 / 60.0:.0f} min of frames"


def error(owner, request):
  if request.key not in KEYS:
    return "Unknown v10 setting"
  if owner._raw(VERSION) != SELECTED or dict(request.dependencies).get(VERSION) != SELECTED:
    return "Lane-centering version changed; refresh settings"
  current, source, valid = owner._value(request.key)
  if not valid or source != request.expected:
    return "Saved v10 setting changed; refresh settings"
  if request.key in V10_BOOLS:
    return "" if current in ("0", "1") and request.value in ("On", "Off") else "Invalid switch value"
  field = next(f for f, k in V10_MAP.items() if k == request.key)
  _, unit, scale, step, _ = SPECS[field]
  low, high = RANGES[field]
  if request.display_unit != unit:
    return "Slider units changed; refresh settings"
  try:
    shown = float(request.value)
    stored = shown / scale
    if not math.isfinite(shown) or not low - 1e-9 <= stored <= high + 1e-9:
      return "Value outside setting range"
    if field == "drop_confidence" and stored >= float(owner._value(V10_MAP["adopt_confidence"])[0]):
      return "Drop confidence must be below adopt confidence"
    if field == "adopt_confidence" and stored <= float(owner._value(V10_MAP["drop_confidence"])[0]):
      return "Adopt confidence must be above drop confidence"
    if field == "min_width" and stored >= float(owner._value(V10_MAP["max_width"])[0]):
      return "Minimum lane width must be below maximum lane width"
    if field == "max_width" and stored <= float(owner._value(V10_MAP["min_width"])[0]):
      return "Maximum lane width must be above minimum lane width"
  except (TypeError, ValueError, OverflowError):
    return "Invalid numeric value"
  return ""


def apply(owner, request):
  from openpilot.starpilot.lateral.lane_runtime import runtime_supported

  def authorized():
    return (not error(owner, request) and runtime_supported(owner.vehicle_params()) and
            bool(request.vehicle_fingerprint) and owner.vehicle_fingerprint() == request.vehicle_fingerprint and
            (owner.authority("lane") or owner.authority("lane_live")) and
            all(owner._readable(key) and owner._raw(key) == value for key, value in request.dependencies))

  if not authorized():
    return False
  if request.key in V10_BOOLS:
    encoded = b"1" if request.value == "On" else b"0"
  else:
    field = next(f for f, k in V10_MAP.items() if k == request.key)
    low, high = RANGES[field]
    value = min(high, max(low, float(request.value) / SPECS[field][2]))
    encoded = str(round(value, 8)).encode()
  return commit_exact(owner.params, key=request.key, max_bytes=128, raw=encoded, expected=request.expected,
                      authorized=authorized, temp_prefix=".lane-v10-").verified
