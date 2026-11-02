"""Pure car-control configuration shared by the input daemon and cruise owner."""
import json
import math

INPUTS = {}
for side in ('left', 'right'):
  for action, label in [('left','Nudge left'),('right','Nudge right'),('up','Scroll up'),('down','Scroll down'),('press','Press inward')]:
    INPUTS[f'{side}_wheel_{action}'] = (f'{side.title()} scroll wheel', label)
for direction in ('left', 'right'):
  for stage in ('half', 'full'):
    INPUTS[f'indicator_{direction}_{stage}'] = ('Left stalk', f'Indicator {direction} — {stage} press')
for key, label in [('beam_pull','Pull (flash beams)'),('beam_push','Push (high beams)'),
                   ('wiper_half','Wiper button — first detent'),('wiper_full','Wiper button — second detent')]:
  INPUTS[key] = ('Left stalk', label)
for direction in ('up', 'down'):
  for stage in ('half', 'full'):
    INPUTS[f'right_stalk_{direction}_{stage}'] = ('Right stalk', f'{direction.title()} — {stage} press')
INPUTS['park_press'] = ('Right stalk', 'Park button')
INPUTS['screen_three_finger_tap'] = ('Tesla screen', 'Three-finger tap')


PARAM = 'CarControlAssignments'
MAX_ASSIGNMENT_BYTES = 32768
MODELS = ('TESLA_MODEL_3', 'TESLA_MODEL_Y')


def parse_assignments(raw):
  if raw in (None, '', b'') or raw == {}:
    return {'version':1, 'vehicle':'', 'bindings':{}}
  try:
    data = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
    if data == {}:
      return {'version':1, 'vehicle':'', 'bindings':{}}
    if not isinstance(data, dict) or set(data) != {'version','vehicle','bindings'}:
      return None
    if type(data['version']) is not int or data['version'] != 1 or data['vehicle'] not in MODELS or not isinstance(data['bindings'], dict):
      return None
    for name, binding in data['bindings'].items():
      if name not in INPUTS or not isinstance(binding, dict) or set(binding) != {'key','value','label'}:
        return None
      if not isinstance(binding['key'], str) or not binding['key'] or not isinstance(binding['label'], str):
        return None
      value = binding['value']
      if value is not None and (type(value) not in (int,float) or not math.isfinite(value)):
        return None
    return data
  except (ValueError, TypeError, UnicodeError):
    return None


SPEED_UP = '__starpilot_controller_action__:comma_speed_up'
SPEED_DOWN = '__starpilot_controller_action__:comma_speed_down'
SPEED_SET = '__starpilot_controller_action__:set_speed'
SPEED_RESUME = '__starpilot_controller_action__:resume_speed_limit'
SPEED_ACTIONS = frozenset((SPEED_UP, SPEED_DOWN, SPEED_SET, SPEED_RESUME))


def scroll_speed_directions(raw, fingerprint):
  config = parse_assignments(raw)
  if not config or config['vehicle'] != fingerprint:
    return ()
  return tuple(direction for name, direction in (('right_wheel_up', 1), ('right_wheel_down', -1))
               if config['bindings'].get(name, {}).get('key') in SPEED_ACTIONS)



def vehicle_supported(cp):
  from opendbc.car.tesla.values import TeslaFlags
  return bool(cp is not None and cp.carFingerprint in MODELS and cp.brand=='tesla' and
              cp.flags & TeslaFlags.HAS_VEHICLE_BUS and not cp.passive and not cp.notCar and not cp.dashcamOnly)

def screen_reserved(cp):
  from opendbc.car.tesla.values import TeslaFlags,TeslaSafetyFlags
  return bool(cp.flags & TeslaFlags.AOL_SCREEN_BUTTON or
              any(s.safetyParam & TeslaSafetyFlags.AOL_SCREEN_BUTTON for s in cp.safetyConfigs))


def software_cruise_requested(params, cp):
  """Saved Tesla speed mappings request software ownership only while SLC is on."""
  from openpilot.starpilot.saved_source import read_saved
  try:
    if not vehicle_supported(cp) or not cp.pcmCruise or not cp.openpilotLongitudinalControl:
      return False
    if read_saved(params, 'SafeMode', 8) not in ((None, True), (b'0', True)):
      return False
    if read_saved(params, 'SpeedLimitController', 8) != (b'1', True):
      return False
    raw, readable = read_saved(params, PARAM, MAX_ASSIGNMENT_BYTES)
    config = parse_assignments(raw) if readable else None
    return bool(config and config['vehicle'] == cp.carFingerprint and any(
      b['key'] in (SPEED_UP, SPEED_DOWN, SPEED_SET) for b in config['bindings'].values()))
  except (AttributeError, TypeError, ValueError, OSError):
    return False
