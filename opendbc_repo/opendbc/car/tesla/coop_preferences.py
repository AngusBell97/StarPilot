"""Model 3 cooperative steering is selected before parser/controller creation."""
from opendbc.car.tesla.screen_button import SPEED_SYNC, speed_sync_allowed
from opendbc.car.tesla.values import CAR, TeslaSafetyFlags

COOP = int(TeslaSafetyFlags.COOP_STEERING)
BASE_WORDS = frozenset((0, 1, 512, 513, 1536, 1537))


def supported(cp):
  try:
    return bool(cp.brand == 'tesla' and cp.carFingerprint == CAR.TESLA_MODEL_3 and
                not (cp.passive or cp.dashcamOnly or cp.notCar) and len(cp.safetyConfigs) == 1 and
                int(cp.safetyConfigs[0].safetyModel.raw) == 10 and
                (int(cp.safetyConfigs[0].safetyParam) & ~(COOP | SPEED_SYNC)) in BASE_WORDS and
                (not int(cp.safetyConfigs[0].safetyParam) & SPEED_SYNC or
                 speed_sync_allowed(cp, int(cp.safetyConfigs[0].safetyParam))) and
                bool(cp.safetyConfigs[0].safetyParam & 1) == bool(cp.openpilotLongitudinalControl))
  except (AttributeError, ValueError, TypeError):
    return False


def apply_cooperative(cp, requested):
  if supported(cp):
    word = int(cp.safetyConfigs[0].safetyParam) & ~COOP
    cp.safetyConfigs[0].safetyParam = word | (COOP if requested else 0)
