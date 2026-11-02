"""SP-031 Model 3 cooperative-steering preference; next-startup selection."""
from opendbc.car.tesla.coop_preferences import supported
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

KEY = 'TeslaCooperativeSteering'


class TeslaCoopFeature:
  def __init__(self, owner):
    self.owner, self.params = owner, owner.params

  def capability(self):
    cp = self.owner.vehicle_params()
    if not supported(cp):
      return None
    return (str(cp.carFingerprint), bool(cp.openpilotLongitudinalControl), int(cp.flags),
            tuple((str(c.safetyModel), int(c.safetyParam)) for c in cp.safetyConfigs))

  def rows(self, parked):
    capability = self.capability()
    if capability is None:
      return ()
    raw, readable = read_saved(self.params, KEY, 8)
    valid = readable and raw in (None, b'0', b'1')
    available = (parked and self.owner.authority('parked_preferences') and valid and
                 read_saved(self.params, 'IsReleaseBranch', 8) in ((None, True), (b'0', True)))
    return (FeatureRow(KEY, 'Cooperative steering', ('On' if raw == b'1' else 'Off') if valid else 'Invalid saved choice',
                       source=raw, choices=('Off', 'On') if valid else (), default_value='Off', available=available,
                       reason='Share steering with gentle driver input. Strong override still disengages. Applies after the next startup.',
                       capability=capability, vehicle_fingerprint=self.owner.vehicle_fingerprint()),)

  def apply(self, request):
    if (request.key != KEY or request.value not in ('On', 'Off') or not request.confirmation or
        request.expected not in (None, b'0', b'1') or request.dependencies or request.related_source is not None or
        request.display_unit or request.direction):
      return False
    def authorized():
      return (self.owner.authority('parked_preferences') and bool(request.vehicle_fingerprint) and
              self.owner.vehicle_fingerprint() == request.vehicle_fingerprint and request.capability is not None and
              self.capability() == request.capability and
              read_saved(self.params, 'IsReleaseBranch', 8) in ((None, True), (b'0', True)))
    result = commit_exact(self.params, key=KEY, max_bytes=8, expected=request.expected,
                          raw=b'1' if request.value == 'On' else b'0', authorized=authorized, temp_prefix='.tesla-coop-')
    return result.committed and result.verified
