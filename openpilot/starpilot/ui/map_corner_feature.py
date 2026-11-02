"""Shared parked map-corner preferences and local-memory management."""
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.feature_capabilities import longitudinal_supported
from openpilot.starpilot.controls.lib.map_corner.settings import MemorySettings
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

KEYS = ('MapCornerSpeed', 'LearnCornerSpeeds')
ACTIONS = frozenset('map_memory:' + x for x in ('refresh', 'undo', 'clear', 'forget'))

class MapCornerFeature:
  def __init__(self, owner):
    self.owner = owner
    self.memory = MemorySettings(lambda: not owner.authority('parked_preferences'))

  def authorized(self):
    return self.owner.authority('parked_preferences') and longitudinal_supported(self.owner.vehicle_params())

  def snapshot(self):
    if not self.memory.action('refresh'):
      return None
    return self.memory.snapshot()

  def rows(self, parked, metric):
    allowed = parked and self.authorized()
    fingerprint = self.owner.vehicle_fingerprint()
    rows = []
    for key, label, help_text in (
      ('MapCornerSpeed', 'Map corner speed', 'Uses local map geometry with visual curve limits. Requires Curve Speed Controller.'),
      ('LearnCornerSpeeds', 'Learn local corner speeds', 'Remember manual corrections after map-owned approaches. Faster learning needs repeated complete passages and visual evidence.')):
      raw, ok = read_saved(self.owner.params, key, 8)
      valid = ok and raw in (None, b'0', b'1')
      rows.append(FeatureRow(key, label, ('On' if raw == b'1' else 'Off') if valid else 'Invalid saved value', raw,
        ('Off', 'On') if valid else (), available=allowed and valid, reason=help_text,
        vehicle_fingerprint=fingerprint, default_value='Off'))
    snapshot = self.snapshot() if allowed else None
    rows.append(FeatureRow('', 'Local corner memory', self.memory.status if allowed else 'Park to inspect memory'))
    for action, label in (('refresh', 'Refresh corner memory'), ('undo', 'Undo last corner learning'), ('clear', 'Clear corner memory')):
      rows.append(FeatureRow('map_memory:' + action, label, '', snapshot, available=allowed and snapshot is not None,
                             vehicle_fingerprint=fingerprint))
    labels = self.memory.labels(metric) if snapshot is not None else {}
    for i,(label,key) in enumerate(list(labels.items())[:64]):
      rows.append(FeatureRow('map_memory:forget:'+str(i),'Forget '+label,'',snapshot,
        available=allowed,vehicle_fingerprint=fingerprint))
    return rows

  def apply(self, request):
    def allowed():
      return (self.authorized() and bool(request.vehicle_fingerprint) and
              self.owner.vehicle_fingerprint() == request.vehicle_fingerprint)
    if not allowed() or request.dependencies or request.related_source is not None or request.display_unit or request.direction:
      return False
    if request.key in KEYS:
      if request.value not in ('On', 'Off') or request.expected not in (None, b'0', b'1'):
        return False
      result = commit_exact(self.owner.params, key=request.key, max_bytes=8, expected=request.expected,
        raw=b'1' if request.value == 'On' else b'0', authorized=allowed, temp_prefix='.map-corner-')
      return result.committed and result.verified
    if (request.key not in ACTIONS and not request.key.startswith('map_memory:forget:')) or not request.confirmation:
      return False
    snapshot = self.snapshot()
    if snapshot is None or request.expected != snapshot or not allowed():
      return False
    action = request.key.split(':')[1]
    metric = read_saved(self.owner.params, 'IsMetric', 8) == (b'1', True)
    key=''
    if action=='forget':
      try:
        index=int(request.key.split(':')[2]);keys=list(self.memory.labels(metric).values())[:64]
        if not 0<=index<len(keys):return False
        key=keys[index]
      except (IndexError,ValueError):return False
    return self.memory.action(action, key, expected=snapshot)
