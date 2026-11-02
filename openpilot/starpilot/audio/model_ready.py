"""Optional GPU completion notification; never controls model admission."""
from openpilot.starpilot.audio.alert_volume import read_volume

KEY = "GpuModelReadyVolume"


def preference(params):
  saved = read_volume(params, KEY)
  return saved.raw, saved.valid, saved.valid and saved.value > 0


class ModelReadyChime:
  def __init__(self):
    self.armed = False
    self.pending_until = None

  def update(self, *, onroad, active, loading, output_ready, enabled, now):
    if not onroad or loading or active is False:
      self.armed = True
      self.pending_until = None
    elif active is True and output_ready and self.armed:
      self.armed = False
      self.pending_until = now + 5.0 if enabled else None
    if not enabled or (self.pending_until is not None and now >= self.pending_until):
      self.pending_until = None
    # Missing output never rearms the chime. Require fresh output at playback.
    return bool(onroad and not loading and active is True and output_ready and self.pending_until is not None)

  def consume(self):
    self.pending_until = None
