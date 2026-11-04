"""Galaxy's always-on hardware reader for the System Monitor.

Once a second, whether or not anyone has the page open, it keeps a copy of the latest chestnutState and managerState
and logs the highest external GPU temperatures (hotspot and memory) to a small file, so the page can show the hottest
reading of any drive. Reading is cheap (a conflated socket each, plus one small file for JetLink).
"""

import json
import math
import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace

PEAKS_PATH = Path('/data/starpilot/galaxy/gpu-peaks.json')
PERIOD_S = 1.0
FRESH_S = 5.0
# A new record is written at most this often (and when the feed stops), so a long hot drive costs a few writes
SAVE_EVERY_S = 30.0
PEAK_FIELDS = ('hotspotC', 'memoryTempC')
GPU_KINDS = ('chestnut', 'jetlink')
CHESTNUT_FIELDS = ('tempC', 'memoryTempC', 'powerDrawW', 'powerLimitW', 'gpuUsagePercent', 'gpuClockMhz', 'fanSpeedRpm',
                   'pcieLtssm', 'supplyVoltage', 'supplyCurrent', 'supplyFault')


def _temperature(value):
  if type(value) not in (int, float) or not math.isfinite(value) or not 1 <= value <= 150:
    return None
  return round(float(value), 1)


def _jetlink_telemetry():
  from openpilot.starpilot.models.jetlink_adapter import link_on
  if not link_on():
    return None
  from openpilot.starpilot.models.jetlink_status import read_host_telemetry
  return read_host_telemetry()


class HardwareFeed:
  def __init__(self, path=PEAKS_PATH, *, messages=None, jetlink_telemetry=_jetlink_telemetry,
               clock=time.monotonic, wall_clock=time.time, period=PERIOD_S):  # noqa: TID251 - peak dates are shown to people
    self.path = Path(path)
    self.messages = messages
    self.jetlink_telemetry = jetlink_telemetry
    self.clock, self.wall_clock, self.period = clock, wall_clock, period
    self.lock = threading.Lock()
    self.stop_event = threading.Event()
    self.thread = None
    self._chestnut = None   # (received monotonic, SimpleNamespace)
    self._manager = None    # (received monotonic, list of dicts)
    self._peaks = self._load()
    self._dirty = False
    self._saved_at = -math.inf

  # ---- lifecycle -------------------------------------------------------------

  def start(self):
    if self.thread is not None:
      return self
    if self.messages is None:
      from openpilot.cereal import messaging
      self.messages = messaging.SubMaster(['chestnutState', 'managerState'])
    def run():
      while not self.stop_event.is_set():
        try:
          self.poll()
        except Exception:
          pass  # a bad read costs one tick; the page shows what it last had
        self.stop_event.wait(self.period)
    self.thread = threading.Thread(target=run, name='galaxy-hardware', daemon=True)
    self.thread.start()
    return self

  def close(self):
    self.stop_event.set()
    if self.thread is not None and self.thread.ident is not None:
      self.thread.join(timeout=2)
    self._save(force=True)

  # ---- one tick --------------------------------------------------------------

  def poll(self):
    now = self.clock()
    sm = self.messages
    sm.update(0)
    chestnut = manager = None
    if sm.seen['chestnutState'] and sm.valid['chestnutState'] and 0 <= now - sm.recv_time['chestnutState'] < FRESH_S:
      state = sm['chestnutState']
      chestnut = SimpleNamespace(**{field: getattr(state, field) for field in CHESTNUT_FIELDS})
    if sm.seen['managerState'] and 0 <= now - sm.recv_time['managerState'] < FRESH_S:
      manager = [{'name': p.name, 'pid': p.pid, 'running': p.running, 'shouldBeRunning': p.shouldBeRunning,
                  'exitCode': p.exitCode} for p in sm['managerState'].processes]
    with self.lock:
      if chestnut is not None:
        self._chestnut = (now, chestnut)
      if manager is not None:
        self._manager = (now, manager)
    if chestnut is not None:
      self.record('chestnut', chestnut.tempC, chestnut.memoryTempC)
    try:
      telemetry = self.jetlink_telemetry() if self.jetlink_telemetry is not None else None
    except Exception:
      telemetry = None
    if telemetry:
      self.record('jetlink', telemetry.get('temp_c'), telemetry.get('memory_temp_c'))
    self._save()

  # ---- readers (any thread) --------------------------------------------------

  def chestnut(self):
    with self.lock:
      if self._chestnut is None or not 0 <= self.clock() - self._chestnut[0] < FRESH_S:
        return None
      return self._chestnut[1]

  def manager(self):
    with self.lock:
      if self._manager is None or not 0 <= self.clock() - self._manager[0] < FRESH_S:
        return None
      return [dict(p) for p in self._manager[1]]

  def peaks(self, kind):
    with self.lock:
      entry = self._peaks.get(kind) or {}
      return {field: dict(entry[field]) if isinstance(entry.get(field), dict) else None for field in PEAK_FIELDS} | \
             {'since': entry.get('since')}

  # ---- the peak log ----------------------------------------------------------

  def record(self, kind, hotspot, memory):
    readings = {'hotspotC': _temperature(hotspot), 'memoryTempC': _temperature(memory)}
    if all(value is None for value in readings.values()):
      return
    at = self.wall_clock()
    with self.lock:
      entry = self._peaks.setdefault(kind, {'since': at})
      for field, value in readings.items():
        if value is not None and (entry.get(field) is None or value > entry[field]['value']):
          entry[field] = {'value': value, 'at': at}
          self._dirty = True

  def reset(self, kind):
    if kind not in GPU_KINDS:
      raise ValueError('Unknown GPU')
    with self.lock:
      self._peaks[kind] = {'since': self.wall_clock()}
      self._dirty = True
    self._save(force=True)

  def _load(self):
    try:
      record = json.loads(self.path.read_text())
    except (OSError, ValueError):
      return {}
    peaks = {}
    if isinstance(record, dict):
      for kind in GPU_KINDS:
        entry = record.get(kind)
        if not isinstance(entry, dict):
          continue
        clean = {'since': entry.get('since') if type(entry.get('since')) in (int, float) else None}
        for field in PEAK_FIELDS:
          item = entry.get(field)
          if isinstance(item, dict) and _temperature(item.get('value')) is not None and type(item.get('at')) in (int, float):
            clean[field] = {'value': _temperature(item['value']), 'at': item['at']}
        peaks[kind] = clean
    return peaks

  def _save(self, force=False):
    with self.lock:
      if not self._dirty or not force and self.clock() - self._saved_at < SAVE_EVERY_S:
        return
      data = json.dumps(self._peaks)
      self._dirty = False
      self._saved_at = self.clock()
    try:
      self.path.parent.mkdir(parents=True, exist_ok=True)
      tmp = self.path.with_name(f'.{self.path.name}.{os.getpid()}')
      tmp.write_text(data)
      os.replace(tmp, self.path)
    except OSError:
      with self.lock:
        self._dirty = True
