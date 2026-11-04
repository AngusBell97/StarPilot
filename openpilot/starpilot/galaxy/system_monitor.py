"""Request-driven Linux telemetry; adapted StarPilot producer (see LICENSE)."""

import math
import os
import pwd
import shutil
import threading
import time
import uuid
from pathlib import Path


READING_FRESH_S = 5
# chestnutState.pcieLtssm when the GPU's PCIe link is up
CHESTNUT_LINK_UP = 0x78
THERMAL_STATUS_TEXT = {'ok': 'Normal', 'overheated': 'Hot · cannot engage until it cools',
                       'critical': 'Too hot · goes offroad to cool down'}
JETLINK_STATUS_TEXT = {'active': 'Running the driving model', 'ready': 'Connected · ready', 'connected': 'Connected',
                       'connecting': 'Connecting…', 'disconnected': 'Not connected', 'unavailable': 'Unavailable'}


def _bounded(value, low, high, digits=None):
  if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
    return None
  return round(value, digits) if digits is not None else value


def _temp(value):
  """A plausible temperature in °C, else None (zero is a missing reading on these sensors)."""
  value = _bounded(value, 1, 150)
  return round(value, 1) if value is not None else None


class SystemMonitor:
  def __init__(self, root=Path('/proc'), *, storage=Path('/data'), clock=time.monotonic,
               wall_clock=time.time, disk_usage=shutil.disk_usage, page_size=None, messages=None,  # noqa: TID251 - Display capture date; intervals use monotonic.
               boot_clock=lambda: time.clock_gettime(getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC)),
               params=None, feed=None, jetlink=None, clock_ticks=None):
    self.messages, self.boot_clock = messages, boot_clock
    # HardwareFeed: chestnutState, managerState and the GPU temperature peaks, read in the background
    self.params, self.feed = params, feed
    self.clock_ticks = os.sysconf('SC_CLK_TCK') if clock_ticks is None else clock_ticks
    if jetlink is not None:
      self._jetlink = jetlink
    self.root = Path(root)
    self.storage = Path(storage)
    self.clock, self.wall_clock, self.disk_usage = clock, wall_clock, disk_usage
    self.page_size = os.sysconf('SC_PAGE_SIZE') if page_size is None else page_size
    self.lock = threading.Lock()
    self.previous = {}
    self.cpu_previous = {}
    self.previous_time = None
    self.source_id = uuid.uuid4().hex

  def sample(self):
    with self.lock:
      try:
        return self._sample()
      except (OSError, ValueError, IndexError):
        # A failed sample breaks the comparison interval, including PID identity.
        self.previous, self.cpu_previous, self.previous_time = {}, {}, None
        raise OSError('System telemetry is unavailable') from None

  def _battery_voltage(self):
    if self.messages is None:
      return None
    try:
      sm = self.messages.snapshot()
      if sm is None or not sm.seen['peripheralState'] or not sm.valid['peripheralState']:
        return None
      age = self.boot_clock() - int(sm.logMonoTime['peripheralState']) / 1e9
      received_age = self.clock() - sm.recv_time['peripheralState']
      state = sm['peripheralState']
      if not (0 <= age < 5 and 0 <= received_age < 5) or state.pandaType == 0:
        return None
      volts = state.voltage / 1000
      return round(volts, 2) if math.isfinite(volts) and 0 < volts < 60 else None
    except (OSError, RuntimeError, AttributeError, KeyError, TypeError, ValueError, OverflowError):
      return None

  # ---- temperatures and external GPUs -------------------------------------

  def _device_state(self):
    if self.messages is None:
      return None
    try:
      sm = self.messages.snapshot()
      if sm is None or not sm.seen['deviceState'] or not sm.valid['deviceState']:
        return None
      age = self.boot_clock() - int(sm.logMonoTime['deviceState']) / 1e9
      received_age = self.clock() - sm.recv_time['deviceState']
      return sm['deviceState'] if 0 <= age < READING_FRESH_S and 0 <= received_age < READING_FRESH_S else None
    except (OSError, RuntimeError, AttributeError, KeyError, TypeError, ValueError, OverflowError):
      return None

  def _comma_thermals(self, device_state):
    if device_state is None:
      return None
    try:
      status = str(device_state.thermalStatus)
      return {
        'cpuTempC': _temp(max(device_state.cpuTempC, default=None)),
        'gpuTempC': _temp(max(device_state.gpuTempC, default=None)),
        'memoryTempC': _temp(device_state.memoryTempC),
        'maxTempC': _temp(device_state.maxTempC),
        'fanPercent': _bounded(device_state.fanSpeedPercentDesired, 0, 100),
        'status': status if status in THERMAL_STATUS_TEXT else None,
        'statusText': THERMAL_STATUS_TEXT.get(status),
      }
    except (AttributeError, KeyError, TypeError, ValueError):
      return None

  def _get_param(self, key):
    try:
      if self.params is None:
        if self.messages is None:
          return None
        from openpilot.common.params import Params
        self.params = Params()
      return self.params.get(key)
    except Exception:
      return None

  def _chestnut_state(self):
    try:
      return self.feed.chestnut() if self.feed is not None else None
    except Exception:
      return None

  def _with_peaks(self, gpu):
    peaks = {'hotspotC': None, 'memoryTempC': None, 'since': None}
    if self.feed is not None:
      try:
        peaks = self.feed.peaks(gpu['kind'])
      except Exception:
        pass
    gpu['hotspotPeak'], gpu['memoryPeak'], gpu['peaksSince'] = peaks['hotspotC'], peaks['memoryTempC'], peaks['since']
    gpu['peaksResettable'] = self.feed is not None
    return gpu

  def _chestnut(self, device_state):
    from openpilot.common.hardware.usb import is_chestnut_usb_id
    from openpilot.system.hardware.chestnut.status import CHESTNUT_POWERED_VOLTAGE, GPU_TEMP_LIMIT, MEMORY_TEMP_LIMIT
    usb = None
    present = False
    if device_state is not None:
      try:
        present = bool(device_state.chestnutPresent)
        usb = next((d for d in device_state.usbState.devices if is_chestnut_usb_id(d.vendorId, d.productId)), None)
      except (AttributeError, TypeError, ValueError):
        pass
    state = self._chestnut_state()
    if not present and usb is None and state is None:
      return None

    gpu = {'hotspotC': None, 'memoryTempC': None, 'loadPercent': None, 'clockMhz': None,
           'powerW': None, 'powerLimitW': None, 'fanRpm': None}
    power = {'supplyVolts': None, 'supplyAmps': None}
    powered = link_up = None
    if state is not None:
      hotspot, memory = _temp(state.tempC), _temp(state.memoryTempC)
      # modeld fills the GPU half only while the driving model runs on the Chestnut; zeros mean no reading yet
      if hotspot or memory:
        gpu = {'hotspotC': hotspot, 'memoryTempC': memory,
               'loadPercent': _bounded(state.gpuUsagePercent, 0, 100), 'clockMhz': _bounded(state.gpuClockMhz, 1, 10000),
               'powerW': _bounded(state.powerDrawW, 0, 2000, 1), 'powerLimitW': _bounded(state.powerLimitW, 1, 2000, 1),
               'fanRpm': _bounded(state.fanSpeedRpm, 0, 20000)}
      powered = not state.supplyFault and state.supplyVoltage >= CHESTNUT_POWERED_VOLTAGE
      link_up = state.pcieLtssm == CHESTNUT_LINK_UP
      power = {'supplyVolts': _bounded(state.supplyVoltage / 1000, 0, 30, 2), 'supplyAmps': _bounded(state.supplyCurrent / 1000, -30, 30, 2)}

    running = self._get_param('ChestnutActive') is True
    overheated = (gpu['hotspotC'] is not None and gpu['hotspotC'] >= GPU_TEMP_LIMIT or
                  gpu['memoryTempC'] is not None and gpu['memoryTempC'] >= MEMORY_TEMP_LIMIT or
                  self._get_param('Offroad_ChestnutOverheated') is not None)
    if overheated:
      status, text = 'danger', 'Overheated'
    elif state is None:
      status, text = 'idle', 'Connected · no readings yet'
    elif not powered:
      status, text = 'idle', 'No 12 V power (normal while the car is off)'
    elif not link_up:
      status, text = 'warning', 'Powered, but the GPU link is down'
    elif running:
      status, text = 'good', 'Running the driving model'
    else:
      status, text = 'idle', 'Connected · not running the driving model'

    usb_speed = getattr(usb, 'speedMbps', None) if usb is not None else None
    return {
      'kind': 'chestnut', 'name': 'Chestnut', 'detail': 'AMD GPU over USB',
      'status': status, 'statusText': text, 'runningModel': running,
      'readingsNote': None if gpu['hotspotC'] is not None else 'GPU temperatures appear while the driving model runs on it.',
      'hotspotLimitC': GPU_TEMP_LIMIT, 'memoryLimitC': MEMORY_TEMP_LIMIT,
      **gpu, 'fanPercent': None, **power, 'frameMs': None,
      'link': None if link_up is None else ('GPU link up' if link_up else 'GPU link down'),
      'usbSpeedMbps': _bounded(usb_speed, 1, 100000),
      'usbLinkErrors': _bounded(getattr(usb, 'linkErrorCount', None), 0, 1 << 32) if usb is not None else None,
    }

  def _jetlink(self):
    try:
      from openpilot.starpilot.models import jetlink_status
      from openpilot.starpilot.models.jetlink_adapter import link_on
      if not link_on():
        return None
      status = jetlink_status.read_status()
    except Exception:
      return None
    if status.state == 'off':
      return None
    server = jetlink_status.read_server()
    telemetry = jetlink_status.read_host_telemetry() if status.state in ('active', 'ready', 'connected') else None
    t = telemetry or {}
    mv, ma = t.get('supply_mv'), t.get('supply_ma')
    status_text = JETLINK_STATUS_TEXT.get(status.state, status.state.capitalize())
    level = 'good' if status.state == 'active' else 'warning' if status.state == 'unavailable' else 'idle'
    device = server.get('device') if isinstance(server.get('device'), str) and server.get('device') else status.device or 'Computer'
    backend = server.get('backend') if isinstance(server.get('backend'), str) else None
    return {
      'kind': 'jetlink', 'name': f'JetLink · {device}', 'detail': f'{status.device or "Computer"} over USB' + (f' · {backend}' if backend else ''),
      'status': level, 'statusText': status_text, 'runningModel': status.state == 'active',
      'readingsNote': None if telemetry else ('This device does not report its temperatures.' if status.state == 'active'
                                              else 'Readings appear while the driving model runs on it.'),
      'hotspotLimitC': None, 'memoryLimitC': None,
      'hotspotC': _temp(t.get('temp_c')), 'memoryTempC': _temp(t.get('memory_temp_c')),
      'loadPercent': _bounded(t.get('gpu_load_pct'), 0, 100), 'clockMhz': _bounded(t.get('gpu_clock_mhz'), 1, 10000),
      'powerW': _bounded(t.get('power_w'), 0, 2000, 1), 'powerLimitW': _bounded(t.get('power_limit_w'), 1, 2000, 1),
      'fanRpm': _bounded(t.get('fan_rpm'), 1, 20000), 'fanPercent': _bounded(t.get('fan_pct'), 0, 100),
      'supplyVolts': _bounded(mv / 1000, 0, 60, 2) if type(mv) in (int, float) else None,
      'supplyAmps': _bounded(ma / 1000, -30, 30, 2) if type(ma) in (int, float) else None,
      'frameMs': _bounded(status.latency_ms, 0, 10000, 1),
      'link': None, 'usbSpeedMbps': None, 'usbLinkErrors': None,
    }

  def _hardware(self):
    device_state = self._device_state()
    gpus = []
    for read in (lambda: self._chestnut(device_state), self._jetlink):
      try:
        gpu = read()
      except Exception:
        gpu = None
      if gpu is not None:
        gpus.append(self._with_peaks(gpu))
    return self._comma_thermals(device_state), gpus

  def _managed(self, rows):
    """The process manager's view: which of its processes should run and which are not running."""
    try:
      manager = self.feed.manager() if self.feed is not None else None
    except Exception:
      manager = None
    if manager is None:
      return None
    by_pid = {p['pid']: p['name'] for p in manager if p['running'] and p['pid']}
    for row in rows:
      if row['pid'] in by_pid:
        row['managedName'] = by_pid[row['pid']]
    expected = [p for p in manager if p['shouldBeRunning']]
    stopped = [{'name': p['name'], 'exitCode': p['exitCode']} for p in expected if not p['running']]
    return {'expected': len(expected), 'running': len(expected) - len(stopped), 'stopped': stopped}

  def _sample(self):
    now = self.clock()
    interval = now - self.previous_time if self.previous_time is not None else None
    comparable = interval is not None and 0 < interval <= 10
    cpu_now, cores = {}, []
    overall = capacity = None
    for line in (self.root / 'stat').read_text().splitlines():
      values = line.split()
      if not values or not (values[0] == 'cpu' or values[0].startswith('cpu') and values[0][3:].isdigit()):
        continue
      ticks = [int(value) for value in values[1:9]]
      if len(ticks) < 5 or any(value < 0 for value in ticks):
        raise ValueError('Invalid CPU counters')
      pair = (sum(ticks), ticks[3] + ticks[4])
      key = values[0]
      cpu_now[key] = pair
      old = self.cpu_previous.get(key) if comparable else None
      percent = None
      if old and pair[0] > old[0] and 0 <= pair[1] - old[1] <= pair[0] - old[0]:
        percent = round(100 * (1 - (pair[1] - old[1]) / (pair[0] - old[0])), 1)
        if key == 'cpu':
          capacity = pair[0] - old[0]
      if key == 'cpu':
        overall = percent
      else:
        cores.append({'name': key, 'percent': percent})
    if 'cpu' not in cpu_now:
      raise ValueError('Missing CPU counters')

    rows, process_ticks = [], {}
    for path in sorted(self.root.iterdir()):
      if not path.name.isdigit() or int(path.name) <= 0:
        continue
      try:
        raw = (path / 'stat').read_text()
        end = raw.rindex(')')
        fields = raw[end + 2:].split()
        identity = (int(path.name), int(fields[19]))
        current = int(fields[11]) + int(fields[12])
        if current < 0:
          continue
        args = [part for part in (path / 'cmdline').read_bytes().decode(errors='replace').split('\0') if part]
        name = raw[raw.index('(') + 1:end]
        if args:
          name = args[0]
          if 'python' in Path(name).name and len(args) > 1:
            name = args[2] if args[1] == '-m' and len(args) > 2 else args[1] if not args[1].startswith('-') else Path(name).name
        uid = path.stat().st_uid
        try:
          user = pwd.getpwuid(uid).pw_name
        except KeyError:
          user = str(uid)
        cpu = None
        previous = self.previous.get(identity) if comparable else None
        if capacity and previous is not None and current >= previous:
          cpu = round(min(100, (current - previous) / capacity * 100), 1)
        rows.append({'pid': identity[0], 'ppid': int(fields[1]), 'name': name.removeprefix('/data/openpilot/'), 'user': user,
                     'kernel': bool(int(fields[6]) & 0x00200000), 'state': fields[0], 'cpu': cpu,
                     'instanceId': f'{self.source_id}:{identity[0]}:{identity[1]}', 'startTicks': identity[1],
                     'memoryMiB': round(max(0, int(fields[21])) * self.page_size / 1048576, 1)})
        process_ticks[identity] = current
      except (OSError, ValueError, IndexError):
        # Processes may disappear between reading stat, cmdline and ownership.
        continue

    memory = {line.split(':')[0]: int(line.split()[1])
              for line in (self.root / 'meminfo').read_text().splitlines() if ':' in line}
    total = memory.get('MemTotal', 0)
    available = memory.get('MemAvailable')
    if total <= 0:
      raise ValueError('Missing total memory')
    # MemFree excludes reclaimable cache and cannot stand in for MemAvailable.
    if available is not None and not 0 <= available <= total:
      available = None
    used = total - available if available is not None else None
    storage = {'usedGiB': None, 'totalGiB': None}
    try:
      disk = self.disk_usage(self.storage)
      storage = {'usedGiB': round(disk.used / 1073741824, 1), 'totalGiB': round(disk.total / 1073741824, 1)}
    except OSError:
      pass
    uptime = float((self.root / 'uptime').read_text().split()[0])
    # openpilot's own processes: the process manager and everything under it
    parents = {row['pid']: row['ppid'] for row in rows}
    managers = {row['pid'] for row in rows if row['name'].endswith('manager.py') or row['name'] == 'system.manager.manager'}
    for row in rows:
      start = row.pop('startTicks')
      age = uptime - start / self.clock_ticks if self.clock_ticks else None
      row['ageSeconds'] = round(age) if age is not None and math.isfinite(age) and age >= 0 else None
      pid, hops = row['pid'], 0
      while pid not in managers and pid in parents and hops < 64:
        pid, hops = parents[pid], hops + 1
      row['openpilot'] = pid in managers and not row['kernel']
    sampled_at = self.wall_clock()
    if not math.isfinite(uptime) or uptime < 0 or not math.isfinite(now) or not math.isfinite(sampled_at) or sampled_at <= 0:
      raise ValueError('Invalid sample time')
    result = {
      'schemaVersion': 1, 'mode': 'local-runtime', 'source': 'Local Linux system telemetry',
      'sampledAt': sampled_at, 'sampleMono': now, 'sourceId': self.source_id,
      'intervalSeconds': interval if comparable else None,
      'cpuPercent': overall, 'cores': cores,
      'memory': {'totalMiB': round(total / 1024, 1),
                 'usedMiB': round(used / 1024, 1) if used is not None else None,
                 'availableMiB': round(available / 1024, 1) if available is not None else None,
                 'percent': round(100 * used / total, 1) if used is not None else None},
      'batteryVoltage': self._battery_voltage(),
      'storage': storage, 'uptimeSeconds': uptime, 'processCount': len(rows), 'processes': rows,
    }
    comma, gpus = self._hardware()
    result['commaThermals'] = comma
    result['externalGpus'] = gpus
    result['managed'] = self._managed(rows)
    self.previous, self.cpu_previous, self.previous_time = process_ticks, cpu_now, now
    return result
