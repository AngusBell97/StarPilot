"""Exercise the real Linux reader through the real loopback HTTP adapter."""

import http.client
import json
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

from openpilot.starpilot.galaxy.access import GalaxyAccessOwner
from openpilot.starpilot.galaxy.server import make_server
from openpilot.starpilot.galaxy.system_monitor import SystemMonitor


class SystemMonitorRuntimeTest(unittest.TestCase):
  def setUp(self):
    self.directory = tempfile.TemporaryDirectory()
    self.addCleanup(self.directory.cleanup)
    self.root = Path(self.directory.name)
    self.time = 10.0
    self.monitor = SystemMonitor(self.root, clock=lambda: self.time, wall_clock=lambda: 1_000_000 + self.time,
                                 disk_usage=lambda _: SimpleNamespace(used=2 ** 30, total=4 * 2 ** 30), page_size=4096)
    self.write_snapshot(100, 100, 10)

  def write_snapshot(self, user, idle, ticks, *, start=1000):
    (self.root / 'stat').write_text(f'cpu {user} 0 0 {idle} 0 0 0 0\ncpu0 {user} 0 0 {idle} 0 0 0 0\n')
    (self.root / 'meminfo').write_text('MemTotal: 4096000 kB\nMemAvailable: 1024000 kB\n')
    (self.root / 'uptime').write_text('300 0\n')
    process = self.root / '42'
    process.mkdir(exist_ok=True)
    fields = ['0'] * 22
    fields[0], fields[11], fields[19], fields[21] = 'S', str(ticks), str(start), '256'
    (process / 'stat').write_text(f'42 (worker (test)) {" ".join(fields)}')
    (process / 'cmdline').write_bytes(b'python\0-m\0openpilot.selfdrive.controls.controlsd\0--private-argument\0')

  def test_cpu_capacity_pid_reuse_and_monotonic_reset(self):
    first = self.monitor.sample()
    self.assertIsNone(first['cpuPercent'])
    self.assertIsNone(first['processes'][0]['cpu'])
    self.assertEqual(first['processes'][0]['name'], 'openpilot.selfdrive.controls.controlsd')
    self.assertNotIn('private-argument', json.dumps(first))
    self.time += 2
    self.write_snapshot(160, 140, 30)
    second = self.monitor.sample()
    self.assertEqual(second['cpuPercent'], 60)
    self.assertEqual(second['processes'][0]['cpu'], 20)
    self.assertEqual(second['processes'][0]['memoryMiB'], 1)
    self.time += 2
    self.write_snapshot(220, 180, 40, start=2000)
    self.assertIsNone(self.monitor.sample()['processes'][0]['cpu'])
    self.time = 1
    self.write_snapshot(280, 220, 60, start=2000)
    self.assertIsNone(self.monitor.sample()['cpuPercent'])

  def test_unavailable_fields_and_failed_interval_are_not_zero_or_stale(self):
    self.monitor.sample()
    (self.root / 'meminfo').write_text('MemTotal: 4096000 kB\nMemFree: 1 kB\n')
    (self.root / '53').mkdir()  # Process disappears during enumeration.
    self.time += 2
    sample = self.monitor.sample()
    self.assertIsNone(sample['memory']['usedMiB'])
    self.assertIsNone(sample['memory']['percent'])
    self.assertEqual(sample['processCount'], 1)
    self.assertNotIn('vitals', sample)
    (self.root / 'stat').unlink()
    with self.assertRaises(OSError):
      self.monitor.sample()
    self.time += 2
    self.write_snapshot(300, 200, 50)
    self.assertIsNone(self.monitor.sample()['cpuPercent'])

  def test_real_http_source_origin_static_paths_and_read_only_boundary(self):
    owner = GalaxyAccessOwner(self.root / 'access')
    self.assertTrue(owner.configure('test-password', lambda: True))
    server = make_server(port=0, monitor=self.monitor, owner=owner)
    worker = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.01}, daemon=True)
    worker.start()
    try:
      def request(path, *, method='GET', headers=None, body=None):
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2)
        try:
          connection.request(method, path, body=body, headers=headers or {})
          response = connection.getresponse()
          return response.status, response.read(), dict(response.getheaders())
        finally:
          connection.close()

      self.assertEqual(request('/api/system/monitor')[0], 401)
      origin = f'http://127.0.0.1:{server.server_port}'
      status, body, login_headers = request('/api/auth/login', method='POST', headers={
        'Content-Type': 'application/json', 'Origin': origin}, body='{"password":"test-password"}')
      self.assertEqual(status, 200)
      cookie = login_headers['Set-Cookie'].split(';', 1)[0]
      auth = {'Cookie': cookie}
      status, body, headers = request('/api/system/monitor', headers=auth)
      self.assertEqual(status, 200)
      self.assertEqual(json.loads(body)['mode'], 'local-runtime')
      self.assertEqual(headers['Cache-Control'], 'no-store')
      self.assertNotIn('Access-Control-Allow-Origin', headers)
      self.assertEqual(json.loads(request('/data/runtime.json')[1])['monitor'], 'local')
      self.assertIn(b'galaxy-app', request('/')[1])
      for path in ('/../server.py', '/%2e%2e/server.py'):
        self.assertEqual(request(path)[0], 404)
      for path in ('/api/params', '/api/vehicle'):
        self.assertEqual(request(path, headers=auth)[0], 404)
      self.assertEqual(request('/api/system/monitor', headers={'Host': 'other.example'})[0], 403)
      self.assertEqual(request('/api/system/monitor', headers={'Origin': 'https://other.example'})[0], 403)
      self.assertEqual(request('/api/system/monitor', method='POST', headers={'Origin': origin})[0], 405)
      (self.root / 'stat').unlink()
      status, body, _ = request('/api/system/monitor', headers=auth)
      self.assertEqual(status, 503)
      self.assertNotIn('sampledAt', json.loads(body))
    finally:
      server.shutdown()
      worker.join(timeout=2)
      server.server_close()


class BatteryVoltageTest(unittest.TestCase):
  def test_peripheral_millivolts_freshness_and_unavailable(self):
    state = SimpleNamespace(pandaType=1, voltage=12640)
    class Snapshot(SimpleNamespace):
      def __getitem__(self, key):
        return state
    sm = Snapshot(seen={'peripheralState':True}, valid={'peripheralState':True},
                  logMonoTime={'peripheralState':9_000_000_000}, recv_time={'peripheralState':9.})
    source = SimpleNamespace(snapshot=lambda:sm)
    monitor = SystemMonitor(messages=source, clock=lambda:10., boot_clock=lambda:10.)
    self.assertEqual(monitor._battery_voltage(),12.64)
    state.voltage=14420
    self.assertEqual(monitor._battery_voltage(),14.42)
    for voltage in (0, -1, 1000000):
      state.voltage=voltage
      self.assertIsNone(monitor._battery_voltage())
    state.voltage=12640
    for field,bad,good in (('seen',False,True),('valid',False,True),
                           ('logMonoTime',5_000_000_000,9_000_000_000),
                           ('logMonoTime',11_000_000_000,9_000_000_000),
                           ('recv_time',5.,9.),('recv_time',11.,9.)):
      getattr(sm,field)['peripheralState']=bad
      self.assertIsNone(monitor._battery_voltage())
      getattr(sm,field)['peripheralState']=good
    state.pandaType=0
    self.assertIsNone(monitor._battery_voltage())
    state.pandaType=1
    source.snapshot=lambda:None
    self.assertIsNone(monitor._battery_voltage())
    self.assertIsNone(SystemMonitor()._battery_voltage())


class HardwareReadingsTest(unittest.TestCase):
  """Comma temperatures and external GPUs (Chestnut, JetLink) on the System Monitor."""

  def setUp(self):
    self.now = 100.
    self.device = SimpleNamespace(
      thermalStatus='ok', cpuTempC=[55.5, 61.4], gpuTempC=[52.0], memoryTempC=50.2, maxTempC=61.4, fanSpeedPercentDesired=35,
      chestnutPresent=True,
      usbState=SimpleNamespace(devices=[SimpleNamespace(vendorId=0xADD1, productId=0x0001, speedMbps=10000, linkErrorCount=0)]))
    self.chestnut = SimpleNamespace(tempC=78.0, memoryTempC=88.0, powerDrawW=71.44, powerLimitW=120., gpuUsagePercent=64,
                                    gpuClockMhz=2410, fanSpeedRpm=1850, pcieLtssm=0x78, supplyVoltage=12100,
                                    supplyCurrent=6420, supplyFault=False)
    test = self

    class Snapshot(SimpleNamespace):
      def __getitem__(self, key):
        return test.device

    class ChestnutMessages(SimpleNamespace):
      def update(self, timeout):
        pass

      def __getitem__(self, key):
        return test.chestnut

    self.snapshot = Snapshot(seen={'deviceState': True}, valid={'deviceState': True},
                             logMonoTime={'deviceState': 99_000_000_000}, recv_time={'deviceState': 99.})
    self.chestnut_messages = ChestnutMessages(seen={'chestnutState': True}, valid={'chestnutState': True},
                                              recv_time={'chestnutState': 99.5})
    self.params = {'ChestnutActive': True}
    self.monitor = SystemMonitor(messages=SimpleNamespace(snapshot=lambda: self.snapshot), clock=lambda: self.now,
                                 boot_clock=lambda: self.now, params=SimpleNamespace(get=lambda key: self.params.get(key)),
                                 chestnut_messages=self.chestnut_messages, jetlink=lambda: None)

  def test_comma_thermals(self):
    comma, _ = self.monitor._hardware()
    self.assertEqual(comma, {'cpuTempC': 61.4, 'gpuTempC': 52.0, 'memoryTempC': 50.2, 'maxTempC': 61.4,
                             'fanPercent': 35, 'status': 'ok', 'statusText': 'Normal'})
    self.device.thermalStatus = 'overheated'
    self.assertIn('cannot engage', self.monitor._hardware()[0]['statusText'])
    self.snapshot.recv_time['deviceState'] = 90.
    self.assertIsNone(self.monitor._hardware()[0])

  def test_chestnut_running(self):
    _, gpus = self.monitor._hardware()
    self.assertEqual(len(gpus), 1)
    gpu = gpus[0]
    self.assertEqual((gpu['kind'], gpu['status'], gpu['statusText']), ('chestnut', 'good', 'Running the driving model'))
    self.assertEqual((gpu['hotspotC'], gpu['memoryTempC'], gpu['hotspotLimitC'], gpu['memoryLimitC']), (78.0, 88.0, 100., 95.))
    self.assertEqual((gpu['loadPercent'], gpu['clockMhz'], gpu['powerW'], gpu['powerLimitW'], gpu['fanRpm']), (64, 2410, 71.4, 120., 1850))
    self.assertEqual((gpu['supplyVolts'], gpu['supplyAmps'], gpu['link'], gpu['usbSpeedMbps']), (12.1, 6.42, 'GPU link up', 10000))
    self.assertIsNone(gpu['readingsNote'])

  def test_chestnut_states(self):
    def status():
      gpu = self.monitor._hardware()[1][0]
      return gpu['status'], gpu['statusText']
    self.chestnut.tempC = 101.
    self.assertEqual(status(), ('danger', 'Overheated'))
    self.chestnut.tempC = 78.
    self.params['Offroad_ChestnutOverheated'] = {'text': 'hot'}
    self.assertEqual(status()[0], 'danger')
    del self.params['Offroad_ChestnutOverheated']
    self.params['ChestnutActive'] = False
    self.assertEqual(status(), ('idle', 'Connected · not running the driving model'))
    self.chestnut.pcieLtssm = 0x01
    self.assertEqual(status()[0], 'warning')
    self.chestnut.supplyVoltage = 0
    self.assertEqual(status()[0], 'idle')
    self.assertIn('12 V', status()[1])
    self.chestnut_messages.recv_time['chestnutState'] = 90.
    self.assertEqual(status(), ('idle', 'Connected · no readings yet'))

  def test_chestnut_without_gpu_readings(self):
    # modeld leaves the GPU half at zero while the driving model is not on the Chestnut
    for field in ('tempC', 'memoryTempC', 'powerDrawW', 'gpuUsagePercent', 'gpuClockMhz', 'fanSpeedRpm'):
      setattr(self.chestnut, field, 0)
    gpu = self.monitor._hardware()[1][0]
    self.assertIsNone(gpu['hotspotC'])
    self.assertIsNone(gpu['loadPercent'])
    self.assertIn('appear', gpu['readingsNote'])
    self.assertEqual(gpu['supplyVolts'], 12.1)

  def test_no_chestnut(self):
    self.device.chestnutPresent = False
    self.device.usbState.devices = []
    self.chestnut_messages.seen['chestnutState'] = False
    self.assertEqual(self.monitor._hardware()[1], [])
    self.assertEqual(SystemMonitor(jetlink=lambda: None)._hardware(), (None, []))

  def test_sample_carries_hardware(self):
    with tempfile.TemporaryDirectory() as directory:
      root = Path(directory)
      (root / 'stat').write_text('cpu 1 0 0 1 0 0 0 0\n')
      (root / 'meminfo').write_text('MemTotal: 4096000 kB\nMemAvailable: 1024000 kB\n')
      (root / 'uptime').write_text('300 0\n')
      self.monitor.root = root
      sample = self.monitor.sample()
    self.assertEqual(sample['commaThermals']['maxTempC'], 61.4)
    self.assertEqual(sample['externalGpus'][0]['kind'], 'chestnut')
    self.assertNotIn('vitals', sample)

  def test_jetlink_host(self):
    from unittest.mock import patch
    from openpilot.starpilot.models import jetlink_status
    status = jetlink_status.JetlinkStatus('Computer', 'active', 31.25, 'connected')
    telemetry = {'temp_c': 64.04, 'memory_temp_c': 58.0, 'power_w': 14.2, 'power_limit_w': 25, 'gpu_load_pct': 71,
                 'gpu_clock_mhz': 1020, 'fan_rpm': 2900, 'supply_mv': 19100, 'supply_ma': 900}
    monitor = SystemMonitor()
    with patch('openpilot.starpilot.models.jetlink_adapter.link_on', return_value=True), \
         patch.object(jetlink_status, 'read_status', return_value=status), \
         patch.object(jetlink_status, 'read_server', return_value={'device': 'Orin Nano', 'backend': 'trt'}), \
         patch.object(jetlink_status, 'read_host_telemetry', return_value=telemetry):
      gpu = monitor._jetlink()
      self.assertEqual((gpu['name'], gpu['detail'], gpu['status']), ('JetLink · Orin Nano', 'Computer over USB · trt', 'good'))
      self.assertEqual((gpu['hotspotC'], gpu['memoryTempC'], gpu['loadPercent'], gpu['powerW'], gpu['fanRpm']), (64.0, 58.0, 71, 14.2, 2900))
      self.assertEqual((gpu['supplyVolts'], gpu['supplyAmps'], gpu['frameMs']), (19.1, 0.9, 31.2))
      self.assertIsNone(gpu['readingsNote'])
      with patch.object(jetlink_status, 'read_host_telemetry', return_value=None):
        self.assertIn('does not report', monitor._jetlink()['readingsNote'])
    with patch('openpilot.starpilot.models.jetlink_adapter.link_on', return_value=False):
      self.assertIsNone(monitor._jetlink())


class JetlinkTelemetryRecordTest(unittest.TestCase):
  def test_modeld_record_carries_host_telemetry(self):
    from openpilot.starpilot.models import jetlink_status
    with tempfile.TemporaryDirectory() as directory:
      path = Path(directory) / 'latency.json'
      client = SimpleNamespace(last_timings=(1000, 0, 30000), last_state={'temp_c': 60.0}, dead=False)
      jetlink_status._write_latency(SimpleNamespace(client=client), str(path))
      self.assertEqual(jetlink_status.read_host_telemetry(str(path)), {'temp_c': 60.0})
      self.assertEqual(jetlink_status.read_latency(str(path)), 30.)
      client.dead = True
      jetlink_status._write_latency(SimpleNamespace(client=client), str(path))
      self.assertIsNone(jetlink_status.read_host_telemetry(str(path)))
      jetlink_status._write_latency(SimpleNamespace(), str(path))
      self.assertIsNone(jetlink_status.read_host_telemetry(str(path)))
      self.assertEqual(jetlink_status.read_server(str(Path(directory) / 'missing.json')), {})
