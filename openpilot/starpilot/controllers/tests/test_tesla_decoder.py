import unittest
from openpilot.starpilot.controllers.tesla_controls import TeslaInputDecoder,INPUTS

def wheel(**values):
  bits = 1
  shifts = {'left_wheel_left':14,'left_wheel_right':3,'left_wheel_press':5,
            'right_wheel_left':8,'right_wheel_right':10,'right_wheel_press':12}
  for name, shift in shifts.items():
    bits |= values.get(name, 1) << shift
  for name, shift in [('left_scroll',16),('right_scroll',24)]:
    bits |= (values.get(name,0) & 63) << shift
  return bits.to_bytes(8,'little')

class DecoderTests(unittest.TestCase):
  def setUp(self):
    self.d = TeslaInputDecoder(bus=1)
    self.t = 1.0
  def feed(self, data=None, address=0x3c2, bus=1, dt=.1):
    self.t += dt
    return self.d.feed(address, data or wheel(), bus, self.t)
  def test_defaults_and_all_ten_wheel_entries(self):
    self.assertEqual(len([k for k in INPUTS if '_wheel_' in k]), 10)
    self.assertEqual(self.feed(wheel(right_wheel_right=2)), [])
    self.assertEqual(self.feed(), [])
    self.assertEqual(self.feed(wheel(right_wheel_right=2)), [])
    self.assertEqual(self.feed(wheel(right_wheel_right=2)), [])
    self.assertEqual(self.feed(), ['right_wheel_right'])
    self.assertEqual(self.feed(), [])
  def test_scroll_signed_magnitude_and_direction_change(self):
    self.feed()
    self.feed(wheel(right_scroll=-2))
    self.assertEqual(self.feed(), ['right_wheel_down'])
    self.feed(wheel(right_scroll=1))
    self.feed(wheel(right_scroll=-1))
    self.assertEqual(self.feed(), [])
    self.feed(wheel(left_scroll=1))
    self.assertEqual(self.feed(), ['left_wheel_up'])
  def test_mux_bus_length_fault_and_simultaneous(self):
    self.feed()
    for data, bus in [(wheel(right_wheel_right=2),0),(bytes(8),1),(bytes(7),1)]:
      self.assertEqual(self.feed(data,bus=bus), [])
    self.feed()
    self.feed(wheel(right_wheel_press=3))
    self.feed(wheel(right_wheel_right=2))
    self.assertEqual(self.feed(), [])
    self.feed(wheel(left_wheel_left=2,right_wheel_right=2))
    self.assertEqual(self.feed(), [])
  def test_gap_reset_and_explicit_reset(self):
    self.feed(); self.feed(wheel(right_wheel_right=2))
    self.assertEqual(self.feed(dt=1), [])
    self.feed(wheel(right_wheel_right=2)); self.d.reset()
    self.assertEqual(self.feed(), [])
  def test_invalid_mux_discards_pending_but_body_mux_does_not(self):
    for mux in (2,3):
      self.feed();self.feed(wheel(right_wheel_press=2))
      self.feed(bytes([mux])+bytes(7));self.assertEqual(self.feed(),[])
    self.feed(wheel(right_wheel_press=2));self.feed(bytes(8))
    self.assertEqual(self.feed(),['right_wheel_press'])
  def stalk(self, value, counter, address=0x229, length=3, shift=12):
    return self.feed(((counter << 8) | (value << shift)).to_bytes(length,'little'), address)
  def test_right_stalk_stage_once_and_counter_gap(self):
    self.stalk(0,0); self.stalk(1,1); self.stalk(2,2)
    self.assertEqual(self.stalk(0,3), ['right_stalk_up_full'])
    self.stalk(3,4); self.stalk(4,6)
    self.assertEqual(self.stalk(0,7), [])
  def test_left_stalk_variants(self):
    for length, half, full in [(3,3,4),(4,6,8)]:
      self.d.reset()
      self.stalk(0,0,0x249,length,16); self.stalk(half,1,0x249,length,16)
      self.stalk(full,2,0x249,length,16)
      self.assertEqual(self.stalk(0,3,0x249,length,16), ['indicator_left_full'])
  def test_captured_park_event_reuses_periodic_counter(self):
    self.feed(bytes.fromhex('8c0b00'),0x229)
    self.feed(bytes.fromhex('490c00'),0x229)
    self.feed(bytes.fromhex('c60d01'),0x229,dt=0)
    self.assertEqual(self.feed(bytes.fromhex('2f0d00'),0x229), ['park_press'])
  def test_every_wheel_button_and_captured_frame(self):
    for name in [n for n in INPUTS if n.endswith(('_left','_right','_press')) and '_wheel_' in n]:
      self.feed(); self.feed(wheel(**{name:2}))
      self.assertEqual(self.feed(), [name])
    self.feed(bytes.fromhex('2959000000000040'))
    self.assertEqual(self.feed(bytes.fromhex('2955000000000040')), ['right_wheel_right'])
