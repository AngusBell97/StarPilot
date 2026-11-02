"""Real native safety with 7.0 brake/steering encoding and cooperative override."""
import unittest
from opendbc.safety.tests import test_tesla_screen_button as screen


class TestCooperativeScreen(screen.TestTeslaScreenButton):
  def start(self, word):
    super().start(word | 256)

  def test_cooperative_torque_disengages_without_removing_existing_checks(self):
    for torque in (-5.01, 5.01):
      self.start(512)
      self.arm()
      frame = self.packer.make_can_msg_safety('EPAS3S_sysStatus', 0,
        {'EPAS3S_internalSAS': 0, 'EPAS3S_eacStatus': 1, 'EPAS3S_handsOnLevel': 0,
         'EPAS3S_torsionBarTorque': torque})
      self.assertTrue(self.lib.safety_rx_hook(frame))
      self.assertEqual(self.permission(), 0)

  def test_light_torque_preserves_screen_arm(self):
    self.start(512)
    self.arm()
    frame = self.packer.make_can_msg_safety('EPAS3S_sysStatus', 0,
      {'EPAS3S_internalSAS': 0, 'EPAS3S_eacStatus': 1, 'EPAS3S_handsOnLevel': 0,
       'EPAS3S_torsionBarTorque': 1.2})
    self.assertTrue(self.lib.safety_rx_hook(frame))
    self.assertEqual(self.permission(), 1)
