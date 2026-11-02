"""SP-011 MAX/SET warning, separate from the sign's confirmation pulses."""
import time
import pyray as rl
from openpilot.starpilot.common.slc_warning import MaxSpeedWarning

class WarningVisual:
  def __init__(self):self.warning=MaxSpeedWarning();self.drive=None
  def update(self,state):
    if self.drive!=state.drive_frame:
      self.drive=state.drive_frame;self.warning=MaxSpeedWarning()
    now=state.observed_ns or time.monotonic_ns();s=state.speed_limit
    return self.warning.update(now/1e9,s.warning_id,s.warning_issued_ns/1e9,
      s.warning_pin,s.warning_pin_issued_ns/1e9,
      fresh=0<s.warning_frame_ns<=now<=s.warning_frame_ns+100_000_000,
      engaged=state.engaged and state.longitudinal_active)
  @staticmethod
  def color(base,pulse):return rl.Color(255,round(45*pulse),round(45*pulse),round(base.a*pulse))
