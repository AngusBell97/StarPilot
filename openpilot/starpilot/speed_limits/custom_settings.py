"""SP-011 saved values, retaining metres/seconds semantics."""
from dataclasses import dataclass
import math

SPECS = {
 'SLCLowerLimitTiming': (-5., 5., 0.),
 'SLCHigherLimitDistance': (0., 500., 50.),
 'SLCWarningLeadTime': (0., 10., 2.),
}
BOOLS = ('SLCTransitionOffsets', 'SLCRememberManualOffset', 'SLCReengageSpeed')

@dataclass(frozen=True)
class CustomSettings:
  lower_timing: float = 0.
  higher_distance: float = 50.
  warning_lead: float = 2.
  transition_offsets: bool = False  # Preserve the existing saved enable switch.
  remember_manual: bool = False
  reengage: bool = False

  @classmethod
  def parse(cls, values):
    numbers=[]
    for key,(low,high,default) in SPECS.items():
      value=values.get(key,default)
      if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
        raise ValueError('Invalid saved '+key)
      numbers.append(float(value))
    flags=[values.get(key,False) for key in BOOLS]
    if any(type(value) is not bool for value in flags):
      raise ValueError('Invalid custom SLC preference')
    return cls(*numbers,*flags)
