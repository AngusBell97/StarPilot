"""SP-011 settings shared by native and Galaxy, with exact saved-value writes."""
from openpilot.starpilot.speed_limits.custom_settings import SPECS, BOOLS
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow
import math

CONFIRM = ('SLCConfirmationLower','SLCConfirmationHigher')
MOVED = frozenset(("SLCWarningVolume", "SLCChangeWarning"))
KEYS = (frozenset(SPECS) | frozenset(BOOLS) | frozenset(CONFIRM)) - MOVED
LABELS = {
 'SLCTransitionOffsets': ('Use SLC speed-change limits','','','Use the fixed acceleration and braking limits set in each driving profile. Existing SLC activation and protective bypass rules still apply.'),
 'SLCWarningLeadTime': ('SLC Warning Lead Time','s',.5,'Seconds before the planned start of slowing or speeding up for a mapped limit. Sudden accepted limit changes warn immediately.'),
 'SLCLowerLimitTiming': ('Lower-limit timing','s',.5,'Offset the calculated braking point so the selected deceleration profile aims to reach the new target at the boundary. 0 uses the calculation; negative later, positive earlier.'),
 'SLCHigherLimitDistance': ('Higher-limit start distance','m',10.,'Begin adopting a higher mapped limit this far before its boundary.'),
 'SLCRememberManualOffset': ('Remember manual speed for each limit','','','Used by Restore SLC speed on re-engagement. Remembers a separate manual target for each posted limit. Each expires after 30 seconds in other valid limits, even while disengaged. Missing map data pauses expiry; a restart clears memory.'),
 'SLCReengageSpeed': ('Restore SLC speed on re-engagement','','','Recall an enabled same-zone manual memory; otherwise match the limit without lowering a faster engagement speed.'),
 'SLCConfirmationLower': ('Confirm lower limits','','','Off uses mapped braking timing for map limits.'),
 'SLCConfirmationHigher': ('Confirm higher limits','','','Off uses mapped distance timing for map limits.'),
}

class SlcCustomFeature:
  def __init__(self, owner):self.owner=owner

  def authorized(self):
    return self.owner.authority('slc') and self.owner.longitudinal_available()

  def rows(self, allowed):
    rows=[]
    for key in (*CONFIRM,*SPECS,*BOOLS):
      if key not in KEYS:continue
      raw,ok=read_saved(self.owner.params,key,32)
      label,unit,step,help_text=LABELS[key]
      boolean=key not in SPECS
      valid=ok
      try:
        if boolean:
          if raw not in (None,b'0',b'1'):raise ValueError()
          if raw is None and key in CONFIRM:
            fallback,valid=read_saved(self.owner.params,'SLCConfirmation',8)
            valid=valid and fallback in (None,b'0',b'1')
            value='On' if fallback==b'1' else 'Off'
          else:value='On' if raw==b'1' else 'Off'
        else:
          low,high,default=SPECS[key]
          numeric=float(raw) if raw is not None else default
          if not math.isfinite(numeric) or not low<=numeric<=high:raise ValueError()
          value=f'{numeric:g}'
      except (ValueError,OverflowError):valid=False;value='Invalid saved value'
      low,high,default=SPECS.get(key,(0,0,0))
      rows.append(FeatureRow(key,label,value,raw,('Off','On') if boolean else (),
        step=0. if boolean else step,minimum=low,maximum=high,unit=unit,
        available=allowed and (valid or (key == "SLCWarningLeadTime" and ok)),reason=help_text,vehicle_fingerprint=self.owner.vehicle_fingerprint(),
        default_value='Off' if boolean else f'{default:g}',
        repair_value='2' if key == 'SLCWarningLeadTime' and not valid and ok else ''))
    return rows

  def apply(self, request):
    if request.key not in KEYS or request.dependencies or request.display_unit or request.related_source is not None:
      return False
    def allowed():
      return (self.authorized() and bool(request.vehicle_fingerprint) and
              request.vehicle_fingerprint==self.owner.vehicle_fingerprint())
    if not allowed():return False
    if request.key in SPECS:
      low,high,_=SPECS[request.key]
      try:value=float(request.value)
      except (ValueError,TypeError):return False
      if not math.isfinite(value) or not low<=value<=high:return False
      if request.key not in ('SLCLowerLimitTiming','SLCWarningLeadTime') and not value.is_integer():return False
      if request.key == 'SLCWarningLeadTime' and (value * 2) % 1:return False
      raw=f'{value:g}'.encode()
    else:
      if request.value not in ('On','Off'):return False
      raw=b'1' if request.value=='On' else b'0'
    result=commit_exact(self.owner.params,key=request.key,max_bytes=32,expected=request.expected,
                        raw=raw,authorized=allowed,temp_prefix='.slc-custom-')
    return result.committed and result.verified
