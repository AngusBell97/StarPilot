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
 'SLCTransitionOffsets': ('Use SLC speed-change limits','','','When SLC changes speed, use the acceleration and braking limits set in each personality.'),
 'SLCWarningLeadTime': ('SLC Warning Lead Time','s',.5,'Warns this many seconds before slowing or speeding up for a mapped limit.'),
 'SLCLowerLimitTiming': ('Lower-limit timing','s',.5,'Moves when braking for a lower limit starts. 0 is calculated; positive is earlier, negative later.'),
 'SLCHigherLimitDistance': ('Higher-limit start distance','m',10.,'Starts speeding up this far before a higher limit begins.'),
 'SLCRememberManualOffset': ('Remember manual speed for each limit','','',('Remembers the speed you set for each limit, for Restore SLC speed on re-engagement. Each ' +
                                                                            'expires after 30 s in other limits.')),
 'SLCReengageSpeed': ('Restore SLC speed on re-engagement','','',('When you re-engage, go back to your remembered speed for this limit, or else the limit (never ' +
                                                                   'below your engage speed).')),
 'SLCConfirmationLower': ('Confirm lower limits','','','On: waits for you to confirm before slowing for a lower limit. Off: slows on its own.'),
 'SLCConfirmationHigher': ('Confirm higher limits','','','On: waits for you to confirm before speeding up for a higher limit. Off: speeds up on its own.'),
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
