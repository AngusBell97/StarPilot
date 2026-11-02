"""SP-033 vehicle input assignments shared by native settings and Galaxy."""
import json
from openpilot.starpilot.common.car_control_config import (PARAM,INPUTS,MODELS,SPEED_UP,SPEED_DOWN,SPEED_SET,SPEED_RESUME,
  parse_assignments,vehicle_supported,screen_reserved)
from openpilot.starpilot.favorites.actions import BOOKMARK,CYCLE_PERSONALITY,EXPERIMENTAL,FORCE_COAST,PULSE_GLIDE,DISENGAGE,SELFIE,SCREEN_OFF,TRAFFIC,SWITCHBACK,FEATURE_KEYS,APPEARANCE_KEYS
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.saved_document import commit_exact
from openpilot.starpilot.ui.feature_settings_state import FeatureRow

PREFIX='tesla_input:'
OPTIONS={'Unassigned':'','Increase Comma Set Speed':SPEED_UP,'Decrease Comma Set Speed':SPEED_DOWN,
 'Set Speed To':SPEED_SET,'Resume Speed Limit + Offset':SPEED_RESUME,'Cycle Driving Personality':CYCLE_PERSONALITY,
 'Bookmark':BOOKMARK,'Experimental Mode':EXPERIMENTAL,'Force Coasting':FORCE_COAST,'Pulse and Glide':PULSE_GLIDE,
 'Disengage Openpilot':DISENGAGE,'Take Comma Selfie':SELFIE,'Toggle Screen Off':SCREEN_OFF,'Traffic Mode':TRAFFIC,'Switchback Mode':SWITCHBACK}
for key in (*[k for keys in FEATURE_KEYS.values() for k in keys],*APPEARANCE_KEYS):
  if key not in OPTIONS.values():OPTIONS['Toggle '+key]=key

class TeslaControlsFeature:
  def __init__(self,owner):self.owner=owner
  def context(self):
    cp=self.owner.vehicle_params();raw,readable=read_saved(self.owner.params,PARAM,16384)
    doc=parse_assignments(raw) if readable else None
    valid=bool(doc is not None and (not doc['vehicle'] or cp is not None and doc['vehicle']==cp.carFingerprint))
    return cp,raw,doc,valid
  def allowed(self):
    return self.owner.authority('parked_preferences') and self.owner.authority('preferences') and vehicle_supported(self.owner.vehicle_params())
  def rows(self):
    cp,raw,doc,valid=self.context()
    if cp is None or cp.carFingerprint not in MODELS:return []
    rows=[];metric,ok=read_saved(self.owner.params,'IsMetric',8)
    units_valid=ok and metric in (None,b'0',b'1');metric_on=metric==b'1'
    for name,(group,label) in INPUTS.items():
      reserved=name=='screen_three_finger_tap' and screen_reserved(cp)
      binding=doc['bindings'].get(name,{}) if valid else {}
      key=binding.get('key','');choice=next((name for name,value in OPTIONS.items() if value==key),'Unavailable saved action')
      help_text=('Reserved by Tesla Three-Finger AOL Tap for this drive.' if reserved else
        'Park to assign. Scroll: 1 mph (or km/h); fast scroll: 5 mph (or km/h). Comma speed actions require SLC on; with SLC off, comma follows the Tesla selected speed. Existing vehicle actions still occur.')
      rows.append(FeatureRow(PREFIX+name,group+' / '+label,choice if valid else 'Invalid or other-vehicle assignments',raw,
        tuple(OPTIONS),available=self.allowed() and valid and not reserved,reason=help_text,
        vehicle_fingerprint=self.owner.vehicle_fingerprint()))
      if key==SPEED_SET:
        value=binding.get('value');low,high=(8,145) if metric_on else (5,90)
        rows.append(FeatureRow(PREFIX+name+':speed','Set speed value',str(value),raw,step=1.,minimum=low,maximum=high,
          unit='km/h' if metric_on else 'mph',display_unit='km/h' if metric_on else 'mph',
          dependencies=(('IsMetric',metric),),available=self.allowed() and units_valid and not reserved,
          vehicle_fingerprint=self.owner.vehicle_fingerprint()))
    return rows
  def apply(self,request):
    cp,raw,doc,valid=self.context();name=request.key.removeprefix(PREFIX);numeric=name.endswith(':speed')
    if numeric:name=name[:-6]
    if not self.allowed() or not valid or name not in INPUTS or request.expected!=raw:return False
    if name=='screen_three_finger_tap' and screen_reserved(cp):return False
    fingerprint=self.owner.vehicle_fingerprint()
    if not request.vehicle_fingerprint or request.vehicle_fingerprint!=fingerprint:return False
    updated={**doc,'vehicle':str(cp.carFingerprint),'bindings':dict(doc['bindings'])}
    if numeric:
      metric,ok=read_saved(self.owner.params,'IsMetric',8)
      unit='km/h' if metric==b'1' else 'mph';low,high=(8,145) if metric==b'1' else (5,90)
      binding=updated['bindings'].get(name)
      if not ok or metric not in (None,b'0',b'1') or request.display_unit!=unit or request.dependencies!=(('IsMetric',metric),):return False
      try:value=float(request.value)
      except (TypeError,ValueError):return False
      if not low<=value<=high or binding is None or binding['key']!=SPEED_SET:return False
      updated['bindings'][name]={**binding,'value':value}
    else:
      if request.value not in OPTIONS or request.dependencies or request.display_unit:return False
      key=OPTIONS[request.value]
      if key:updated['bindings'][name]={'key':key,'value':30 if key==SPEED_SET else None,'label':request.value}
      else:updated['bindings'].pop(name,None)
    def authorized():
      return (self.allowed() and self.owner.vehicle_fingerprint()==fingerprint and
        (not numeric or read_saved(self.owner.params,'IsMetric',8)==(metric,True)))
    result=commit_exact(self.owner.params,key=PARAM,max_bytes=16384,expected=raw,
      raw=json.dumps(updated,separators=(',',':'),allow_nan=False).encode(),authorized=authorized,temp_prefix='.tesla-input-')
    return result.committed and result.verified
