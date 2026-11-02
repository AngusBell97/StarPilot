from dataclasses import replace
import pytest
from openpilot.starpilot.controls.lib.map_corner.controller import Coordinator
from openpilot.starpilot.controls.lib.map_corner.worker import Engine
from openpilot.starpilot.controls.lib.map_corner.store import CornerStore
from openpilot.starpilot.controls.lib.map_corner.types import Corner,Fix,Frame,Match

def test_map_owned_manual_slowdown_changes_only_next_visit_and_never_visual_cap(tmp_path):
  corner=Corner('bend',100,1,'local',300,350,400,(13,13,13),((0,0),(20,10)),90,curvature=tuple(1.8/(v*v) for v in (13,13,13)))
  class Provider:
    status='Map ready'
    progress=0
    def update(self,fix,now):return Match(100,1,self.progress,0,11,(corner,),'local')
  p=Provider();co=Coordinator();latest=None
  with CornerStore(tmp_path) as store:
    engine=Engine(p,store)
    changed_during_bend=False
    for i in range(1000):
      now=1+i*.05
      manual=p.progress>=120
      speed=11 if manual else 20
      takeover=manual and co.previous_auto
      d=co.step(now=now,speed=speed,acceleration=0,cruise=25,visual=25,visual_valid=False,other=25,
         automated=not manual,parent=True,learn=True,available=not manual,button=False,gas=False,
         brake=takeover,cancel=False,confounded=False,fault=False,result=latest)
      f=Fix(now,i+1,51,0,90,1,speed,True)
      frame=Frame(now,i+1,co.control_epoch,co.observation_fence,f,speed,0,co.odometer,not manual,True,
                  d.owner,d.intervention,d.observation_id,False,False,False,False,(),d.binding_key)
      latest=engine.step(frame)
      if p.progress<corner.exit:
        changed_during_bend |= store.lookup(corner) is not None
      if p.progress>corner.exit+12:break
      p.progress+=speed*.05
    assert not changed_during_bend
    learned=store.lookup(corner)
    assert learned is not None
    assert learned.profile==pytest.approx((12.10592,)*3)
    p.progress=0
    next_visit=Engine(p,store).step(replace(frame,now=100,sequence=1,fix=replace(f,received_at=100),speed=20,automated=True))
    assert next_visit.points[0].speed==pytest.approx(12.10592)
    assert next_visit.remembered

def test_real_tile_reindex_during_manual_pass_preserves_observation(tmp_path,monkeypatch):
  from pathlib import Path
  import capnp
  import openpilot.starpilot.controls.lib.map_corner.provider as mod
  from openpilot.starpilot.controls.lib.map_corner.geometry import _sample,bearing,METRES_DEG
  from test_map_corner_provider import bend_way,ll
  import math
  base=bend_way()
  road=replace(base,points=tuple(ll(x,0) for x in range(-600,-150,10))+base.points)
  schema=capnp.load(str(Path(mod.__file__).with_name('offline.capnp')))
  msg=schema.Offline.new_message(minLat=50.875,minLon=-.125,maxLat=51.125,maxLon=.125,overlap=0,
    ways=[{'id':road.identity,'highwayClass':'secondary','nodes':[{'latitude':a,'longitude':b} for a,b in road.points]}])
  tilefile=tmp_path/'tile';tilefile.write_bytes(msg.to_bytes_packed())
  provider=mod.OfflineProvider(tmp_path)
  monkeypatch.setattr(provider,'_paths',lambda f:(tilefile,))
  provider.update(Fix(1,1,*ll(-600,0),90,1,20,True),1)
  geometry=provider.tile.roads[road.identity]
  origin=provider.tile.origin
  progress=0.;co=Coordinator();latest=None;corner_key=''
  with CornerStore(tmp_path/'memory') as store:
    engine=Engine(provider,store)
    for i in range(1500):
      now=1.05+i*.05
      manual=progress>=470
      speed=11 if manual else 20
      decision=co.step(now=now,speed=speed,acceleration=0,cruise=25,visual=25,visual_valid=False,other=25,
        automated=not manual,parent=True,learn=True,available=not manual,button=False,gas=False,
        brake=manual and co.previous_auto,cancel=False,confounded=False,fault=False,result=latest)
      xy=_sample(geometry,progress);ahead=_sample(geometry,progress+1)
      lat=origin[0]+xy[1]/METRES_DEG
      lon=origin[1]+xy[0]/(METRES_DEG*math.cos(math.radians(origin[0])))
      f=Fix(now,i+2,lat,lon,bearing(xy,ahead),1,speed,True)
      latest=engine.step(Frame(now,i+1,co.control_epoch,co.observation_fence,f,speed,0,co.odometer,not manual,True,
        decision.owner,decision.intervention,decision.observation_id,False,False,False,False,(),decision.binding_key,decision.cap))
      if latest.key:corner_key=latest.key
      if store.stats()['observations']:break
      progress+=speed*.05
    assert corner_key and store.stats()['observations']==1,(engine.observer.rejections,engine.stats,progress,provider.status,co.previous_owner)
    assert provider.reindex_count>=2 and provider.decode_count==1
