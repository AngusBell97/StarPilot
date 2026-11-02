"""Existing offline tiles only: no map download, daemon settings or network I/O."""
from pathlib import Path
import hashlib
import math
import time

from .geometry import build_tile, find_match, transition_distance, angle_delta
from .types import Way

MAP_ROOT = Path('/data/media/0/osm/offline')


def tile_relative_path(latitude, longitude):
  lat=math.floor(latitude*4)/4
  lon=math.floor(longitude*4)/4
  return Path(str(math.floor(lat/2)*2))/str(math.floor(lon/2)*2)/f'{lat:.6f}_{lon:.6f}_{lat+.25:.6f}_{lon+.25:.6f}'


class MatchTracker:
  def __init__(self):
    self.reset()

  def reset(self):
    self.tile_provenance=None
    self.last_fix=None
    self.last_match=None
    self.since=0.0
    self.start=0.0
    self.offset=0.0
    self.qualified=False
    self.travel=0.0

  def update(self,tile,fix):
    if self.tile_provenance!=tile.provenance:
      self.reset()
      self.tile_provenance=tile.provenance
    if self.last_fix and fix.identity==self.last_fix.identity:
      return self.last_match if self.qualified else None
    previous=self.last_match
    dt=fix.received_at-self.last_fix.received_at if self.last_fix else 0
    established=previous if self.qualified and 0<dt<=.75 else None
    match=find_match(tile,fix,established)
    expected=(fix.speed+self.last_fix.speed)*.5*dt if self.last_fix else 0
    delta=transition_distance(tile,previous,match) if previous is not None and match is not None else None
    continuous=(previous is not None and match is not None and
                (match.revision==previous.revision or established is not None) and 0<dt<=.75 and
                delta is not None and -1<=delta<=55*dt+5 and
                abs(delta-expected)<=max(5,expected*.35) and
                angle_delta(fix.bearing,self.last_fix.bearing)<=max(10,60*dt) and
                abs(match.offset-previous.offset)<=2.5)
    if not continuous:
      self.since=fix.received_at
      self.start=match.progress if match else 0
      self.offset=match.offset if match else 0
      self.qualified=False
      self.travel=0.0
    elif delta is not None:
      self.travel+=max(0,delta)
    self.last_fix=fix
    self.last_match=match
    if match and fix.received_at-self.since>=4 and self.travel>=100:
      self.qualified=True
    return match if self.qualified else None


class OfflineProvider:
  def __init__(self,root=MAP_ROOT):
    self.root=Path(root)
    self.tile=None
    self.path=None
    self.stamp=None
    self.last_check=0.0
    self.tracker=MatchTracker()
    self.status='Map unavailable'
    self.schema=None
    self.failed_paths=None
    self.retry_after=0.0
    self.failed_stamp=None
    self.decoded={}
    self.bad_decodes={}
    self.failed_position=None
    self.decode_count=0
    self.reindex_count=0

  def _paths(self,fix):
    lat=math.floor(fix.latitude*4)/4
    lon=math.floor(fix.longitude*4)/4
    dy=1250/111195
    dx=dy/max(.1,math.cos(math.radians(fix.latitude)))
    lats={lat}
    lons={lon}
    if fix.latitude-lat<dy: lats.add(lat-.25)
    if lat+.25-fix.latitude<dy: lats.add(lat+.25)
    if fix.longitude-lon<dx: lons.add(lon-.25)
    if lon+.25-fix.longitude<dx: lons.add(lon+.25)
    return tuple(sorted(self.root/tile_relative_path(a+.001,b+.001) for a in lats for b in lons))

  @staticmethod
  def _stamp(path):
    stat=path.stat()
    return stat.st_size,stat.st_mtime_ns,stat.st_ino

  def _decode(self,path):
    before=self._stamp(path)
    if not 32<=before[0]<=16*1024*1024: raise ValueError('Unsupported tile size')
    raw=path.read_bytes()
    digest=hashlib.sha256(raw).hexdigest()
    if before!=self._stamp(path) or len(raw)!=before[0]: raise ValueError('Map update in progress')
    if self.schema is None:
      import capnp
      self.schema=capnp.load(str(Path(__file__).with_name('offline.capnp')))
    data=self.schema.Offline.from_bytes_packed(raw,traversal_limit_in_words=16*1024*1024)
    self.decode_count+=1
    bounds=(data.minLat,data.minLon,data.maxLat,data.maxLon)
    if not all(math.isfinite(v) for v in (*bounds,data.overlap)) or not 0<=data.overlap<=.01:
      raise ValueError('Invalid tile bounds')
    south,west,north,east=bounds
    if not .24<=north-south<=.26 or not .24<=east-west<=.26 or len(data.ways)>40000:
      raise ValueError('Unsupported tile')
    ways=[];boxes=[]
    min_lat=south-.02;max_lat=north+.02;min_lon=west-.02;max_lon=east+.02
    centre_lat=(south+north)/2;centre_lon=(west+east)/2
    count=0
    for w in data.ways:
      count+=len(w.nodes)
      if count>300000: raise ValueError('Unsupported tile size')
      points=[];finite=True;intersects=False;unbounded=False
      for p in w.nodes:
        lat=p.latitude;lon=p.longitude
        if not points: lo_lat=hi_lat=lat;lo_lon=hi_lon=lon
        # Strict comparisons retain the first equal coordinate, including -0.0.
        if lat<lo_lat: lo_lat=lat
        if lat>hi_lat: hi_lat=lat
        if lon<lo_lon: lo_lon=lon
        if lon>hi_lon: hi_lon=lon
        points.append((lat,lon))
        if not math.isfinite(lat) or not math.isfinite(lon): finite=False
        if min_lat<=lat<=max_lat and min_lon<=lon<=max_lon: intersects=True
        if abs(lat-centre_lat)>2 or abs(lon-centre_lon)>2: unbounded=True
      # Ways can cross the tile edge: the offline generator includes full ways.
      # Require intersection with bounds and bounded geographic extent instead
      # of rejecting valid full-way vertices outside its overlap.
      if not points: continue
      if not finite: raise ValueError('Invalid tile coordinates')
      if not intersects:
        raise ValueError('Tile geometry outside bounds')
      if unbounded:
        raise ValueError('Unbounded map geometry')
      ways.append(Way(int(w.id),tuple(points),bool(w.oneWay),str(w.highwayClass),str(w.hazard)[:120],int(w.lanes)))
      boxes.append((lo_lat,lo_lon,hi_lat,hi_lon))
    if not ways or any(w.identity==0 for w in ways): raise ValueError('Unsupported map identity')
    if before!=self._stamp(path): raise ValueError('Map update in progress')
    return tuple(ways),tuple(boxes),bounds,digest,before,count

  def _load(self,path,fix=None):
    stamp=self._stamp(path)
    cached=self.decoded.get(path)
    if cached is None or cached[4]!=stamp:
      if self.bad_decodes.get(path)==stamp: raise ValueError('Map decode unavailable')
      try: decoded=self._decode(path)
      except Exception:
        self.bad_decodes[path]=stamp
        raise
      retained=sum(v[5] for k,v in self.decoded.items() if k!=path)
      if retained+decoded[5]>600000: raise ValueError('Decoded map capacity')
      self.decoded[path]=decoded
      cached=decoded
    ways,boxes,bounds,digest,before,_=cached
    if fix is not None:
      margin=.02/max(.1,math.cos(math.radians(fix.latitude)))
      ways=tuple(w for w,b in zip(ways,boxes) if b[2]>=fix.latitude-.02 and b[0]<=fix.latitude+.02 and
                 b[3]>=fix.longitude-margin and b[1]<=fix.longitude+margin)
    self.reindex_count+=1
    return build_tile(ways,digest,bounds,index=False),before

  def update(self,fix,now=None):
    now=time.monotonic() if now is None else now
    if fix is None or not 0<=now-fix.received_at<=.75:
      self.tracker.reset()
      self.status='GPS unavailable'
      return None
    if not all(math.isfinite(v) for v in (fix.latitude,fix.longitude)) or abs(fix.latitude)>85 or abs(fix.longitude)>180:
      self.tracker.reset()
      return None
    paths=self._paths(fix)
    if paths==self.failed_paths and now<self.retry_after:
      return None
    stamp=None
    try:
      if paths!=self.path or now-self.last_check>=1:
        self.last_check=now
        stamp=tuple(self._stamp(p) for p in paths)
        failure_moved=self.failed_position is not None and math.hypot((fix.latitude-self.failed_position[0])*111195,
          (fix.longitude-self.failed_position[1])*111195*math.cos(math.radians(fix.latitude)))>=500
        if paths==self.failed_paths and stamp==self.failed_stamp and not failure_moved:
          self.retry_after=now+1.0
          return None
        moved=self.tile is not None and math.hypot((fix.latitude-self.loaded_at[0])*111195,
                    (fix.longitude-self.loaded_at[1])*111195*math.cos(math.radians(fix.latitude)))>500
        if paths!=self.path or stamp!=self.stamp or moved:
          if sum(s[0] for s in stamp)>48*1024*1024: raise ValueError('Unsupported map size')
          ways={}
          self.decoded={p:v for p,v in self.decoded.items() if p in paths}
          self.bad_decodes={p:v for p,v in self.bad_decodes.items() if p in paths}
          bounds=[]
          provenance=[]
          for path in paths:
            part,_=self._load(path,fix)
            bounds.append(part.bounds)
            provenance.append(part.provenance)
            for identity,road in part.roads.items():
              if identity in ways and ways[identity]!=road.way: raise ValueError('Conflicting map tiles')
              ways[identity]=road.way
          if stamp!=tuple(self._stamp(p) for p in paths): raise ValueError('Map update in progress')
          box=(min(b[0] for b in bounds),min(b[1] for b in bounds),max(b[2] for b in bounds),max(b[3] for b in bounds))
          self.tile=build_tile(tuple(ways.values()),hashlib.sha256(''.join(provenance).encode()).hexdigest(),box)
          self.stamp=stamp
          self.path=paths
          self.loaded_at=(fix.latitude,fix.longitude)
      match=self.tracker.update(self.tile,fix)
      self.failed_paths=None
      self.status='Map ready' if match else 'Qualifying road match'
      return match
    except Exception:
      self.tile=None
      self.path=None
      self.tracker.reset()
      self.status='Map unavailable'
      self.failed_paths=paths
      self.retry_after=now+1.0
      self.failed_stamp=stamp
      self.failed_position=(fix.latitude,fix.longitude)
      return None
