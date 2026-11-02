"""Bounded offline geometry, only called in the non-realtime worker."""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field, replace
import hashlib
import math

from .types import Corner, Fix, Match, Way, MATCH_VERSION, PRIOR_VERSION

METRES_DEG = 111195.0
CELL = 100.0
CLASSES = {'motorway','trunk','primary','secondary','tertiary','unclassified','residential'}


def distance(a, b):
  return math.hypot(b[0]-a[0], b[1]-a[1])


def project(p, a, b):
  dx, dy = b[0]-a[0], b[1]-a[1]
  length2 = dx*dx+dy*dy
  f = max(0.0,min(1.0,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length2)) if length2 else 0.0
  return distance(p,(a[0]+f*dx,a[1]+f*dy)), f


def bearing(a,b):
  return math.degrees(math.atan2(b[0]-a[0],b[1]-a[1])) % 360


def angle_delta(a,b):
  return abs((a-b+180) % 360-180)


@dataclass
class Road:
  way: Way
  xy: tuple
  along: tuple


@dataclass
class Tile:
  origin: tuple
  provenance: str
  roads: dict
  cells: dict
  bounds: tuple | None = None
  corners: dict = field(default_factory=dict)
  nodes: dict = field(default_factory=dict)
  route_cache: dict = field(default_factory=dict)

  def xy(self, point):
    return ((point[1]-self.origin[1])*METRES_DEG*math.cos(math.radians(self.origin[0])),
            (point[0]-self.origin[0])*METRES_DEG)

  def nearby(self,p,radius):
    result=set()
    for x in range(math.floor((p[0]-radius)/CELL),math.floor((p[0]+radius)/CELL)+1):
      for y in range(math.floor((p[1]-radius)/CELL),math.floor((p[1]+radius)/CELL)+1):
        result.update(self.cells.get((x,y),()))
    return result


def build_tile(ways, provenance, bounds=None):
  if not ways or len(ways)>40000 or sum(len(w.points) for w in ways)>300000:
    raise ValueError('Unsupported map size')
  tile=Tile(ways[0].points[0],provenance,{}, {},bounds)
  owners={}
  shared=set()
  for way in ways:
    for point in way.points:
      if point in owners and owners[point]!=way.identity: shared.add(point)
      owners[point]=way.identity
  index_entries=0
  for way in ways:
    if len(way.points)<2:
      continue
    if len(way.points)>8192 or way.identity==0: raise ValueError('Unsupported road geometry')
    if not all(math.isfinite(v) for p in way.points for v in p):
      raise ValueError('Invalid map coordinates')
    if any(abs(p[0])>85 or abs(p[1])>180 for p in way.points):
      raise ValueError('Invalid map coordinates')
    clean=[]
    for point in way.points:
      if not clean or point in shared or distance(tile.xy(clean[-1]),tile.xy(point))>=.05:
        if not clean or clean[-1]!=point: clean.append(point)
    if len(clean)<2: raise ValueError('Degenerate competing road')
    way=replace(way,points=tuple(clean))
    if way.identity in tile.roads:
      if tile.roads[way.identity].way != way:
        raise ValueError('Conflicting road geometry')
      continue
    points=tuple(tile.xy(p) for p in way.points)
    along=[0.0]
    for a,b in zip(points,points[1:]):
      length=distance(a,b)
      if length<.05 or length>5000:
        break
      along.append(along[-1]+length)
    if len(along)!=len(points): raise ValueError('Invalid road geometry')
    tile.roads[way.identity]=Road(way,points,tuple(along))
    for i,point in enumerate(way.points):
      tile.nodes.setdefault(point,[]).append((way.identity,i))
    for i,(a,b) in enumerate(zip(points,points[1:])):
      for x in range(math.floor(min(a[0],b[0])/CELL),math.floor(max(a[0],b[0])/CELL)+1):
        for y in range(math.floor(min(a[1],b[1])/CELL),math.floor(max(a[1],b[1])/CELL)+1):
          tile.cells.setdefault((x,y),[]).append((way.identity,i))
          index_entries+=1
          if index_entries>1000000: raise ValueError('Map index capacity')
  return tile


def _sample(road, s):
  i=max(0,min(len(road.xy)-2,bisect_right(road.along,s)-1))
  f=max(0,min(1,(s-road.along[i])/(road.along[i+1]-road.along[i])))
  a,b=road.xy[i:i+2]
  return a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])


def _curvature(road,i):
  s=road.along[i]
  # 10 m half-window, enlarged for sparse vertices; avoids tiny-segment noise.
  window=max(10,min(30,(road.along[i+1]-road.along[i-1])/2))
  a,b,c=_sample(road,s-window),road.xy[i],_sample(road,s+window)
  denom=distance(a,b)*distance(b,c)*distance(a,c)
  return 2*((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/denom if denom>1 else 0


def _corners(tile,road,direction):
  cache_key=(road.way.identity,direction)
  if cache_key in tile.corners:
    return tile.corners[cache_key]
  if direction<0:
    total=road.along[-1]
    road=Road(road.way,tuple(reversed(road.xy)),tuple(total-s for s in reversed(road.along)))
  output=[]
  points=road.xy
  if len(points)>512 or distance(points[0],points[-1])<5 or road.way.road_class not in CLASSES or road.way.hazard:
    return ()
  ks=[0]+[_curvature(road,i) for i in range(1,len(points)-1)]+[0]
  # Unsupported vertices split a bend; a bad tail must not erase its supported
  # approach. Never fill a sparse gap or predict through an unqualified junction.
  supported=[]
  for i,k in enumerate(ks):
    valid=bool(k and .001<abs(k)<=.12 and road.along[i]>=40 and road.along[-1]-road.along[i]>=40)
    if valid:
      valid=max(road.along[i]-road.along[i-1],road.along[i+1]-road.along[i])<=80
    if valid:
      half=max(road.along[i+1]-road.along[i],road.along[i]-road.along[i-1])/2
      valid=not any(ident!=road.way.identity and
                    project(points[i],tile.roads[ident].xy[j],tile.roads[ident].xy[j+1])[0]<=40+half
                    for ident,j in tile.nearby(points[i],100))
    supported.append(k if valid else 0.0)
  ks=supported
  groups=[]
  active=[]
  sign=0
  for i,k in enumerate(ks):
    new_sign=1 if k>.001 else -1 if k<-.001 else 0
    if new_sign==0 or (sign and new_sign!=sign):
      if active: groups.append(active)
      active=[]
    if new_sign: active.append(i)
    sign=new_sign
  if active: groups.append(active)
  for indices in groups:
    first,last=indices[0],indices[-1]
    start,end=road.along[first],road.along[last]
    if end-start<15 or start<40 or road.along[-1]-end<40 or max(abs(ks[i]) for i in indices)>.12:
      continue
    if any(road.along[i+1]-road.along[i]>80 for i in range(max(0,first-1),min(last+1,len(points)-1))):
      continue
    nearby=set()
    junction=False
    # Exclude all other roads within 40m of any bend segment, even without shared
    # vertices/layer data. This deliberately excludes some parallel/crossing roads.
    for i in range(first,last+1):
      for ident,j in tile.nearby(points[i],100):
        if ident==road.way.identity: continue
        other=tile.roads[ident]
        local_half_gap=max(road.along[min(i+1,len(points)-1)]-road.along[i],road.along[i]-road.along[max(0,i-1)])/2
        if project(points[i],other.xy[j],other.xy[j+1])[0]<=40+local_half_gap:
          junction=True
        if project(points[i],other.xy[j],other.xy[j+1])[0]<100:
          nearby.add((ident,tuple((round(p[0],7),round(p[1],7)) for p in other.way.points[j:j+2])))
    if junction: continue
    source_points=road.way.points if direction>0 else tuple(reversed(road.way.points))
    context=source_points[max(0,first-3):min(len(points),last+4)]
    geometry=tuple((round(p[0],7),round(p[1],7)) for p in context)
    revision=hashlib.sha256(repr((geometry,sorted(nearby),MATCH_VERSION,PRIOR_VERSION)).encode()).hexdigest()
    key=hashlib.sha256(f'{road.way.identity}:{direction}:{revision}'.encode()).hexdigest()
    thirds=[[],[],[]]
    for i in indices:
      bucket=min(2,int((road.along[i]-start)/max(1,end-start)*3))
      thirds[bucket].append(max(5,min(40,math.sqrt(1.8/abs(ks[i])))))
    prior=tuple(min(bucket) if bucket else min(min(b) for b in thirds if b) for bucket in thirds)
    shape=tuple((round((p[0]-points[first][0])*10),round((p[1]-points[first][1])*10)) for p in points[first:last+1])
    if len(shape)>96 or any(abs(v)>32767 for p in shape for v in p): continue
    output.append(Corner(key,road.way.identity,direction,revision,start,(start+end)/2,end,prior,shape,
                         round(bearing(points[first],points[min(first+1,last)])),1 if ks[first]>0 else -1))
  tile.corners[cache_key]=tuple(output)
  return tuple(output)


def connected_near(tile, first, second, point, radius=45):
  """Only explicit shared map vertices, never a geometric crossing."""
  a=tile.roads[first]
  return any(any(ident==second for ident,_ in tile.nodes.get(p,())) and
             distance(tile.xy(p),point)<=radius for p in a.way.points)


def unique_join(tile,first,second,point):
  for node in tile.roads[first].way.points:
    members=tile.nodes.get(node,())
    if distance(tile.xy(node),point)>45 or not any(ident==second for ident,_ in members): continue
    neighbours=set()
    for ident,i in members:
      points=tile.roads[ident].way.points
      if i: neighbours.add(points[i-1])
      if i+1<len(points): neighbours.add(points[i+1])
    if len(neighbours)==2: return True
  return False


def transition_distance(tile, previous, match):
  """Observed motion across a shared vertex in directed way coordinates."""
  if previous.revision==match.revision: return match.progress-previous.progress
  if previous.way_id==match.way_id: return None
  old=tile.roads.get(previous.way_id);new=tile.roads.get(match.way_id)
  if old is None or new is None: return None
  deltas=[]
  for i,p in enumerate(old.way.points):
    old_s=old.along[i] if previous.direction>0 else old.along[-1]-old.along[i]
    if not -2<=old_s-previous.progress<=50: continue
    for ident,j in tile.nodes.get(p,()):
      if ident!=match.way_id: continue
      new_s=new.along[j] if match.direction>0 else new.along[-1]-new.along[j]
      if -2<=match.progress-new_s<=50:
        deltas.append(old_s-previous.progress+match.progress-new_s)
  return deltas[0] if len(deltas)==1 else None


def find_match(tile: Tile, fix: Fix, previous: Match | None = None) -> Match | None:
  if not fix.has_fix or not all(math.isfinite(v) for v in (fix.latitude,fix.longitude,fix.bearing,fix.accuracy,fix.speed)):
    return None
  if not 0<fix.accuracy<=5 or not 0<=fix.speed<=55: return None
  if tile.bounds:
    south,west,north,east=tile.bounds
    if min((fix.latitude-south)*METRES_DEG,(north-fix.latitude)*METRES_DEG,
           (fix.longitude-west)*METRES_DEG*math.cos(math.radians(fix.latitude)),
           (east-fix.longitude)*METRES_DEG*math.cos(math.radians(fix.latitude)))<1250:
      return None
  p=tile.xy((fix.latitude,fix.longitude))
  candidates=[]
  nearby=[]
  for ident,i in tile.nearby(p,30):
    road=tile.roads[ident]
    d,f=project(p,road.xy[i],road.xy[i+1])
    if d>25: continue
    along=road.along[i]+f*(road.along[i+1]-road.along[i])
    nearby.append((d,ident,along,i))
    h=bearing(road.xy[i],road.xy[i+1])
    direction=1 if angle_delta(h,fix.bearing)<=25 else -1 if angle_delta(h+180,fix.bearing)<=25 else 0
    width=max(1,min(4,road.way.lanes or 2))*3.5
    # Acquisition keeps the strict corridor. Only an established same-road
    # track gets a bounded allowance for map-centreline error on bends.
    extra=2.0 if previous and previous.way_id==ident and previous.direction==direction else 0.0
    if direction and (direction>0 or not road.way.one_way) and d<=width/2+fix.accuracy+extra and road.way.road_class in CLASSES:
      candidates.append((d,ident,along,direction))
  if not candidates: return None
  d,ident,along,direction=min(candidates)
  road=tile.roads[ident]
  progress=along if direction>0 else road.along[-1]-along
  rev=hashlib.sha256(repr((ident,direction,road.way.points)).encode()).hexdigest()
  proposed=Match(ident,direction,progress,d,12+fix.accuracy,(),rev)
  handoff=previous is not None and transition_distance(tile,previous,proposed) is not None
  for dist,other,s,j in nearby:
    if other==ident:
      if dist<8+fix.accuracy and abs(s-along)>30: return None
      continue
    if dist>=18+fix.accuracy: continue
    other_road=tile.roads[other]
    h=bearing(other_road.xy[j],other_road.xy[j+1])
    incompatible=min(angle_delta(h,fix.bearing),angle_delta(h+180,fix.bearing))>45
    explicit=connected_near(tile,ident,other,p)
    # Ignore a side access only on a sustained track. A same-heading successor
    # is harmless solely when it is the road just exited through a shared node.
    old_road=False
    if previous is not None and other==previous.way_id and ident!=other and handoff:
      # A continuing old way is still a plausible alternative at a shallow
      # fork. Only its directed endpoint can be treated as the road behind us.
      endpoint=other_road.way.points[-1 if previous.direction>0 else 0]
      old_road=(distance(tile.xy(endpoint),p)<=45 and
                any(joined==ident for joined,_ in tile.nodes.get(endpoint,())))
    continuation=unique_join(tile,ident,other,p)
    if not (handoff and explicit and (incompatible or old_road or continuation)): return None
  corners=_corners(tile,road,direction)
  # Local revision independently anchors continuity even on a straight.
  return Match(ident,direction,progress,d,12+fix.accuracy if previous else 10+fix.accuracy,corners,rev)
