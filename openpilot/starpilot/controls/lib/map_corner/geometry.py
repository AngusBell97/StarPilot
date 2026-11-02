"""Bounded offline geometry, only called in the non-realtime worker."""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass, field, replace
import hashlib
import math

from .types import Corner, Fix, Match, Way, MATCH_VERSION, PRIOR_VERSION, MPH

METRES_DEG = 111195.0
CELL = 100.0
CLASSES = {'motorway','trunk','primary','secondary','tertiary','unclassified','residential'}
# Known link geometry can prove physical connectivity, but cannot supply a
# matched road or speed target. Unknown classes and hazards still fail closed.
TOPOLOGY_CLASSES = frozenset(CLASSES | {'motorwayLink','trunkLink','primaryLink','secondaryLink','tertiaryLink'})


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


def legal_heading_error(road, segment, heading):
  """Stored one-way points follow legal travel, including normalized oneway=-1."""
  h=bearing(road.xy[segment],road.xy[segment+1])
  return angle_delta(h,heading) if road.way.one_way else min(angle_delta(h,heading),angle_delta(h+180,heading))


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
  continuity_cache: dict = field(default_factory=dict,repr=False,compare=False)
  _continuity_provenance: str | None = field(default=None,init=False,repr=False,compare=False)
  _cosine: float | None = field(default=None,init=False,repr=False,compare=False)
  _last_render: tuple | None = field(default=None,init=False,repr=False,compare=False)

  def xy(self, point):
    if self._cosine is None: self._cosine=math.cos(math.radians(self.origin[0]))
    return ((point[1]-self.origin[1])*METRES_DEG*self._cosine,
            (point[0]-self.origin[0])*METRES_DEG)

  def nearby(self,p,radius):
    result=set()
    for x in range(math.floor((p[0]-radius)/CELL),math.floor((p[0]+radius)/CELL)+1):
      for y in range(math.floor((p[1]-radius)/CELL),math.floor((p[1]+radius)/CELL)+1):
        result.update(self.cells.get((x,y),()))
    return result


def build_tile(ways, provenance, bounds=None, *, index=True):
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
    if index:
      for i,point in enumerate(way.points):
        tile.nodes.setdefault(point,[]).append((way.identity,i))
    for i,(a,b) in enumerate(zip(points,points[1:])):
      x0=math.floor(min(a[0],b[0])/CELL);x1=math.floor(max(a[0],b[0])/CELL)
      y0=math.floor(min(a[1],b[1])/CELL);y1=math.floor(max(a[1],b[1])/CELL)
      # Partial tiles are still normalized and fully validated before merging.
      index_entries+=(x1-x0+1)*(y1-y0+1)
      if index_entries>1000000: raise ValueError('Map index capacity')
      if index:
        for x in range(x0,x1+1):
          for y in range(y0,y1+1):
            tile.cells.setdefault((x,y),[]).append((way.identity,i))
  return tile


def _sample(road, s):
  i=max(0,min(len(road.xy)-2,bisect_right(road.along,s)-1))
  f=max(0,min(1,(s-road.along[i])/(road.along[i+1]-road.along[i])))
  a,b=road.xy[i:i+2]
  return a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1])


def _curvature_point(road,s,direction):
  target=max(road.along[0],min(road.along[-1],s+direction*10.0))
  j=max(1,min(len(road.xy)-1,bisect_left(road.along,target)))
  # A near-exact support vertex does not use the chord beyond it. Preserve
  # interpolation arithmetic here so dense geometry stays bitwise equivalent.
  if target-road.along[j-1]<=1e-6 or road.along[j]-target<=1e-6:
    return _sample(road,target)
  gap=road.along[j]-road.along[j-1]
  # A dense cluster can put the sample inside a sparse chord farther away.
  # Its far real vertex preserves curvature; unsupported support fails closed.
  if gap>80: return None
  # Projection round-off at a vertex must not jump to the next chord.
  if gap>=10 and road.along[j-1]+1e-6<target<road.along[j]-1e-6:
    return road.xy[j if direction>0 else j-1]
  return _sample(road,target)


def _sparse_curvature(road,i):
  s=road.along[i]
  # Keep the 10 m dense-point noise window without shortening sparse chords.
  a,b,c=_curvature_point(road,s,-1),road.xy[i],_curvature_point(road,s,1)
  if a is None or c is None: return None
  denom=distance(a,b)*distance(b,c)*distance(a,c)
  return 2*((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/denom if denom>1 else 0


# Preserve tighter-bend calibration. These are legacy estimated radii, not
# surveyed road radii; the transition deliberately leaves some bias in place.
CURVATURE_PRESERVE_RADIUS = 200.0
CURVATURE_FULL_RADIUS = 350.0


def _legacy_curvature(road,i):
  s=road.along[i]
  # 10 m half-window, enlarged for sparse vertices; avoids tiny-segment noise.
  window=max(10,min(30,(road.along[i+1]-road.along[i-1])/2))
  a,b,c=_sample(road,s-window),road.xy[i],_sample(road,s+window)
  denom=distance(a,b)*distance(b,c)*distance(a,c)
  return 2*((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/denom if denom>1 else 0


def _curvature(road,i):
  previous=_legacy_curvature(road,i)
  if previous==0 or abs(previous)>=1/CURVATURE_PRESERVE_RADIUS:
    return previous
  corrected=_sparse_curvature(road,i)
  # Missing support must not remove an existing target. Nor may a different
  # turn sign or a sharper estimate introduce new braking through this fix.
  if corrected is None or previous*corrected<0 or abs(corrected)>=abs(previous):
    return previous
  weight=min(1.0,(1/abs(previous)-CURVATURE_PRESERVE_RADIUS)/
             (CURVATURE_FULL_RADIUS-CURVATURE_PRESERVE_RADIUS))
  return previous+weight*(corrected-previous)


def seed_speed(curvature):
  """Global comfort prior; only called for supported absolute curvature."""
  old=math.sqrt(1.8/curvature)
  ordinary=math.sqrt(2.0/curvature)
  medium=min(34*MPH,math.sqrt(3.0/curvature),old+1.2*max(0,old-20*MPH))
  return max(5.0,min(40.0,max(ordinary,medium)))


def physical_speed(curvature):
  """Independent 4m/s^2 learned-speed ceiling, computed off the control loop."""
  return min(40.0,math.sqrt(4.0/curvature))


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
  # Keep bend boundaries and profile bins stable while calibrating speeds.
  # Re-grouping on relaxed curvature can shift braking targets or their keys.
  ks=[0]+[_legacy_curvature(road,i) for i in range(1,len(points)-1)]+[0]
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
    token=(geometry,sorted(nearby),MATCH_VERSION,PRIOR_VERSION)
    revision=hashlib.sha256(repr(token).encode()).hexdigest()
    key=hashlib.sha256(f'{road.way.identity}:{direction}:{revision}'.encode()).hexdigest()
    thirds=[[],[],[]]
    admission_thirds=[[],[],[]]
    preserve=max(abs(ks[i]) for i in indices)>=1/CURVATURE_PRESERVE_RADIUS
    for i in indices:
      bucket=min(2,int((road.along[i]-start)/max(1,end-start)*3))
      # A tighter bend keeps its entire approach profile. For a gentle bend,
      # retain its bins even when the corrected estimate is nearly straight;
      # a positive floor makes its resulting nonbinding ceiling finite.
      k=abs(ks[i]) if preserve else max(1e-9,abs(_curvature(road,i)))
      thirds[bucket].append(k)
      admission_thirds[bucket].append(abs(ks[i]))
    curvature=tuple(max(bucket) if bucket else max(max(b) for b in thirds if b) for bucket in thirds)
    prior=tuple(seed_speed(k) for k in curvature)
    admission_prior=tuple(seed_speed(max(bucket) if bucket else max(max(b) for b in admission_thirds if b)) for bucket in admission_thirds)
    shape=tuple((round((p[0]-points[first][0])*10),round((p[1]-points[first][1])*10)) for p in points[first:last+1])
    if len(shape)>96 or any(abs(v)>32767 for p in shape for v in p): continue
    output.append(Corner(key,road.way.identity,direction,revision,start,(start+end)/2,end,prior,shape,
                         round(bearing(points[first],points[min(first+1,last)])),1 if ks[first]>0 else -1,
                         curvature=curvature,admission_prior=admission_prior))
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
    if _degree_two(tile,node): return True
  return False


def _degree_two(tile,node):
  neighbours=set()
  for ident,i in tile.nodes.get(node,()):
    points=tile.roads[ident].way.points
    if i: neighbours.add(points[i-1])
    if i+1<len(points): neighbours.add(points[i+1])
  return len(neighbours)==2


def _local_join_nodes(tile,first,other,segment,point):
  """Shared nodes on this local arm, not a connection beyond a turn/junction."""
  road=tile.roads[other]
  heading=bearing(road.xy[segment],road.xy[segment+1])
  output=[]
  for node in tile.roads[first].way.points:
    if distance(tile.xy(node),point)>45: continue
    for ident,i in tile.nodes.get(node,()):
      if ident!=other: continue
      endpoint=min(max(i,segment),segment+1)
      if abs(road.along[i]-road.along[endpoint])>45: continue
      # Adjacent short segments can belong to the same arm. A bend or another
      # junction between the shared node and this segment removes that proof.
      lo,hi=sorted((i,endpoint))
      if any(any(owner!=other for owner,_ in tile.nodes.get(road.way.points[k],()))
             for k in range(lo,hi+1) if k!=i): continue
      if any(angle_delta(bearing(road.xy[k],road.xy[k+1]),heading)>25
             for k in range(lo,hi)): continue
      output.append(node)
  return tuple(output)


def transition_distance(tile, previous, match, *, unique_join_only=False, joining_node=None):
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
        deltas.append((old_s-previous.progress+match.progress-new_s,p))
  if len(deltas)!=1: return None
  delta,node=deltas[0]
  if joining_node is not None and node!=joining_node: return None
  return delta if not unique_join_only or _degree_two(tile,node) else None


def find_match(tile: Tile, fix: Fix, previous: Match | None = None, *, provisional=False, _continuity=None) -> Match | None:
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
      candidates.append((d,ident,along,direction,i))
  if not candidates: return None
  d,ident,along,direction,segment=min(candidates,key=lambda c:(c[1]!=previous.way_id,c[0],c[1:])) if _continuity is not None else min(candidates)
  road=tile.roads[ident]
  selected_heading=bearing(road.xy[segment],road.xy[segment+1])+(180 if direction<0 else 0)
  progress=along if direction>0 else road.along[-1]-along
  rev=hashlib.sha256(repr((ident,direction,road.way.points)).encode()).hexdigest()
  proposed=Match(ident,direction,progress,d,12+fix.accuracy,(),rev)
  handoff=previous is not None and transition_distance(tile,previous,proposed,unique_join_only=provisional or _continuity is not None) is not None
  if (provisional or _continuity is not None) and previous is not None and proposed.revision!=previous.revision and not handoff:
    return None
  for dist,other,s,j in nearby:
    if other==ident:
      if dist<8+fix.accuracy and abs(s-along)>30: return None
      continue
    if dist>=18+fix.accuracy: continue
    other_road=tile.roads[other]
    width=max(1,min(4,other_road.way.lanes or 2))*3.5
    # Opposing legal traffic is not a route choice. Heading alone cannot excuse
    # a fix inside its corridor or with insufficient separation from our road.
    # Apply per segment: a nearby diverging arm must still veto the match.
    if (other_road.way.one_way and fix.speed>=3 and
        legal_heading_error(other_road,j,fix.bearing)>=150 and
        legal_heading_error(other_road,j,selected_heading)>=150 and
        dist>d+max(2*fix.accuracy,3.0) and dist>width/2+fix.accuracy):
      continue
    if _continuity is not None and _continuity.permits(proposed,other,j):
      continue
    h=bearing(other_road.xy[j],other_road.xy[j+1])
    # Connection/crossing checks describe physical geometry, not legal travel.
    # Keep these and _local_join_nodes bidirectional.
    incompatible=min(angle_delta(h,fix.bearing),angle_delta(h+180,fix.bearing))>45
    explicit=connected_near(tile,ident,other,p)
    # Ignore a side access only on a sustained track. A same-heading successor
    # is harmless solely when it is the road just exited through a shared node.
    old_road=False
    if not provisional and previous is not None and other==previous.way_id and ident!=other and handoff:
      # A continuing old way is still a plausible alternative at a shallow
      # fork. Only its directed endpoint can be treated as the road behind us.
      endpoint=other_road.way.points[-1 if previous.direction>0 else 0]
      old_road=(distance(tile.xy(endpoint),p)<=45 and
                any(joined==ident for joined,_ in tile.nodes.get(endpoint,())))
    continuation=unique_join(tile,ident,other,p)
    if provisional:
      # A connection elsewhere on a crossing way cannot excuse this segment.
      nodes=_local_join_nodes(tile,ident,other,j,p)
      explicit=len(nodes)==1
      if explicit and previous is not None and other==previous.way_id and ident!=other:
        explicit=transition_distance(tile,previous,proposed,unique_join_only=True,joining_node=nodes[0]) is not None
      continuation=explicit and _degree_two(tile,nodes[0])
    if not (handoff and explicit and (incompatible or old_road or continuation)): return None
  corners=_corners(tile,road,direction)
  # Local revision independently anchors continuity even on a straight.
  return Match(ident,direction,progress,d,12+fix.accuracy if previous else 10+fix.accuracy,corners,rev)


class ContinuityGuard:
  """One bounded graph budget per fresh fix; unknown topology never earns a lock."""
  LIMIT=1024
  CACHE_LIMIT=512

  def __init__(self,tile,fix,previous,shadow=False,*,fork_free=0.0):
    self.tile=tile;self.fix=fix;self.previous=previous;self.shadow=shadow
    self.remaining=self.LIMIT;self.crossings={}
    self.fork_free=fork_free
    self.proofs={}
    self.proof_state='unknown'
    self.unknown=False
    self.p=tile.xy((fix.latitude,fix.longitude))
    self.near=tuple(tile.nearby(self.p,30))
    if len(self.near)>self.remaining:
      self.near=();self.remaining=0
    else: self.remaining-=len(self.near)
    if tile._continuity_provenance!=tile.provenance:
      tile.continuity_cache.clear();tile._continuity_provenance=tile.provenance

  def _spend(self,n=1):
    if self.remaining<n:
      self.remaining=0;return False
    self.remaining-=n;return True

  def _segment(self,match):
    road=self.tile.roads[match.way_id]
    along=match.progress if match.direction>0 else road.along[-1]-match.progress
    return max(0,min(len(road.xy)-2,bisect_right(road.along,along)-1))

  def crossed_fork(self,match):
    key=(match.revision,match.progress)
    if key in self.crossings: return self.crossings[key]
    from .routes import outgoing
    old=self.previous
    if old.way_id==match.way_id and old.direction==match.direction:
      intervals=[(old.way_id,old.direction,old.progress,match.progress)]
    elif old.way_id!=match.way_id:
      # Examine only the possible handoff interval, under the same budget as
      # connection searches. A whole-way rescan can dwarf a dense local graph.
      oldroad=self.tile.roads[old.way_id];newroad=self.tile.roads[match.way_id]
      lo,hi=old.progress-2,old.progress+50
      if old.direction<0: lo,hi=oldroad.along[-1]-hi,oldroad.along[-1]-lo
      joins=[]
      for i in range(bisect_left(oldroad.along,lo),bisect_right(oldroad.along,hi)):
        if self.remaining<=0: return None
        node=oldroad.way.points[i];members=self.tile.nodes.get(node,())
        if not self._spend(max(1,len(members))): return None
        for ident,j in members:
          if ident!=match.way_id: continue
          a=oldroad.along[i] if old.direction>0 else oldroad.along[-1]-oldroad.along[i]
          b=newroad.along[j] if match.direction>0 else newroad.along[-1]-newroad.along[j]
          if -2<=a-old.progress<=50 and -2<=match.progress-b<=50: joins.append((a,b,node))
      if len(joins)!=1 or not _degree_two(self.tile,joins[0][2]): return None
      a,b,_=joins[0];intervals=[(old.way_id,old.direction,old.progress,a),(match.way_id,match.direction,b,match.progress)]
    else: return None
    result=False
    for ident,direction,start,end in intervals:
      road=self.tile.roads[ident]
      if end<start-1: return None
      lo,hi=sorted((start,end))
      if direction<0: lo,hi=road.along[-1]-hi,road.along[-1]-lo
      for j in range(bisect_left(road.along,lo-2),bisect_right(road.along,hi+2)):
        node=road.way.points[j]
        if not self._spend(max(1,len(self.tile.nodes.get(node,())))): return None
        edge=(ident,j-1,1) if direction>0 and j else (ident,j,-1) if direction<0 and j<len(road.xy)-1 else None
        if edge is not None and len(outgoing(self.tile,node,edge))>1: result=True
    self.crossings[key]=result
    return result

  def clear_shadow(self,match):
    """All other nearby corridors must separate, even across way splits.

    This is stricter than tracking just the original fork arm. It cannot forget
    a long parallel slip because its junction or old way ID has fallen behind.
    """
    if self.remaining<=0: return False
    road=self.tile.roads[match.way_id]
    along=match.progress if match.direction>0 else road.along[-1]-match.progress
    for ident,j in self.near:
      other=self.tile.roads[ident];d,f=project(self.p,other.xy[j],other.xy[j+1])
      if d>=18+self.fix.accuracy: continue
      s=other.along[j]+f*(other.along[j+1]-other.along[j])
      if ident!=match.way_id or abs(s-along)>30: return False
    return True

  def disconnected(self,match,other,segment,*,limit=300.0):
    from heapq import heappop,heappush
    from .routes import outgoing,incoming_edges,edge_nodes
    root=(match.way_id,self._segment(match),match.direction)
    key=(limit,root,(other,segment))
    cache=self.tile.continuity_cache
    if key in cache: return cache[key]
    start,end=edge_nodes(self.tile,root)
    # Searching from both segment ends covers every position on that segment;
    # caching by edge is conservative for progress moving within the segment.
    heap=[(0.0,end),(0.0,start)];seen={};uncertain=False
    while heap:
      travelled,node=heappop(heap)
      if seen.get(node,float('inf'))<=travelled: continue
      seen[node]=travelled
      if not self._spend(max(1,len(self.tile.nodes.get(node,())))): return None
      # A shared merge/fork node is relevant even when the competing arm is
      # legal only towards it; searching outgoing edges alone misses merges.
      if any(ident==other for ident,_ in self.tile.nodes.get(node,())):
        cache[key]=False
        if len(cache)>self.CACHE_LIMIT: cache.pop(next(iter(cache)))
        return False
      # This is physical connectivity, not a proposed route. Follow both legal
      # incoming and outgoing arms so a fork/merge survives changes of way ID.
      choices=set(outgoing(self.tile,node))|set(incoming_edges(self.tile,node))
      for new in choices:
        if not self._spend(): return None
        if new[0]==other:
          cache[key]=False
          if len(cache)>self.CACHE_LIMIT: cache.pop(next(iter(cache)))
          return False
        way=self.tile.roads[new[0]].way
        if way.road_class not in TOPOLOGY_CLASSES or way.hazard:
          uncertain=True;continue
        a,b=edge_nodes(self.tile,new);length=distance(self.tile.xy(a),self.tile.xy(b))
        if travelled+length<=limit: heappush(heap,(travelled+length,b if a==node else a))
    if uncertain: return None
    cache[key]=True
    if len(cache)>self.CACHE_LIMIT: cache.pop(next(iter(cache)))
    return True

  def permits(self,match,other,segment):
    for ident in (match.way_id,other):
      way=self.tile.roads[ident].way
      if way.road_class not in CLASSES or way.hazard: return False
    crossed=self.crossed_fork(match)
    if crossed is not False: self.shadow=True
    if crossed is None or self.remaining<=0:
      self.unknown=True;self.proof_state='unknown'
      return False
    if self.shadow: return False
    separated=self.disconnected(match,other,segment)
    if separated is None:
      self.unknown=True;self.proof_state='unknown'
      return False
    self.proof_state='proved_disconnected' if separated else 'proved_connected'
    if separated:
      self.proofs[(other,segment)]='disconnected'
      return True
    # A far future connection cannot put the car on a crossing road now.
    # Admit this narrower proof only after positively checked fork-free travel.
    if self.fork_free<60+match.uncertainty:return False
    road=self.tile.roads[other]
    legal=legal_heading_error(road,segment,self.fix.bearing)
    tangent=bearing(road.xy[segment],road.xy[segment+1])
    physical=min(angle_delta(tangent,self.fix.bearing),angle_delta(tangent+180,self.fix.bearing))
    # Coordinate projection can put an exact120deg boundary a few ulps below it.
    # Classify that edge conservatively as opposing, retaining corridor checks.
    opposing=False
    if road.way.one_way and legal>=120-1e-6 and self.fix.speed>=3:
      selected=self.tile.roads[match.way_id];j=self._segment(match)
      heading=bearing(selected.xy[j],selected.xy[j+1])+(180 if match.direction<0 else 0)
      dist,_=project(self.p,road.xy[segment],road.xy[segment+1])
      width=max(1,min(4,road.way.lanes or 2))*3.5
      opposing=(legal_heading_error(road,segment,heading)>=120-1e-6 and
        dist>match.offset+max(2*self.fix.accuracy,3.0) and dist>width/2+self.fix.accuracy)
    # One-way opposing geometry always needs its corridor/separation proof,
    # including physical angles >45 degrees near the 120-degree boundary.
    if road.way.one_way and legal>=120-1e-6:
      if not opposing:return False
    elif physical<=45:return False
    local=self.disconnected(match,other,segment,limit=60.0)
    if local is None or self.remaining<=0:
      self.unknown=True;self.proof_state='unknown'
      return False
    if local is not True:return False
    self.proofs[(other,segment)]='opposing' if opposing else 'local_crossing'
    return True

  def competitor(self,match):
    candidates=[]
    for ident,j in self.near:
      if ident==match.way_id: continue
      road=self.tile.roads[ident]
      if road.way.road_class not in CLASSES or road.way.hazard: continue
      d,f=project(self.p,road.xy[j],road.xy[j+1])
      h=bearing(road.xy[j],road.xy[j+1])
      direction=1 if angle_delta(h,self.fix.bearing)<=25 else -1 if not road.way.one_way and angle_delta(h+180,self.fix.bearing)<=25 else 0
      width=max(1,min(4,road.way.lanes or 2))*3.5
      if not direction or d>width/2+self.fix.accuracy: continue
      along=road.along[j]+f*(road.along[j+1]-road.along[j])
      progress=along if direction>0 else road.along[-1]-along
      rev=hashlib.sha256(repr((ident,direction,road.way.points)).encode()).hexdigest()
      candidates.append(Match(ident,direction,progress,d,10+self.fix.accuracy,(),rev))
    return min(candidates,key=lambda m:(m.offset,m.way_id,m.progress)) if candidates else None
