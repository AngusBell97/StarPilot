"""Bounded onward-path geometry. No routing preference, networking or storage.

Options remain separate: a permissive continuation must survive the maximum.
The graph uses explicit shared coordinates, not inferred geometric crossings.
"""
from bisect import bisect_right
from dataclasses import dataclass, replace
import hashlib
import math
import struct

from .geometry import CLASSES, Road, _corners, _curvature, _sample, angle_delta, bearing, connected_near, distance, project
from .types import MATCH_VERSION, PointCap
from .controller import corner_positions

MAX_PATHS=32
MAX_EDGES=1024
MAX_BRANCHES=64
MAX_POINTS=128
HORIZON=1100.0


@dataclass(frozen=True)
class Preview:
  key: str
  common_end: float
  options: tuple[tuple[PointCap, ...], ...]
  complete: bool


def outgoing(tile,node,previous=None):
  edges=set()
  for ident,i in tile.nodes.get(node,()):
    road=tile.roads[ident]
    if i<len(road.xy)-1: edges.add((ident,i,1))
    if i and not road.way.one_way: edges.add((ident,i-1,-1))
  if previous: edges.discard((previous[0],previous[1],-previous[2]))
  return tuple(sorted(edges))


def edge_nodes(tile,edge):
  ident,i,direction=edge
  pair=tile.roads[ident].way.points[i:i+2]
  return pair if direction>0 else tuple(reversed(pair))


def junction_speed(deflection):
  if deflection<=45: return None
  radius=max(120.0,120.0/math.tan(math.radians(min(179,deflection))/2))
  return min(40.0,math.sqrt(2.5*radius))


def unknown_continuation(tile,node,incoming):
  """A near-miss straight continuation must not leave only a mapped turn."""
  a,b=edge_nodes(tile,incoming);p=tile.xy(node)
  h=bearing(tile.xy(a),tile.xy(b))
  connected={ident for ident,_ in tile.nodes.get(node,())}
  for ident,i in tile.nearby(p,40):
    if ident in connected: continue
    road=tile.roads[ident];d,f=project(p,road.xy[i],road.xy[i+1])
    if d>40: continue
    other_h=bearing(road.xy[i],road.xy[i+1])
    if min(angle_delta(h,other_h),angle_delta(h,other_h+180))>30: continue
    q=(road.xy[i][0]+f*(road.xy[i+1][0]-road.xy[i][0]),road.xy[i][1]+f*(road.xy[i+1][1]-road.xy[i][1]))
    forward=(q[0]-p[0])*math.sin(math.radians(h))+(q[1]-p[1])*math.cos(math.radians(h))
    # An explicit connection just ahead will be enumerated at its actual node.
    # Do not invent an additional unknown exit beside a mapped ring/side road.
    if forward>=-2 and not connected_near(tile,incoming[0],ident,p,80): return True
  return False


def _targets(tile,nodes,edges,root,root_direction,edges_at=None,path_key=None):
  if edges_at is None: edges_at=lambda node,edge:outgoing(tile,node,edge)
  if path_key is None: path_key=lambda token:'path:'+hashlib.sha256(repr(token).encode()).hexdigest()
  if len(nodes)<3: return ()
  xy=tuple(tile.xy(p) for p in nodes)
  along=[0.0]
  for a,b in zip(xy,xy[1:]): along.append(along[-1]+distance(a,b))
  road=Road(tile.roads[root].way,xy,tuple(along));points=[];covered=set()
  # An identified bend must keep the same prior profile when an onward option
  # becomes the observed road. Otherwise bin-start targeting appears suddenly
  # at commitment and the comfortable-acquisition gate correctly rejects it.
  start=0
  while start<len(edges):
    ident,j,direction=edges[start];end=start+1
    while (end<len(edges) and edges[end][0]==ident and edges[end][2]==direction and
           edges[end][1]==edges[end-1][1]+direction): end+=1
    local=tile.roads[ident];index=j if direction>0 else j+1
    local_start=local.along[index] if direction>0 else local.along[-1]-local.along[index]
    local_end=local_start+along[end]-along[start]
    for corner in _corners(tile,local,direction):
      if corner.entry<local_start-1e-6 or corner.exit>local_end+1e-6: continue
      offset=along[start]-local_start
      points.extend(PointCap(offset+s,v,v,corner.key) for s,v in zip(corner_positions(corner),corner.prior))
      covered.update(i for i in range(start,end+1) if corner.entry-1e-6<=along[i]-offset<=corner.exit+1e-6)
    start=end
  for i in range(1,len(nodes)-1):
    before,after=edges[i-1],edges[i]
    first=tile.roads[before[0]].way;second=tile.roads[after[0]].way
    if first.road_class not in CLASSES or second.road_class not in CLASSES or first.hazard or second.hazard: continue
    junction=len(edges_at(nodes[i],before))>1
    if junction:
      if min(along[i],along[-1]-along[i])<30: continue
      h0=bearing(_sample(road,along[i]-30),xy[i]);h1=bearing(xy[i],_sample(road,along[i]+30))
      speed=junction_speed(angle_delta(h0,h1))
      if speed is None: continue
    else:
      if i in covered: continue
      if max(along[i]-along[i-1],along[i+1]-along[i])>80: continue
      k=abs(_curvature(road,i))
      if not .001<k<=.12: continue
      speed=max(5.0,min(40.0,math.sqrt(1.8/k)))
    # A local corner has the same ID before/after observed route commitment.
    # Junction-spanning targets remain prior-only and have no memory lookup.
    ident,j,direction=after
    local=tile.roads[ident]
    index=j if direction>0 else j+1
    s=local.along[index] if direction>0 else local.along[-1]-local.along[index]
    key=''
    if not junction:
      for corner in _corners(tile,local,direction):
        if corner.entry<=s<=corner.exit:
          key=corner.key
          break
    if not key:
      token=(MATCH_VERSION,before,after,nodes[i-1:i+2])
      key=path_key(token)
    points.append(PointCap(along[i],speed,speed,key))
    if len(points)>MAX_POINTS: return None
  if len(points)>MAX_POINTS: return None
  return tuple(sorted(points,key=lambda p:p.distance))


def _build(tile,match,segment):
  # Shared prefixes are visited by several alternatives. These caches live only
  # for this bounded traversal, never across tiles or changing control inputs.
  outgoing_cache={};unknown_cache={};path_keys={}
  def edges_at(node,edge):
    key=(node,edge)
    if key not in outgoing_cache: outgoing_cache[key]=outgoing(tile,node,edge)
    return outgoing_cache[key]
  def unknown_at(node,edge):
    key=(node,edge)
    if key not in unknown_cache: unknown_cache[key]=unknown_continuation(tile,node,edge)
    return unknown_cache[key]
  def path_key(token):
    # Dict tuple equality merges signed zero; geometry IDs hash its exact repr.
    text=repr(token)
    if text not in path_keys: path_keys[text]='path:'+hashlib.sha256(text.encode()).hexdigest()
    return path_keys[text]
  root=tile.roads[match.way_id];direction=match.direction
  edge=(match.way_id,segment,direction)
  first,last=edge_nodes(tile,edge)
  index=segment if direction>0 else segment+1
  indices=list(range(max(0,index-2),index+1)) if direction>0 else list(range(min(len(root.xy)-1,index+2),index-1,-1))
  prefix=tuple(root.way.points[i] for i in indices)
  prefix_edges=tuple((match.way_id,min(a,b),direction) for a,b in zip(indices,indices[1:]))
  prefix_length=sum(distance(tile.xy(a),tile.xy(b)) for a,b in zip(prefix,prefix[1:]))
  start=root.along[index] if direction>0 else root.along[-1]-root.along[index]
  nodes=prefix+(last,);edges=prefix_edges+(edge,)
  length=prefix_length+distance(tile.xy(first),tile.xy(last))
  stack=[(nodes,edges,frozenset((edge,)),length,0)]
  paths=[];complete=True;first_branch=math.inf;work=0
  while stack:
    nodes,edges,visited,length,branches=stack.pop();work+=1
    if work>MAX_PATHS*MAX_EDGES or len(paths)+len(stack)>MAX_PATHS:
      complete=False;break
    last_edge=edges[-1];node=nodes[-1];road=tile.roads[last_edge[0]].way
    options=edges_at(node,last_edge)
    at=length-prefix_length
    unknown=unknown_at(node,last_edge)
    if len(options)>1 or unknown:
      first_branch=min(first_branch,at)
    if (at>=HORIZON or not options or road.road_class not in CLASSES or road.hazard):
      paths.append((nodes,edges));continue
    if len(edges)>=MAX_EDGES or branches>=MAX_BRANCHES:
      complete=False;break
    if unknown: paths.append((nodes,edges))
    for new_edge in options:
      if new_edge in visited:
        paths.append((nodes,edges));continue
      _,end=edge_nodes(tile,new_edge)
      stack.append((nodes+(end,),edges+(new_edge,),visited|{new_edge},
                    length+distance(tile.xy(node),tile.xy(end)),branches+(len(options)>1)))
  output=[]
  for nodes,edges in paths[:MAX_PATHS]:
    points=_targets(tile,nodes,edges,match.way_id,direction,edges_at,path_key)
    if points is None:
      complete=False;continue
    output.append(tuple(PointCap(p.distance-prefix_length,p.speed,p.prior,p.key) for p in points if p.distance>=prefix_length))
  if not complete: output.append(())
  # Empty is a meaningful permissive option. dict.fromkeys preserves it.
  options=tuple(dict.fromkeys(output)) or ((),)
  key='routes:'+hashlib.sha256(repr((match.way_id,direction,segment,MATCH_VERSION)).encode()).hexdigest()
  common_end=min(root.along[-1],start+first_branch)
  return start,Preview(key,common_end,options,complete)


def preview_routes(tile,match):
  root=tile.roads[match.way_id]
  along=match.progress if match.direction>0 else root.along[-1]-match.progress
  segment=max(0,min(len(root.xy)-2,bisect_right(root.along,along)-1))
  key=(match.way_id,segment,match.direction)
  if key not in tile.route_cache:
    if len(tile.route_cache)>=64: tile.route_cache.pop(next(iter(tile.route_cache)))
    tile.route_cache[key]=_build(tile,match,segment)
  start,preview=tile.route_cache[key]
  progress_key=struct.pack('!d',match.progress)
  if tile._last_render is not None and tile._last_render[:2]==(key,progress_key):
    return tile._last_render[2]
  travelled=match.progress-start
  options=tuple(tuple(PointCap(max(0,p.distance-travelled),p.speed,p.prior,p.key) for p in points
                      if travelled-10<=p.distance<=travelled+HORIZON) for points in preview.options)
  rendered=replace(preview,options=tuple(dict.fromkeys(options)))
  tile._last_render=(key,progress_key,rendered)
  return rendered
