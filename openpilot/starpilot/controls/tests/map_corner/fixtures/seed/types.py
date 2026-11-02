"""Immutable worker messages. Times are local monotonic seconds, speeds m/s."""
from dataclasses import dataclass

PRIOR_VERSION = 'geometry-1.8-v1'
MATCH_VERSION = 'routes-v2'
MPH = 0.44704


@dataclass(frozen=True)
class Fix:
  received_at: float
  identity: int
  latitude: float
  longitude: float
  bearing: float
  accuracy: float
  speed: float
  has_fix: bool


@dataclass(frozen=True)
class Way:
  identity: int
  points: tuple[tuple[float, float], ...]
  one_way: bool
  road_class: str
  hazard: str
  lanes: int = 2


@dataclass(frozen=True)
class Corner:
  key: str                         # Internal geometry digest; never published/logged.
  way_id: int
  direction: int
  revision: str                    # Local geometry/topology, NOT a whole tile digest.
  entry: float
  apex: float
  exit: float
  prior: tuple[float, float, float]
  shape: tuple[tuple[int, int], ...] # Relative decimetres, bounded signed16.
  heading: int
  turn: int = 1


@dataclass(frozen=True)
class Match:
  way_id: int
  direction: int
  progress: float
  offset: float
  uncertainty: float
  corners: tuple[Corner, ...]
  revision: str


@dataclass(frozen=True)
class Preference:
  profile: tuple[float, float, float]
  count: int
  reason: str


@dataclass(frozen=True)
class Sample:
  now: float
  progress: float
  speed: float
  automated: bool
  owner: str
  intervention: bool = False
  confounded: bool = False
  fault: bool = False
  visual_valid: bool = False
  clear_conditions: bool = False
  learn: bool = True
  observation_id: str = ''
  odometer: float = 0.0


@dataclass(frozen=True)
class Observation:
  corner: Corner
  profile: tuple[float, float, float]
  token: str
  faster_allowed: bool


@dataclass(frozen=True)
class Frame:
  now: float
  sequence: int
  control_epoch: int
  observation_fence: int
  fix: Fix | None
  speed: float
  acceleration: float
  odometer: float
  automated: bool
  learn: bool
  owner: str
  intervention: bool
  observation_id: str
  confounded: bool
  fault: bool
  visual_valid: bool
  clear_conditions: bool
  visual_profile: tuple[tuple[float, float, bool], ...] = ()
  binding_key: str = ''
  applied_map_cap: float = 0.0
  reference_speed: float = float('inf')  # Current non-map target; legacy callers use measured speed.


@dataclass(frozen=True)
class PointCap:
  distance: float
  speed: float
  prior: float
  key: str = ''


@dataclass(frozen=True)
class Result:
  now: float = 0.0
  sequence: int = 0
  control_epoch: int = 0
  odometer: float = 0.0
  key: str = ''
  progress: float = 0.0
  exit: float = 0.0
  points: tuple[PointCap, ...] = ()
  status: str = 'Map unavailable'
  remembered: bool = False
  uncertainty: float = 0.0
  visual_confirmed: bool = False
  route_options: tuple[tuple[PointCap, ...], ...] = ()
