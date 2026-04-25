import math
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class FormationPlan:
    formation_name: str
    follower_offsets: List[Vec3]  # offsets for UAV1..UAVn relative to leader
    leader_waypoints: List[Vec3]


def _dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def distance(a: Vec3, b: Vec3) -> float:
    return _dist(a, b)


def validate_plan(plan: FormationPlan, *, min_separation_m: float = 0.9) -> None:
    if not plan.leader_waypoints:
        raise ValueError("leader_waypoints must be non-empty")
    if len(plan.follower_offsets) < 1:
        raise ValueError("follower_offsets must contain at least one offset (for UAV1+)")

    offsets = [(0.0, 0.0, 0.0), *plan.follower_offsets]
    for i in range(len(offsets)):
        for j in range(i + 1, len(offsets)):
            if _dist(offsets[i], offsets[j]) < min_separation_m:
                raise ValueError(
                    f"Offsets too close: uav{i} vs uav{j} separation < {min_separation_m}m"
                )


class WaypointProgressor:
    """
    Advances through waypoints using either:
    - closed-loop distance-to-waypoint when a leader position is available, or
    - open-loop time-based stepping as a fallback.
    """

    def __init__(
        self,
        waypoints: Sequence[Vec3],
        *,
        arrival_radius_m: float = 0.6,
        fallback_ticks_per_waypoint: int = 80,  # 0.1s tick -> ~8s per waypoint
        loop: bool = True,
    ):
        if not waypoints:
            raise ValueError("waypoints must be non-empty")
        self._waypoints = list(waypoints)
        self._idx = 0
        self._arrival_radius_m = arrival_radius_m
        self._fallback_ticks_per_waypoint = max(1, int(fallback_ticks_per_waypoint))
        self._ticks_on_current = 0
        self._loop = loop

    @property
    def index(self) -> int:
        return self._idx

    @property
    def current_target(self) -> Vec3:
        return self._waypoints[self._idx]

    def tick(self, leader_position: Optional[Vec3]) -> None:
        # Hold final waypoint forever when loop=False (no wrap to wp0).
        if not self._loop and self._idx >= len(self._waypoints) - 1:
            return

        self._ticks_on_current += 1

        if leader_position is not None:
            if _dist(leader_position, self.current_target) <= self._arrival_radius_m:
                self._advance()
                return

        if self._ticks_on_current >= self._fallback_ticks_per_waypoint:
            self._advance()

    def _advance(self) -> None:
        if not self._loop and self._idx >= len(self._waypoints) - 1:
            return
        self._idx = (self._idx + 1) % len(self._waypoints)
        self._ticks_on_current = 0


def follower_targets_from_offsets(leader_target: Vec3, follower_offsets: Sequence[Vec3]) -> List[Vec3]:
    return [
        (leader_target[0] + off[0], leader_target[1] + off[1], leader_target[2] + off[2])
        for off in follower_offsets
    ]


def formation_errors(
    leader_position: Vec3,
    follower_positions: Sequence[Vec3],
    follower_offsets: Sequence[Vec3],
) -> List[float]:
    if len(follower_positions) != len(follower_offsets):
        raise ValueError("follower_positions and follower_offsets must have the same length")
    expected = follower_targets_from_offsets(leader_position, follower_offsets)
    return [_dist(actual, desired) for actual, desired in zip(follower_positions, expected)]


def leader_reached_goal(
    leader_position: Optional[Vec3],
    goal: Vec3,
    *,
    tolerance_m: float,
) -> bool:
    return leader_position is not None and _dist(leader_position, goal) <= tolerance_m

