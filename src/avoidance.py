import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ObstacleAabb:
    """
    Axis-aligned bounding box in world frame.
    """

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float


@dataclass
class Ranges:
    front_m: float
    left_m: float
    right_m: float


def _ray_aabb_intersection_2d(
    *,
    origin_xy: Tuple[float, float],
    dir_xy: Tuple[float, float],
    aabb: ObstacleAabb,
    max_range: float,
) -> float:
    """
    2D ray (origin + t*dir, t>=0) against XY AABB slab intersection.
    Returns distance along the ray in meters, or max_range if no hit.
    """
    ox, oy = origin_xy
    dx, dy = dir_xy
    eps = 1e-9

    tmin = 0.0
    tmax = max_range

    if abs(dx) < eps:
        if ox < aabb.min_x or ox > aabb.max_x:
            return max_range
    else:
        tx1 = (aabb.min_x - ox) / dx
        tx2 = (aabb.max_x - ox) / dx
        t1x, t2x = (tx1, tx2) if tx1 <= tx2 else (tx2, tx1)
        tmin = max(tmin, t1x)
        tmax = min(tmax, t2x)
        if tmax < tmin:
            return max_range

    if abs(dy) < eps:
        if oy < aabb.min_y or oy > aabb.max_y:
            return max_range
    else:
        ty1 = (aabb.min_y - oy) / dy
        ty2 = (aabb.max_y - oy) / dy
        t1y, t2y = (ty1, ty2) if ty1 <= ty2 else (ty2, ty1)
        tmin = max(tmin, t1y)
        tmax = min(tmax, t2y)
        if tmax < tmin:
            return max_range

    if tmin < 0.0:
        return max_range
    return float(min(tmin, max_range))


def compute_virtual_ranges(
    *,
    position: Vec3,
    desired_direction_xy: Tuple[float, float],
    obstacles: Sequence[ObstacleAabb],
    max_range_m: float = 30.0,
) -> Ranges:
    """
    Virtual rangefinders (front/left/right) in XY plane.
    The \"front\" direction is defined by desired_direction_xy.
    Each ray returns distance to the closest obstacle among ``obstacles``.
    """
    px, py, _pz = position
    dx, dy = desired_direction_xy
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        # No meaningful direction -> point \"front\" along +X
        fx, fy = 1.0, 0.0
    else:
        fx, fy = dx / norm, dy / norm

    # Left/right vectors (90 deg rotation)
    lx, ly = -fy, fx
    rx, ry = fy, -fx

    if not obstacles:
        return Ranges(front_m=max_range_m, left_m=max_range_m, right_m=max_range_m)

    def _min_hit(dir_xy: Tuple[float, float]) -> float:
        return min(
            _ray_aabb_intersection_2d(
                origin_xy=(px, py),
                dir_xy=dir_xy,
                aabb=o,
                max_range=max_range_m,
            )
            for o in obstacles
        )

    return Ranges(
        front_m=_min_hit((fx, fy)),
        left_m=_min_hit((lx, ly)),
        right_m=_min_hit((rx, ry)),
    )


@dataclass
class AvoidanceParams:
    d_safe: float = 6.0
    d_critical: float = 3.0
    max_lateral_bias: float = 6.0
    bias_rate_per_tick: float = 0.3
    clear_ticks_required: int = 20  # 0.1s tick -> 2s


@dataclass
class AvoidanceState:
    bias_xy: Tuple[float, float] = (0.0, 0.0)
    clear_ticks: int = 0
    active: bool = False


class AvoidanceController:
    def __init__(self, params: Optional[AvoidanceParams] = None):
        self.params = params or AvoidanceParams()
        self._state: Dict[int, AvoidanceState] = {}

    def reset(self) -> None:
        self._state.clear()

    def compute_adjusted_target(
        self,
        *,
        uav_id: int,
        nominal_target: Vec3,
        ranges: Ranges,
    ) -> Vec3:
        st = self._state.setdefault(uav_id, AvoidanceState())

        if ranges.front_m < self.params.d_safe:
            st.active = True
            st.clear_ticks = 0

            # Pick the side with more clearance
            prefer_left = ranges.left_m >= ranges.right_m
            side = 1.0 if prefer_left else -1.0

            # Stronger push when closer
            strength = 1.0
            if ranges.front_m < self.params.d_critical:
                strength = 1.8

            bx, by = st.bias_xy
            by_next = by + side * self.params.bias_rate_per_tick * strength
            by_next = max(-self.params.max_lateral_bias, min(self.params.max_lateral_bias, by_next))
            st.bias_xy = (bx, by_next)

        else:
            # Clear -> decay bias back to 0 smoothly after stable clearance
            st.clear_ticks += 1
            if st.clear_ticks >= self.params.clear_ticks_required:
                bx, by = st.bias_xy
                decay = self.params.bias_rate_per_tick
                if abs(by) <= decay:
                    by = 0.0
                else:
                    by = by - math.copysign(decay, by)
                st.bias_xy = (bx, by)
                st.active = abs(by) > 1e-6

        bx, by = st.bias_xy
        return (nominal_target[0] + bx, nominal_target[1] + by, nominal_target[2])

