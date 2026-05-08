import math
from typing import Sequence, Tuple


Vec3 = Tuple[float, float, float]

# Gazebo/SDF model poses are ENU: x=East, y=North, z=Up.
DEFAULT_CRUISE_Z_ENU = 6.0

# Known-map corridor scenario: six 6x6x12 m buildings with a gap around y=0.
CORRIDOR_WAYPOINTS: Tuple[Vec3, ...] = (
    (4.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (10.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (16.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (22.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (28.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (34.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (40.0, 0.0, DEFAULT_CRUISE_Z_ENU),
)

LINE_FOLLOWER_OFFSETS: Tuple[Vec3, ...] = (
    (2.0, 0.0, 0.0),
    (4.0, 0.0, 0.0),
)

TRIANGLE_DEMO_OFFSETS: Tuple[Vec3, ...] = (
    (2.0, 0.0, 0.0),
    (1.0, math.sqrt(3.0), 0.0),
)

SPAWN_POSITIONS_ENU: Tuple[Vec3, ...] = (
    (0.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (2.0, 0.0, DEFAULT_CRUISE_Z_ENU),
    (4.0, 0.0, DEFAULT_CRUISE_Z_ENU),
)

PX4_NAMESPACES: Tuple[str, ...] = ("", "/px4_1", "/px4_2")
PX4_SYSTEM_IDS: Tuple[int, ...] = (1, 2, 3)
DECENTRALIZED_LEADER_TARGET_TOPIC = "/decentralized/leader_target"

CORRIDOR_X_MIN = 8.0
CORRIDOR_X_MAX = 40.0
CORRIDOR_Y_ABS_LIMIT = 2.8
CORRIDOR_Z_MIN = 0.5
CORRIDOR_Z_MAX = 11.5


def distance(a: Vec3, b: Vec3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def vehicle_local_to_alliance(vehicle_index: int, local_xyz_ned: Vec3) -> Vec3:
    """
    Convert per-vehicle PX4 local NED odometry into the shared Gazebo ENU frame.
    Each SITL instance has its own local origin at its Gazebo spawn pose.
    """
    nx, ey, dz = local_xyz_ned
    sx, sy, _sz = SPAWN_POSITIONS_ENU[vehicle_index]
    return (ey + sx, nx + sy, -dz)


def alliance_to_vehicle_local(vehicle_index: int, alliance_xyz_enu: Vec3) -> Vec3:
    """
    Convert a shared Gazebo ENU target into one vehicle's PX4 local NED setpoint.
    """
    ex, ny, uz = alliance_xyz_enu
    sx, sy, _sz = SPAWN_POSITIONS_ENU[vehicle_index]
    return (ny - sy, ex - sx, -uz)


def is_in_corridor_safe_region(position: Vec3) -> bool:
    x, y, z = position
    if not (CORRIDOR_X_MIN <= x <= CORRIDOR_X_MAX):
        return True
    return abs(y) <= CORRIDOR_Y_ABS_LIMIT and CORRIDOR_Z_MIN <= z <= CORRIDOR_Z_MAX


def all_finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(v)) for v in values)
