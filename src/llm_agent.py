import json
import math
import os
import re
from typing import Any, Dict, List, Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry,
    VehicleLocalPosition,
)
try:
    # Optional, depends on px4_msgs version
    from px4_msgs.msg import VehicleStatus  # type: ignore
except Exception:  # pragma: no cover
    VehicleStatus = None  # type: ignore
try:
    from px4_msgs.msg import VehicleCommandAck  # type: ignore
except Exception:  # pragma: no cover
    VehicleCommandAck = None  # type: ignore
try:
    from px4_msgs.msg import TimesyncStatus  # type: ignore
except Exception:  # pragma: no cover
    TimesyncStatus = None  # type: ignore
from groq import Groq
import threading
import time

from formation_flight import FormationPlan, WaypointProgressor, follower_targets_from_offsets, validate_plan
from avoidance import AvoidanceController, AvoidanceParams, ObstacleAabb, compute_virtual_ranges
from scenario import (
    CORRIDOR_WAYPOINTS,
    DEFAULT_CRUISE_Z_ENU,
    LINE_FOLLOWER_OFFSETS,
    PX4_NAMESPACES,
    PX4_SYSTEM_IDS,
    SPAWN_POSITIONS_ENU,
    TRIANGLE_DEMO_OFFSETS,
    alliance_to_vehicle_local,
    vehicle_local_to_alliance,
)

# PX4 TrajectorySetpoint: NaN on a component means "do not control this state". Leaving velocity/accel as
# implicit zeros makes the position controller fight zero-velocity constraints while tracking position,
# which in SITL often shows up as bogus vertical motion (odom z drifting while cmd z stays fixed).
_NAN = float("nan")


def _trajectory_position_hold(
    *, x: float, y: float, z: float, yaw: float, timestamp_us: int
) -> TrajectorySetpoint:
    msg = TrajectorySetpoint()
    msg.timestamp = timestamp_us
    msg.position = [float(x), float(y), float(z)]
    msg.velocity = [_NAN, _NAN, _NAN]
    msg.acceleration = [_NAN, _NAN, _NAN]
    # PX4: jerk is logging-only; finite values avoid edge cases on some builds.
    msg.jerk = [0.0, 0.0, 0.0]
    msg.yaw = float(yaw)
    msg.yawspeed = 0.0
    return msg


# Gazebo/SDF model poses are ENU (x=East, y=North, z=Up).
# PX4 offboard setpoints/odometry are NED (x=North, y=East, z=Down).
#
# We keep the *planning / corridor* frame as Gazebo ENU so what you see matches the numbers, and we
# convert ENU↔NED at the PX4 interface boundary.
# Must match docker/inject_obstacle_into_default_world.py corridor (six 6×6 m footprints, gap |y|<3 along +X).
# Leader path: approach from west, traverse the 6 m-wide gap, exit past the eastern pair.
DEBUG_RAM_BUILDING_WAYPOINTS: List[Tuple[float, float, float]] = list(CORRIDOR_WAYPOINTS)


class FormationNode(Node):
    def __init__(self):
        super().__init__('formation_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        namespaces = list(PX4_NAMESPACES)
        self.system_ids = list(PX4_SYSTEM_IDS)
        self.traj_pubs = []
        self.offboard_pubs = []
        self.cmd_pubs = []
        # Gazebo ENU spawn offsets vs leader — MUST match PX4_GZ_MODEL_POSE in scripts/start_px4_multi.sh
        # (first two numbers per drone). Each SITL instance uses its own local NED origin at spawn; waypoints
        # and formation are planned in this shared "alliance ENU" frame (world corridor frame).
        self.initial_positions = [
            {'x': p[0], 'y': p[1], 'z': p[2]} for p in SPAWN_POSITIONS_ENU
        ]

        for ns in namespaces:
            self.traj_pubs.append(self.create_publisher(
                TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos))
            self.offboard_pubs.append(self.create_publisher(
                OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos))
            self.cmd_pubs.append(self.create_publisher(
                VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos))

        # Best-effort subscription to leader odometry (optional; used for waypoint progression)
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self._on_leader_odom,
            odom_qos,
        )
        self.create_subscription(
            VehicleOdometry,
            '/px4_1/fmu/out/vehicle_odometry',
            self._on_follower1_odom,
            odom_qos,
        )
        self.create_subscription(
            VehicleOdometry,
            '/px4_2/fmu/out/vehicle_odometry',
            self._on_follower2_odom,
            odom_qos,
        )

        # PX4 expects trajectory/offboard timestamps in each FCU's time base (µs), not ROS time.
        # Wrong clock → XY can still track while Z integrators drift (odom z runs away from cmd z).
        self._px4_ts_us: List[int] = [0, 0, 0]
        self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self._on_vlp_leader,
            odom_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self._on_vlp_follower1,
            odom_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/px4_2/fmu/out/vehicle_local_position',
            self._on_vlp_follower2,
            odom_qos,
        )

        # Per-vehicle status/ack caches (optional depending on px4_msgs version)
        self._vehicle_status: List[Dict[str, Any]] = [{}, {}, {}]
        self._last_cmd_ack: List[Dict[str, Any]] = [{}, {}, {}]
        self._timesync: List[Dict[str, Any]] = [{}, {}, {}]

        if VehicleStatus is not None:
            self.create_subscription(
                VehicleStatus,
                'fmu/out/vehicle_status',
                lambda m: self._on_vehicle_status(0, m),
                odom_qos,
            )
            self.create_subscription(
                VehicleStatus,
                '/px4_1/fmu/out/vehicle_status',
                lambda m: self._on_vehicle_status(1, m),
                odom_qos,
            )
            self.create_subscription(
                VehicleStatus,
                '/px4_2/fmu/out/vehicle_status',
                lambda m: self._on_vehicle_status(2, m),
                odom_qos,
            )

        if VehicleCommandAck is not None:
            self.create_subscription(
                VehicleCommandAck,
                'fmu/out/vehicle_command_ack',
                lambda m: self._on_vehicle_command_ack(0, m),
                odom_qos,
            )
            self.create_subscription(
                VehicleCommandAck,
                '/px4_1/fmu/out/vehicle_command_ack',
                lambda m: self._on_vehicle_command_ack(1, m),
                odom_qos,
            )
            self.create_subscription(
                VehicleCommandAck,
                '/px4_2/fmu/out/vehicle_command_ack',
                lambda m: self._on_vehicle_command_ack(2, m),
                odom_qos,
            )

        if TimesyncStatus is not None:
            self.create_subscription(
                TimesyncStatus,
                'fmu/out/timesync_status',
                lambda m: self._on_timesync(0, m),
                odom_qos,
            )
            self.create_subscription(
                TimesyncStatus,
                '/px4_1/fmu/out/timesync_status',
                lambda m: self._on_timesync(1, m),
                odom_qos,
            )
            self.create_subscription(
                TimesyncStatus,
                '/px4_2/fmu/out/timesync_status',
                lambda m: self._on_timesync(2, m),
                odom_qos,
            )

        # Posición inicial — hover en su lugar
        self._formation_plan: Optional[FormationPlan] = None
        self._progressor: Optional[WaypointProgressor] = None
        # Default formation: line (can be changed interactively)
        self._default_follower_offsets: List[Tuple[float, float, float]] = list(LINE_FOLLOWER_OFFSETS)
        self._follower_offsets: List[Tuple[float, float, float]] = list(self._default_follower_offsets)
        # Safe default: hover (no motion) until the user loads a plan
        self._leader_waypoints: List[Tuple[float, float, float]] = [(0.0, 0.0, DEFAULT_CRUISE_Z_ENU)]
        self._leader_pos_est: Optional[Tuple[float, float, float]] = None
        self._follower_pos_est: Dict[int, Tuple[float, float, float]] = {}
        self._last_offboard_log_counter = -10_000
        self._last_arm_log_counter = -10_000
        # When False: keep publishing hover setpoints but do NOT advance waypoints (no motion)
        self._motion_enabled = False
        # When False: after last waypoint, hold it (no wrap to index 0).
        self._waypoints_loop = True
        self._wp_arrival_radius_m = 0.6
        # Open-loop wp advance: ~8 s at 20 Hz when odom missing (was 80 @ 10 Hz).
        self._wp_fallback_ticks = 160

        # AABBs for the six corridor buildings (see docker/inject_obstacle_into_default_world.py).
        half = float(os.environ.get("UAV_BUILDING_HALF_XY", "3"))
        hz = float(os.environ.get("UAV_BUILDING_HALF_Z", "6"))
        centers_xy = (
            (12.0, -6.0),
            (24.0, -6.0),
            (36.0, -6.0),
            (12.0, 6.0),
            (24.0, 6.0),
            (36.0, 6.0),
        )
        self._obstacles: Tuple[ObstacleAabb, ...] = tuple(
            ObstacleAabb(
                min_x=cx - half,
                max_x=cx + half,
                min_y=cy - half,
                max_y=cy + half,
                min_z=0.0,
                max_z=2.0 * hz,
            )
            for cx, cy in centers_xy
        )
        # Off by default: restores original “formation + waypoint” behavior. Set UAV_OBSTACLE_AVOIDANCE=1 to enable.
        self._avoidance_enabled = os.environ.get(
            "UAV_OBSTACLE_AVOIDANCE", ""
        ).strip().lower() in ("1", "true", "yes")
        self._avoid = AvoidanceController(AvoidanceParams())
        self._log_pose = os.environ.get("UAV_LOG_POSE", "").strip().lower() in ("1", "true", "yes")
        # When True, only print pose lines while a mission is active (option 2/3). Default False = log whenever UAV_LOG_POSE=1
        self._log_pose_mission_only = os.environ.get(
            "UAV_LOG_POSE_MISSION_ONLY", ""
        ).strip().lower() in ("1", "true", "yes")
        # CSV for analysis (docker cp back to host). Default path when pose logging is on.
        _pf = os.environ.get("UAV_LOG_POSE_FILE", "").strip()
        self._pose_csv_path: Optional[str] = _pf if _pf else (
            "/tmp/uav_pose_mission.csv" if self._log_pose else None
        )
        self._pose_csv_header_done = False

        self.counter = 0
        # 20 Hz: PX4 offboard prefers a healthy setpoint rate; helps Z track vs 10 Hz + bad timestamps.
        self.timer = self.create_timer(0.05, self.send_setpoints)
        self.get_logger().info('Formation Node iniciado 🚁🚁🚁')
        try:
            sp = [(float(p["x"]), float(p["y"])) for p in self.initial_positions]
            print(
                f"[UAV_CFG] spawn_offsets_xy={sp} expected_dds_ports=[8888,8889,8890]",
                flush=True,
            )
        except Exception:
            pass

        # Track whether we are actually receiving critical status/time topics.
        self._last_status_seen_ros_s: List[float] = [0.0, 0.0, 0.0]
        self._last_vlp_seen_ros_s: List[float] = [0.0, 0.0, 0.0]

    def _spawn_xy(self, vehicle_index: int) -> Tuple[float, float]:
        sp = self.initial_positions[vehicle_index]
        return float(sp['x']), float(sp['y'])

    def _vehicle_local_to_alliance(
        self, vehicle_index: int, local_xyz_ned: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Convert per-vehicle local PX4 NED odometry into alliance Gazebo ENU.

        Local PX4 NED: x=North, y=East, z=Down (meters)
        Gazebo ENU:    x=East,  y=North, z=Up   (meters)
        """
        return vehicle_local_to_alliance(vehicle_index, local_xyz_ned)

    def _alliance_to_vehicle_local(
        self, vehicle_index: int, alliance_xyz_enu: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """
        Convert alliance Gazebo ENU target into per-vehicle local PX4 NED setpoint.

        ENU→NED:
        - x_ned (North) = y_enu (North)
        - y_ned (East)  = x_enu (East)
        - z_ned (Down)  = -z_enu (Up)
        """
        return alliance_to_vehicle_local(vehicle_index, alliance_xyz_enu)

    def _on_leader_odom(self, msg: VehicleOdometry) -> None:
        try:
            loc = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
            self._leader_pos_est = self._vehicle_local_to_alliance(0, loc)
        except Exception:
            # Keep best-effort; ignore malformed messages
            return

    def _on_follower1_odom(self, msg: VehicleOdometry) -> None:
        try:
            loc = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
            self._follower_pos_est[1] = self._vehicle_local_to_alliance(1, loc)
        except Exception:
            return

    def _on_follower2_odom(self, msg: VehicleOdometry) -> None:
        try:
            loc = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
            self._follower_pos_est[2] = self._vehicle_local_to_alliance(2, loc)
        except Exception:
            return

    def _vlp_timestamp_us(self, msg: VehicleLocalPosition) -> int:
        t = int(getattr(msg, "timestamp", 0) or 0)
        if t <= 0:
            t = int(getattr(msg, "timestamp_sample", 0) or 0)
        return t

    def _on_vlp_leader(self, msg: VehicleLocalPosition) -> None:
        self._px4_ts_us[0] = self._vlp_timestamp_us(msg)
        self._last_vlp_seen_ros_s[0] = float(self.get_clock().now().nanoseconds) / 1e9

    def _on_vlp_follower1(self, msg: VehicleLocalPosition) -> None:
        self._px4_ts_us[1] = self._vlp_timestamp_us(msg)
        self._last_vlp_seen_ros_s[1] = float(self.get_clock().now().nanoseconds) / 1e9

    def _on_vlp_follower2(self, msg: VehicleLocalPosition) -> None:
        self._px4_ts_us[2] = self._vlp_timestamp_us(msg)
        self._last_vlp_seen_ros_s[2] = float(self.get_clock().now().nanoseconds) / 1e9

    def _on_vehicle_status(self, vehicle_index: int, msg: Any) -> None:
        try:
            self._vehicle_status[vehicle_index] = {
                "timestamp": int(getattr(msg, "timestamp", 0) or 0),
                "nav_state": int(getattr(msg, "nav_state", -1)),
                "arming_state": int(getattr(msg, "arming_state", -1)),
                "failsafe": bool(getattr(msg, "failsafe", False)),
            }
            self._last_status_seen_ros_s[vehicle_index] = float(self.get_clock().now().nanoseconds) / 1e9
        except Exception:
            return

    def _on_vehicle_command_ack(self, vehicle_index: int, msg: Any) -> None:
        try:
            self._last_cmd_ack[vehicle_index] = {
                "timestamp": int(getattr(msg, "timestamp", 0) or 0),
                "command": int(getattr(msg, "command", -1)),
                "result": int(getattr(msg, "result", -1)),
            }
        except Exception:
            return

    def _on_timesync(self, vehicle_index: int, msg: Any) -> None:
        try:
            self._timesync[vehicle_index] = {
                "timestamp": int(getattr(msg, "timestamp", 0) or 0),
                "estimated_offset": int(getattr(msg, "estimated_offset", 0) or 0),
                "round_trip_time": int(getattr(msg, "round_trip_time", 0) or 0),
            }
        except Exception:
            return

    def _leader_ready_for_motion(self) -> bool:
        # If VehicleStatus/VLP are not being bridged in this setup (common), we must not block forever.
        now_s = float(self.get_clock().now().nanoseconds) / 1e9
        status_recent = (now_s - float(self._last_status_seen_ros_s[0])) < 2.0
        vlp_recent = (now_s - float(self._last_vlp_seen_ros_s[0])) < 2.0

        if VehicleStatus is None or not status_recent:
            # Best-effort fallback: require odometry exists. Do NOT require PX4 timestamps.
            return self._leader_pos_est is not None

        st = self._vehicle_status[0] if self._vehicle_status else {}
        nav = int(st.get("nav_state", -1))
        arm = int(st.get("arming_state", -1))
        fs = bool(st.get("failsafe", False))
        if fs:
            return False

        nav_offboard = getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14)
        arm_armed = getattr(VehicleStatus, "ARMING_STATE_ARMED", 2)

        # Require reasonable estimator inputs for leader progression.
        return (
            self._leader_pos_est is not None
            and (self._px4_ts_us[0] > 0 if vlp_recent else True)
            and nav == int(nav_offboard)
            and arm == int(arm_armed)
        )

    def update_plan(self, plan: FormationPlan, *, enable_motion: bool = True) -> None:
        validate_plan(plan, min_separation_m=0.9)
        self._formation_plan = plan
        self._follower_offsets = list(plan.follower_offsets)
        self._leader_waypoints = list(plan.leader_waypoints)
        self._waypoints_loop = True
        self._wp_arrival_radius_m = 0.6
        # Open-loop wp advance: ~8 s at 20 Hz when odom missing (was 80 @ 10 Hz).
        self._wp_fallback_ticks = 160
        self._progressor = WaypointProgressor(
            self._leader_waypoints,
            loop=self._waypoints_loop,
            arrival_radius_m=self._wp_arrival_radius_m,
            fallback_ticks_per_waypoint=self._wp_fallback_ticks,
        )
        self._motion_enabled = enable_motion
        if enable_motion:
            self._avoid.reset()
        self.get_logger().info(f"Plan actualizado. Formación='{plan.formation_name}', waypoints={len(plan.leader_waypoints)}")

    def update_offsets(self, formation_name: str, offsets: List[Tuple[float, float, float]]) -> None:
        plan = FormationPlan(formation_name=formation_name, follower_offsets=offsets, leader_waypoints=self._leader_waypoints)
        validate_plan(plan, min_separation_m=0.9)
        self._formation_plan = plan
        self._follower_offsets = offsets
        self._avoid.reset()
        # Do NOT enable motion when only updating formation
        self.get_logger().info(f"Formación actualizada. Formación='{formation_name}'")

    def update_waypoints(
        self,
        waypoints: List[Tuple[float, float, float]],
        *,
        enable_motion: bool = True,
        loop_waypoints: bool = True,
        arrival_radius_m: float = 0.6,
        fallback_ticks_per_waypoint: int = 160,
    ) -> None:
        if not waypoints:
            raise ValueError("leader_waypoints must be non-empty")
        self._leader_waypoints = waypoints
        self._waypoints_loop = loop_waypoints
        self._wp_arrival_radius_m = float(arrival_radius_m)
        self._wp_fallback_ticks = max(1, int(fallback_ticks_per_waypoint))
        self._progressor = WaypointProgressor(
            self._leader_waypoints,
            loop=self._waypoints_loop,
            arrival_radius_m=self._wp_arrival_radius_m,
            fallback_ticks_per_waypoint=self._wp_fallback_ticks,
        )
        self._motion_enabled = enable_motion
        self._avoid.reset()
        self.get_logger().info(
            f"Misión actualizada. waypoints={len(waypoints)} loop={loop_waypoints} "
            f"r={self._wp_arrival_radius_m} fallticks={self._wp_fallback_ticks}"
        )

    def enable_motion(self) -> None:
        self._motion_enabled = True

    def standby(self, *, reset_formation: bool = True) -> None:
        """
        Pause mission (no waypoint advance).

        If reset_formation (default): restore default line offsets and command the **canonical**
        spawn-line hover — leader at initial_positions[0], followers via offsets at +2m and +4m on X
        (same as at stack startup). Drones should fly back into that line and hold.

        If reset_formation is False: hold at current leader estimate (old behavior).
        """
        self._avoid.reset()
        if reset_formation:
            self._follower_offsets = list(self._default_follower_offsets)
            self._formation_plan = None
            leader_hover = (
                float(self.initial_positions[0]['x']),
                float(self.initial_positions[0]['y']),
                float(self.initial_positions[0]['z']),
            )
            self.update_waypoints([leader_hover], enable_motion=False)
            return
        hold = self._leader_pos_est if self._leader_pos_est is not None else (
            self._progressor.current_target if self._progressor is not None else self._leader_waypoints[0]
        )
        self.update_waypoints([hold], enable_motion=False)

    def send_setpoints(self):
        ts_ros_fallback = int(self.get_clock().now().nanoseconds / 1000)

        if self._progressor is None:
            self._progressor = WaypointProgressor(
                self._leader_waypoints,
                loop=self._waypoints_loop,
                arrival_radius_m=self._wp_arrival_radius_m,
                fallback_ticks_per_waypoint=self._wp_fallback_ticks,
            )

        # Only advance waypoints after the user explicitly enables motion AND the leader is actually ready.
        # Without this, waypoint indices can advance while PX4 is not in OFFBOARD/ARMED yet, making it
        # look like the mission is ignored or the target is "running away".
        if self._motion_enabled:
            if self._leader_ready_for_motion():
                self._progressor.tick(self._leader_pos_est)
        leader_target = self._progressor.current_target
        follower_nominals = follower_targets_from_offsets(leader_target, self._follower_offsets)
        targets = [leader_target]

        # Optional obstacle avoidance (opt-in). Default off = same behavior as before we added the building.
        if not self._motion_enabled:
            self._avoid.reset()
            targets.extend(follower_nominals)
        elif not self._avoidance_enabled:
            self._avoid.reset()
            targets.extend(follower_nominals)
        else:
            for follower_id, nominal in enumerate(follower_nominals, start=1):
                pos = self._follower_pos_est.get(follower_id)
                if pos is None:
                    targets.append(nominal)
                    continue

                desired_dir = (nominal[0] - pos[0], nominal[1] - pos[1])
                ranges = compute_virtual_ranges(
                    position=pos,
                    desired_direction_xy=desired_dir,
                    obstacles=self._obstacles,
                    max_range_m=30.0,
                )
                adjusted = self._avoid.compute_adjusted_target(
                    uav_id=follower_id, nominal_target=nominal, ranges=ranges
                )
                targets.append(adjusted)

        if self._log_pose:
            do_log = self._motion_enabled if self._log_pose_mission_only else True
            if do_log and self.counter % 10 == 0:
                def _xyz(t: Optional[Tuple[float, float, float]]) -> str:
                    if t is None:
                        return "n/a"
                    return f"({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})"

                def _xyz_csv(t: Optional[Tuple[float, float, float]]) -> str:
                    if t is None:
                        return "nan,nan,nan"
                    return f"{t[0]:.6f},{t[1]:.6f},{t[2]:.6f}"

                wp_i = self._progressor.index if self._progressor else -1
                t_sec = self.get_clock().now().nanoseconds / 1e9
                print(
                    f"[UAV_POSE] motion={int(self._motion_enabled)} wp_idx={wp_i} "
                    f"avoid={int(self._avoidance_enabled)} "
                    f"cmd_alliance_xyz: L={_xyz(targets[0])} F1={_xyz(targets[1] if len(targets) > 1 else None)} "
                    f"F2={_xyz(targets[2] if len(targets) > 2 else None)} | "
                    f"nom_xyz: F1={_xyz(follower_nominals[0] if follower_nominals else None)} "
                    f"F2={_xyz(follower_nominals[1] if len(follower_nominals) > 1 else None)} | "
                    f"odom_alliance_xyz: L={_xyz(self._leader_pos_est)} F1={_xyz(self._follower_pos_est.get(1))} "
                    f"F2={_xyz(self._follower_pos_est.get(2))}",
                    flush=True,
                )
                if self._pose_csv_path:
                    try:
                        hdr = (
                            "t_sec,motion,wp_idx,avoid,"
                            "L_cmd_x,L_cmd_y,L_cmd_z,F1_cmd_x,F1_cmd_y,F1_cmd_z,F2_cmd_x,F2_cmd_y,F2_cmd_z,"
                            "F1_nom_x,F1_nom_y,F1_nom_z,F2_nom_x,F2_nom_y,F2_nom_z,"
                            "L_odom_x,L_odom_y,L_odom_z,F1_odom_x,F1_odom_y,F1_odom_z,F2_odom_x,F2_odom_y,F2_odom_z\n"
                        )
                        p = self._pose_csv_path
                        if not self._pose_csv_header_done:
                            if not os.path.exists(p) or os.path.getsize(p) == 0:
                                with open(p, "w", encoding="utf-8") as fh:
                                    fh.write(hdr)
                            self._pose_csv_header_done = True
                        f1c = targets[1] if len(targets) > 1 else None
                        f2c = targets[2] if len(targets) > 2 else None
                        n1 = follower_nominals[0] if follower_nominals else None
                        n2 = follower_nominals[1] if len(follower_nominals) > 1 else None
                        row = (
                            f"{t_sec:.6f},{int(self._motion_enabled)},{wp_i},{int(self._avoidance_enabled)},"
                            + ",".join(
                                [
                                    _xyz_csv(targets[0]),
                                    _xyz_csv(f1c),
                                    _xyz_csv(f2c),
                                    _xyz_csv(n1),
                                    _xyz_csv(n2),
                                    _xyz_csv(self._leader_pos_est),
                                    _xyz_csv(self._follower_pos_est.get(1)),
                                    _xyz_csv(self._follower_pos_est.get(2)),
                                ]
                            )
                            + "\n"
                        )
                        with open(p, "a", encoding="utf-8") as fh:
                            fh.write(row)
                    except OSError:
                        pass

        # High-signal status line (opt-in). Helps distinguish:
        # - not actually OFFBOARD/ARMED
        # - failsafe active
        # - missing PX4 timestamps (ros fallback)
        # - waypoint "running away" (distance not decreasing)
        if os.environ.get("UAV_STATUS_LOG", "").strip().lower() in ("1", "true", "yes"):
            # Default: only print while a mission is running (prevents corrupting the interactive menu).
            mission_only = os.environ.get("UAV_STATUS_LOG_MISSION_ONLY", "").strip().lower() not in ("0", "false", "no")
            if (not mission_only or self._motion_enabled) and self.counter % 20 == 0:
                wp_i = self._progressor.index if self._progressor else -1
                lx, ly, lz = self._leader_pos_est if self._leader_pos_est is not None else (float("nan"), float("nan"), float("nan"))
                tx, ty, tz = leader_target
                dist = math.sqrt((tx - lx) ** 2 + (ty - ly) ** 2 + (tz - lz) ** 2) if self._leader_pos_est is not None else float("nan")

                def _st(i: int) -> str:
                    if VehicleStatus is None:
                        return f"u{i}:st=NA ts={'px4' if self._px4_ts_us[i] > 0 else 'ros'}"
                    st = self._vehicle_status[i] if i < len(self._vehicle_status) else {}
                    nav = st.get("nav_state", -1)
                    arm = st.get("arming_state", -1)
                    fs = int(bool(st.get('failsafe', False)))
                    ts_src = "px4" if self._px4_ts_us[i] > 0 else "ros"
                    return f"u{i}:nav={nav} arm={arm} fs={fs} ts={ts_src}"

                ack = self._last_cmd_ack[0] if self._last_cmd_ack else {}
                ack_s = f"ack(cmd={ack.get('command','?')},res={ack.get('result','?')})" if ack else "ack(n/a)"

                ready = int(self._leader_ready_for_motion())
                print(
                    f"[UAV_STATUS] motion={int(self._motion_enabled)} ready={ready} wp={wp_i} "
                    f"d={dist:.2f} | {_st(0)} {_st(1)} {_st(2)} | {ack_s}",
                    flush=True,
                )

        for i, tgt in enumerate(targets):
            ts_i = (
                int(self._px4_ts_us[i])
                if i < len(self._px4_ts_us) and self._px4_ts_us[i] > 0
                else ts_ros_fallback
            )
            om = OffboardControlMode()
            om.timestamp = ts_i
            om.position = True
            om.velocity = False
            om.acceleration = False
            om.attitude = False
            om.body_rate = False
            if hasattr(om, "actuator"):
                om.actuator = False
            self.offboard_pubs[i].publish(om)

            loc = self._alliance_to_vehicle_local(i, tgt)
            msg = _trajectory_position_hold(
                x=loc[0], y=loc[1], z=loc[2], yaw=0.0, timestamp_us=ts_i
            )
            self.traj_pubs[i].publish(msg)

        # Retry offboard + arm periodically so PX4 accepts whenever ready (timer is 20 Hz → scale ticks).
        if self.counter >= 40 and self.counter % 60 == 40:
            for i, sid in enumerate(self.system_ids):
                ts_i = (
                    int(self._px4_ts_us[i])
                    if i < len(self._px4_ts_us) and self._px4_ts_us[i] > 0
                    else ts_ros_fallback
                )
                self._send_cmd(
                    i,
                    sid,
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                    timestamp_us=ts_i,
                )
            # Keep retries silent (prevents drowning the interactive CLI)

        if self.counter >= 50 and self.counter % 60 == 50:
            for i, sid in enumerate(self.system_ids):
                # param2=21196 is the common PX4 "force arm" magic number used in sim setups
                ts_i = (
                    int(self._px4_ts_us[i])
                    if i < len(self._px4_ts_us) and self._px4_ts_us[i] > 0
                    else ts_ros_fallback
                )
                self._send_cmd(
                    i,
                    sid,
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                    21196.0,
                    timestamp_us=ts_i,
                )
            # Keep retries silent (prevents drowning the interactive CLI)

        self.counter += 1

    def _send_cmd(self, i, sid, command, param1=0.0, param2=0.0, *, timestamp_us: Optional[int] = None):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = sid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = (
            int(timestamp_us)
            if timestamp_us is not None and int(timestamp_us) > 0
            else int(self.get_clock().now().nanoseconds / 1000)
        )
        self.cmd_pubs[i].publish(msg)


def _extract_json_object(text: str) -> str:
    json_match = re.search(r'\{.*\}', text, re.DOTALL)
    return json_match.group() if json_match else text


def _get_groq_client() -> Groq:
    api_key = os.environ.get("GROQ_API_KEY")
    if not api_key:
        raise RuntimeError("GROQ_API_KEY is missing. Set it in .env or export it in the environment.")
    return Groq(api_key=api_key)


def _safe_json_loads(text: str) -> Dict[str, Any]:
    candidate = _extract_json_object(text or "").strip()
    if not candidate:
        raise ValueError("LLM returned empty response (no JSON).")
    try:
        return json.loads(candidate)
    except json.JSONDecodeError as e:
        preview = candidate.replace("\n", " ")[:300]
        raise ValueError(f"LLM response was not valid JSON. Preview: {preview!r}") from e


def _groq_chat_with_retries(*, system: str, user: str, max_tokens: int, attempts: int = 3) -> str:
    client = _get_groq_client()
    last_err: Optional[Exception] = None
    for attempt in range(1, attempts + 1):
        try:
            try:
                response = client.chat.completions.create(
                    model="llama-3.3-70b-versatile",
                    messages=[
                        {"role": "system", "content": system},
                        {"role": "user", "content": user},
                    ],
                    max_tokens=max_tokens,
                    temperature=0,
                    response_format={"type": "json_object"},
                )
            except TypeError:
                response = client.chat.completions.create(
                    model="llama-3.3-70b-versatile",
                    messages=[
                        {"role": "system", "content": system},
                        {"role": "user", "content": user},
                    ],
                    max_tokens=max_tokens,
                    temperature=0,
                )
            return (response.choices[0].message.content or "").strip()
        except Exception as e:
            last_err = e
            if attempt < attempts:
                time.sleep(1.0 * attempt)
                continue
            raise RuntimeError(f"Groq request failed after {attempts} attempts: {e}") from e
    raise RuntimeError(f"Groq request failed: {last_err}")


_JSON_ONLY_SYSTEM = (
    "You are a strict JSON generator for a robot API. "
    "Output exactly one JSON object and nothing else — no markdown, no code fences, no explanations, no leading text."
)

# Injected into LLM waypoint planners so missions can reach x≈40 and stay in the 6 m gap (|y|<3 between walls).
_CORRIDOR_WORLD_HINT = """\
World layout (meters): leader hover / spawn near (0,0). Six static buildings: two rows facing each other —
south row centerline y=-6, north row y=+6, with building centers at x=12, x=24, and x=36. Each footprint is 6m×6m
(inner wall faces at y=±3). Open corridor along +X through y≈0 from roughly x=9 to x=39.
For a safe transit: use z=+6 (6m AGL, about half the 12m building height), keep leader |y|<=2.0 while 8<=x<=40, and use several
waypoints with x increasing (e.g. 3→10→18→26→34→41) so the path does not cut corners into the walls.
Follower offsets: prefer narrow formation in y (small |dy|) so all three UAVs stay in the gap; min spacing 0.9m.
You may use leader x in [0,45] and y in [-4,4] when the mission requires it outside the corridor."""


def get_offsets_from_llm(formation_command: str) -> Tuple[str, List[Tuple[float, float, float]]]:
    """
    Returns: (formation_name, [(dx,dy,dz) for uav1, (dx,dy,dz) for uav2])
    """
    user = f"""Multi-UAV formation planner.

3 UAVs. UAV0 is leader. UAV1 and UAV2 use offsets relative to UAV0.

User formation request: {formation_command!r}

Rules:
- Units: meters.
- Offsets are dx,dy,dz relative to UAV0.
- Minimum spacing between any pair among UAV0 (0,0,0), UAV1 offset, UAV2 offset must be >= 0.9 m.
- Do not explain. Do not use prose.

Return JSON with this exact shape:
{{"formation":{{"name":"string","follower_offsets":[{{"uav":1,"dx":0,"dy":0,"dz":0}},{{"uav":2,"dx":0,"dy":0,"dz":0}}]}}}}"""
    raw = _groq_chat_with_retries(system=_JSON_ONLY_SYSTEM, user=user, max_tokens=400, attempts=4)
    payload: Dict[str, Any] = _safe_json_loads(raw)
    formation = payload.get("formation") or {}
    name = str(formation.get("name") or "formation")
    offsets_raw = formation.get("follower_offsets") or []
    offsets_by_uav: Dict[int, Tuple[float, float, float]] = {}
    for item in offsets_raw:
        uav = int(item["uav"])
        offsets_by_uav[uav] = (float(item["dx"]), float(item["dy"]), float(item["dz"]))
    offsets = [offsets_by_uav[1], offsets_by_uav[2]]
    return name, offsets


def get_waypoints_from_llm(mission_command: str) -> List[Tuple[float, float, float]]:
    user = f"""UAV waypoint planner for UAV0 (leader).

{_CORRIDOR_WORLD_HINT}

Mission: {mission_command!r}

Rules:
- JSON only.
- z is ENU Up (z=+6 is 6m AGL; use this for corridor missions so UAVs are mid-height vs 12m buildings).
- 4 to 12 waypoints.
- z roughly in [4,12].

Shape: {{"leader_waypoints":[{{"x":0,"y":0,"z":6}},{{"x":5,"y":0,"z":6}}]}}"""
    raw = _groq_chat_with_retries(system=_JSON_ONLY_SYSTEM, user=user, max_tokens=900, attempts=4)
    payload: Dict[str, Any] = _safe_json_loads(raw)
    wps_raw = payload.get("leader_waypoints") or []
    waypoints = [(float(wp["x"]), float(wp["y"]), float(wp["z"])) for wp in wps_raw]

    if os.environ.get("UAV_VALIDATE_CORRIDOR", "").strip().lower() in ("1", "true", "yes"):
        fixed: List[Tuple[float, float, float]] = []
        warned = False
        for (x, y, z) in waypoints:
            if 8.0 <= x <= 40.0 and abs(y) > 2.0:
                warned = True
                y = max(-2.0, min(2.0, y))
            fixed.append((x, y, z))
        if warned:
            print(
                "[UAV_VALIDATE_CORRIDOR] Clamped some waypoint y into [-2,2] while x in [8,40].",
                flush=True,
            )
        waypoints = fixed

    return waypoints


def get_plan_from_llm(formation_command: str, mission_command: str) -> FormationPlan:
    user = f"""Multi-UAV mission planner (PX4 SITL). UAV0 leader; UAV1/UAV2 follower offsets vs leader.

{_CORRIDOR_WORLD_HINT}

Formation: {formation_command!r}
Leader mission: {mission_command!r}

Rules:
- JSON only; meters; z is ENU Up (z=+6 is 6m AGL for corridor missions).
- Follower offsets vs leader; min 0.9m spacing vs (0,0,0) and between followers.
- 4 to 12 leader waypoints.

Shape:
{{"formation":{{"name":"str","follower_offsets":[{{"uav":1,"dx":0,"dy":0,"dz":0}},{{"uav":2,"dx":0,"dy":0,"dz":0}}]}},"leader_waypoints":[{{"x":0,"y":0,"z":6}}]}}"""

    raw = _groq_chat_with_retries(system=_JSON_ONLY_SYSTEM, user=user, max_tokens=1100, attempts=4)
    payload: Dict[str, Any] = _safe_json_loads(raw)

    formation = payload.get("formation") or {}
    name = str(formation.get("name") or "formation")

    offsets_raw = formation.get("follower_offsets") or []
    offsets_by_uav: Dict[int, Tuple[float, float, float]] = {}
    for item in offsets_raw:
        uav = int(item["uav"])
        offsets_by_uav[uav] = (float(item["dx"]), float(item["dy"]), float(item["dz"]))

    follower_offsets: List[Tuple[float, float, float]] = [
        offsets_by_uav[1],
        offsets_by_uav[2],
    ]

    wps_raw = payload.get("leader_waypoints") or []
    leader_waypoints: List[Tuple[float, float, float]] = [
        (float(wp["x"]), float(wp["y"]), float(wp["z"])) for wp in wps_raw
    ]

    if os.environ.get("UAV_VALIDATE_CORRIDOR", "").strip().lower() in ("1", "true", "yes"):
        fixed: List[Tuple[float, float, float]] = []
        warned = False
        for (x, y, z) in leader_waypoints:
            if 8.0 <= x <= 40.0 and abs(y) > 2.0:
                warned = True
                y = max(-2.0, min(2.0, y))
            fixed.append((x, y, z))
        if warned:
            print(
                "[UAV_VALIDATE_CORRIDOR] Clamped some waypoint y into [-2,2] while x in [8,40].",
                flush=True,
            )
        leader_waypoints = fixed

    return FormationPlan(formation_name=name, follower_offsets=follower_offsets, leader_waypoints=leader_waypoints)


def main():
    rclpy.init()
    node = FormationNode()
    # Silence INFO logs so the interactive prompt isn't corrupted by background retries.
    # (Errors will still show.)
    try:
        rclpy.logging.set_logger_level('formation_node', LoggingSeverity.WARN)
    except Exception:
        pass

    # ROS2 spin en un thread separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\nAgente LLM (3 UAVs) — comportamiento esperado:")
    print("- Los drones pasan a offboard y arman solos, y se quedan en hover.")
    print("- Solo se mueven cuando tú habilitas una misión.\n")
    print("Opciones:")
    print("  1) Definir/actualizar formación (offsets) — NO mueve")
    print("  2) Definir misión (waypoints) y empezar a volar — mueve en la formación actual")
    print("  3) Definir formación + misión y empezar a volar")
    print("  4) Standby (hover / pausar movimiento)")
    print("  5) DEBUG: líder +X por el pasillo entre edificios (sin LLM)")
    print("  6) Demo: triángulo + mismo pasillo (un solo paso)")
    print("  salir) terminar\n")

    try:
        while True:
            try:
                choice = input("opción (1/2/3/4/5/6/salir)>>> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == 'salir':
                break

            if not choice:
                continue

            if choice == '1':
                formation_cmd = input("formación (ej: triangulo)>>> ").strip()
                if not formation_cmd or formation_cmd.lower() == 'salir':
                    continue

                print("\nGenerando offsets con LLM...")
                try:
                    name, offsets = get_offsets_from_llm(formation_cmd)
                except Exception as e:
                    print(f"\nError generando offsets: {e}")
                    print("Intenta de nuevo.")
                    continue
                print(f"\nFormación: {name}")
                print("Offsets relativos al líder:")
                print(f"  UAV1: dx={offsets[0][0]}, dy={offsets[0][1]}, dz={offsets[0][2]}")
                print(f"  UAV2: dx={offsets[1][0]}, dy={offsets[1][1]}, dz={offsets[1][2]}")

                confirmar = input("\n¿Aplicar formación? (s/n): ").strip().lower()
                if confirmar == 's':
                    node.update_offsets(name, offsets)
                    print("Formación aplicada. Los drones siguen en hover.")
                continue

            if choice == '2':
                mission_cmd = input("misión (ej: vuela un cuadrado a 5m)>>> ").strip()
                if not mission_cmd or mission_cmd.lower() == 'salir':
                    continue

                print("\nGenerando waypoints con LLM...")
                try:
                    wps = get_waypoints_from_llm(mission_cmd)
                except Exception as e:
                    print(f"\nError generando waypoints: {e}")
                    print("Intenta de nuevo.")
                    continue
                print(f"\nWaypoints líder: {len(wps)}")
                confirmar = input("\n¿Empezar misión? (s/n): ").strip().lower()
                if confirmar == 's':
                    node.update_waypoints(wps, enable_motion=True)
                    print("Misión aplicada. Formación volando.")
                continue

            if choice == '3':
                formation_cmd = input("formación (ej: triangulo)>>> ").strip()
                if not formation_cmd or formation_cmd.lower() == 'salir':
                    continue
                mission_cmd = input("misión (ej: vuela un cuadrado a 5m)>>> ").strip()
                if not mission_cmd or mission_cmd.lower() == 'salir':
                    continue

                print("\nGenerando plan con LLM...")
                try:
                    plan = get_plan_from_llm(formation_cmd, mission_cmd)
                except Exception as e:
                    print(f"\nError generando plan: {e}")
                    print("Intenta de nuevo.")
                    continue

                print(f"\nFormación: {plan.formation_name}")
                print("Offsets relativos al líder:")
                print(f"  UAV1: dx={plan.follower_offsets[0][0]}, dy={plan.follower_offsets[0][1]}, dz={plan.follower_offsets[0][2]}")
                print(f"  UAV2: dx={plan.follower_offsets[1][0]}, dy={plan.follower_offsets[1][1]}, dz={plan.follower_offsets[1][2]}")
                print(f"Waypoints líder: {len(plan.leader_waypoints)}")

                confirmar = input("\n¿Empezar? (s/n): ").strip().lower()
                if confirmar == 's':
                    node.update_plan(plan, enable_motion=True)
                    print("Plan aplicado. Formación volando.")
                continue

            if choice == '4':
                node.standby()
                print("Standby activado. Movimiento pausado; drones en hover.")
                continue

            if choice == '5':
                print(
                    f"\nDEBUG — Líder en línea recta +X por el pasillo (y≈0, z={DEFAULT_CRUISE_Z_ENU}m ENU-Up ≈ mitad altura edificios).\n"
                    "2 waypoints: (4,0) → (40,0) a esa z. Sin bucle. Formación actual (p. ej. triángulo vía opción 1).\n"
                    "UAV_OBSTACLE_AVOIDANCE=0 suele bastar si y≈0; enciende =1 si quieres repeler followers.\n"
                )
                for i, w in enumerate(DEBUG_RAM_BUILDING_WAYPOINTS):
                    print(f"  wp{i}: ({w[0]:.1f}, {w[1]:.1f}, {w[2]:.1f})")
                confirmar = input("\n¿Empezar esta misión DEBUG? (s/n): ").strip().lower()
                if confirmar == 's':
                    node.update_waypoints(
                        list(DEBUG_RAM_BUILDING_WAYPOINTS),
                        enable_motion=True,
                        loop_waypoints=False,
                        arrival_radius_m=2.5,
                        fallback_ticks_per_waypoint=1_000_000,
                    )
                    print("Misión DEBUG aplicada. Pasillo entre filas norte/sur; cámara +X desde el spawn.")
                continue

            if choice == '6':
                print(
                    "\nDemo — Triángulo (~2 m lado) + travesía +X por el pasillo entre edificios.\n"
                    "Aplica offsets y arranca la misión DEBUG en un paso.\n"
                )
                confirmar = input("¿Empezar demo? (s/n): ").strip().lower()
                if confirmar == 's':
                    node.update_offsets("triangulo_demo", list(TRIANGLE_DEMO_OFFSETS))
                    node.update_waypoints(
                        list(DEBUG_RAM_BUILDING_WAYPOINTS),
                        enable_motion=True,
                        loop_waypoints=False,
                        arrival_radius_m=2.5,
                        fallback_ticks_per_waypoint=1_000_000,
                    )
                    print("Demo en curso: triángulo atravesando el pasillo en +X.")
                continue

            print("Opción inválida. Usa 1, 2, 3, 4, 5, 6 o 'salir'.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()