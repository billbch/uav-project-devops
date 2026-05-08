import argparse
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
)
from std_msgs.msg import Float32MultiArray

from formation_flight import WaypointProgressor
from scenario import (
    CORRIDOR_WAYPOINTS,
    DEFAULT_CRUISE_Z_ENU,
    LINE_FOLLOWER_OFFSETS,
    PX4_NAMESPACES,
    PX4_SYSTEM_IDS,
    SPAWN_POSITIONS_ENU,
    Vec3,
    alliance_to_vehicle_local,
    vehicle_local_to_alliance,
)


_NAN = float("nan")


def _trajectory_position_hold(
    *, x: float, y: float, z: float, yaw: float, timestamp_us: int
) -> TrajectorySetpoint:
    msg = TrajectorySetpoint()
    msg.timestamp = timestamp_us
    msg.position = [float(x), float(y), float(z)]
    msg.velocity = [_NAN, _NAN, _NAN]
    msg.acceleration = [_NAN, _NAN, _NAN]
    msg.jerk = [0.0, 0.0, 0.0]
    msg.yaw = float(yaw)
    msg.yawspeed = 0.0
    return msg


class LocalUavController(Node):
    """
    One local controller for one UAV.

    UAV0 tracks the known corridor route after takeoff/settle. UAV1 and UAV2
    each track UAV0 with fixed offsets. No controller in this baseline computes
    all three final setpoints.
    """

    def __init__(self, uav_id: int):
        if uav_id not in (0, 1, 2):
            raise ValueError("uav_id must be 0, 1, or 2")
        super().__init__(f"uav{uav_id}_local_controller")
        self.uav_id = int(uav_id)
        self.namespace = PX4_NAMESPACES[self.uav_id]
        self.system_id = PX4_SYSTEM_IDS[self.uav_id]
        self._own_pos: Optional[Vec3] = None
        self._leader_pos: Optional[Vec3] = None
        self._leader_target: Optional[Vec3] = None
        self._last_target: Optional[Vec3] = None
        self._px4_ts_us = 0
        self._counter = 0
        self._settle_ticks = 0
        self._mission_started = False
        self._progressor = WaypointProgressor(
            CORRIDOR_WAYPOINTS,
            loop=False,
            arrival_radius_m=1.2,
            fallback_ticks_per_waypoint=1_000_000,
        )

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, f"{self.namespace}/fmu/in/trajectory_setpoint", cmd_qos
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f"{self.namespace}/fmu/in/offboard_control_mode", cmd_qos
        )
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"{self.namespace}/fmu/in/vehicle_command", cmd_qos
        )
        self.reference_pub = self.create_publisher(
            Float32MultiArray, "/uav_eval/decentralized/leader_reference", 10
        )

        self.create_subscription(
            VehicleOdometry,
            f"{self.namespace}/fmu/out/vehicle_odometry",
            lambda msg: self._on_odom(self.uav_id, msg, own=True),
            odom_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"{self.namespace}/fmu/out/vehicle_local_position",
            self._on_vlp,
            odom_qos,
        )

        if self.uav_id > 0:
            self.create_subscription(
                VehicleOdometry,
                f"{PX4_NAMESPACES[0]}/fmu/out/vehicle_odometry",
                lambda msg: self._on_odom(0, msg, own=False),
                odom_qos,
            )
            self.create_subscription(
                Float32MultiArray,
                "/uav_eval/decentralized/leader_reference",
                self._on_leader_reference,
                10,
            )

        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Local controller started for UAV{self.uav_id}")

    def _on_odom(self, vehicle_index: int, msg: VehicleOdometry, *, own: bool) -> None:
        try:
            loc = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
            pos = vehicle_local_to_alliance(vehicle_index, loc)
        except Exception:
            return
        if own:
            self._own_pos = pos
        else:
            self._leader_pos = pos

    def _on_leader_reference(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            return
        self._leader_target = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _on_vlp(self, msg: VehicleLocalPosition) -> None:
        t = int(getattr(msg, "timestamp", 0) or 0)
        if t <= 0:
            t = int(getattr(msg, "timestamp_sample", 0) or 0)
        self._px4_ts_us = t

    def _timestamp_us(self) -> int:
        if self._px4_ts_us > 0:
            return int(self._px4_ts_us)
        return int(self.get_clock().now().nanoseconds / 1000)

    def _target_alliance(self) -> Vec3:
        hover = SPAWN_POSITIONS_ENU[self.uav_id]
        if not self._mission_started:
            if not self._is_airborne_and_stable():
                self._settle_ticks = 0
                return hover

            self._settle_ticks += 1
            if self._settle_ticks < 80:
                return hover

            self._mission_started = True

        if self.uav_id == 0:
            if self._own_pos is not None:
                self._progressor.tick(self._own_pos)
            target = self._progressor.current_target
            self._publish_leader_reference(target)
            return self._limit_target_step(target)

        if self._leader_target is None and self._leader_pos is None:
            return hover

        offset = LINE_FOLLOWER_OFFSETS[self.uav_id - 1]
        leader_ref = self._leader_target if self._leader_target is not None else self._leader_pos
        assert leader_ref is not None
        # Fixed local rule: each follower independently computes its own setpoint from the leader reference.
        desired = (
            leader_ref[0] + offset[0],
            leader_ref[1] + offset[1],
            leader_ref[2] + offset[2],
        )
        return self._limit_target_step(desired)

    def _publish_leader_reference(self, target: Vec3) -> None:
        msg = Float32MultiArray()
        msg.data = [float(target[0]), float(target[1]), float(target[2])]
        self.reference_pub.publish(msg)

    def _limit_target_step(self, desired: Vec3) -> Vec3:
        if self._last_target is None:
            self._last_target = desired
            return desired

        max_step_m = 0.35
        dx = desired[0] - self._last_target[0]
        dy = desired[1] - self._last_target[1]
        dz = desired[2] - self._last_target[2]
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        if dist <= max_step_m or dist < 1e-9:
            self._last_target = desired
            return desired

        scale = max_step_m / dist
        limited = (
            self._last_target[0] + dx * scale,
            self._last_target[1] + dy * scale,
            self._last_target[2] + dz * scale,
        )
        self._last_target = limited
        return limited

    def _is_airborne_and_stable(self) -> bool:
        if self._own_pos is None:
            return False
        x, y, z = self._own_pos
        hx, hy, hz = SPAWN_POSITIONS_ENU[self.uav_id]
        return (
            z >= DEFAULT_CRUISE_Z_ENU - 0.5
            and abs(x - hx) <= 1.5
            and abs(y - hy) <= 1.5
            and abs(z - hz) <= 1.0
        )

    def _tick(self) -> None:
        ts = self._timestamp_us()
        target = self._target_alliance()

        offboard = OffboardControlMode()
        offboard.timestamp = ts
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        if hasattr(offboard, "actuator"):
            offboard.actuator = False
        self.offboard_pub.publish(offboard)

        local = alliance_to_vehicle_local(self.uav_id, target)
        self.traj_pub.publish(
            _trajectory_position_hold(
                x=local[0], y=local[1], z=local[2], yaw=0.0, timestamp_us=ts
            )
        )

        if self._counter >= 40 and self._counter % 60 == 40:
            self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, timestamp_us=ts)
        if self._counter >= 50 and self._counter % 60 == 50:
            self._send_cmd(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                21196.0,
                timestamp_us=ts,
            )

        self._counter += 1

    def _send_cmd(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        *,
        timestamp_us: int,
    ) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.system_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(timestamp_us)
        self.cmd_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run one decentralized local UAV controller.")
    parser.add_argument("--uav-id", type=int, required=True, choices=(0, 1, 2))
    parser.add_argument("--scenario", default="corridor", choices=("corridor",))
    args = parser.parse_args()

    rclpy.init()
    node = LocalUavController(args.uav_id)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
