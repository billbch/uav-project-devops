import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class UAV:
    def __init__(self, node, namespace, system_id):
        self.namespace = namespace
        self.system_id = system_id
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_pub = node.create_publisher(
            OffboardControlMode, f'{namespace}/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint, f'{namespace}/fmu/in/trajectory_setpoint', qos)
        self.command_pub = node.create_publisher(
            VehicleCommand, f'{namespace}/fmu/in/vehicle_command', qos)

    def send_offboard_mode(self, node):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def send_trajectory(self, node, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def arm(self, node):
        self.publish_command(node, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self, node):
        self.publish_command(node, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def publish_command(self, node, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.system_id  # ← cada dron su propio ID
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


class MultiUAVNode(Node):
    def __init__(self):
        super().__init__('multi_uav_node')

        self.uavs = [
            UAV(self, '',       1),  # UAV 0 → system_id 1
            UAV(self, '/px4_1', 2),  # UAV 1 → system_id 2
            UAV(self, '/px4_2', 3),  # UAV 2 → system_id 3
        ]

        self.positions = [
            (0.0, 0.0, -5.0),
            (2.0, 0.0, -5.0),
            (4.0, 0.0, -5.0),
        ]

        self.counter = 0
        self.timer = self.create_timer(0.1, self.fly)
        self.get_logger().info('MultiUAV Node iniciado 🚁🚁🚁')

    def fly(self):
        for i, uav in enumerate(self.uavs):
            x, y, z = self.positions[i]
            uav.send_offboard_mode(self)
            uav.send_trajectory(self, x, y, z)

        # Primero offboard, luego arm
        if self.counter == 20:
            for uav in self.uavs:
                uav.set_offboard_mode(self)
            self.get_logger().info('Modo Offboard activado en los 3 🚁')

        if self.counter == 30:
            for uav in self.uavs:
                uav.arm(self)
            self.get_logger().info('Armando los 3 drones ✅')

        if self.counter == 100:
            self.get_logger().info('Los 3 drones deberían estar a 5m 🎯')

        self.counter += 1


def main():
    rclpy.init()
    node = MultiUAVNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()