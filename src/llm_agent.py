import json
import os
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand
from groq import Groq
import threading


class FormationNode(Node):
    def __init__(self):
        super().__init__('formation_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        namespaces = ['', '/px4_1', '/px4_2']
        self.system_ids = [1, 2, 3]
        self.traj_pubs = []
        self.offboard_pubs = []
        self.cmd_pubs = []
        self.initial_positions = [
    {'x': 0.0, 'y': 0.0, 'z': -5.0},
    {'x': 2.0, 'y': 0.0, 'z': -5.0},
    {'x': 4.0, 'y': 0.0, 'z': -5.0},
]

        for ns in namespaces:
            self.traj_pubs.append(self.create_publisher(
                TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos))
            self.offboard_pubs.append(self.create_publisher(
                OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos))
            self.cmd_pubs.append(self.create_publisher(
                VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos))

        # Posición inicial — hover en su lugar
        self.posiciones = [
            {'uav': 0, 'x': 0.0, 'y': 0.0, 'z': -5.0},
            {'uav': 1, 'x': 2.0, 'y': 0.0, 'z': -5.0},
            {'uav': 2, 'x': 4.0, 'y': 0.0, 'z': -5.0},
        ]

        self.counter = 0
        self.armed = False
        self.timer = self.create_timer(0.1, self.send_setpoints)
        self.get_logger().info('Formation Node iniciado 🚁🚁🚁')

    def update_formation(self, posiciones):
        new_positions = []

        posiciones_sorted = sorted(posiciones, key=lambda x: x['uav'])

        for i, pos in enumerate(posiciones_sorted):
            init = self.initial_positions[i]

            new_positions.append({
                'uav': i,
                'x': pos['x'] - init['x'],
                'y': pos['y'] - init['y'],
                'z': pos['z'] - init['z']
            })

        self.posiciones = new_positions

        self.get_logger().info("=== DEBUG FORMATION ===")
        self.get_logger().info(f"Iniciales: {self.initial_positions}")
        self.get_logger().info(f"Global: {posiciones}")
        self.get_logger().info(f"Local: {self.posiciones}")

    def send_setpoints(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)

        for i, pos in enumerate(self.posiciones):
            om = OffboardControlMode()
            om.position = True
            om.timestamp = ts
            self.offboard_pubs[i].publish(om)

            msg = TrajectorySetpoint()
            msg.position = [float(pos['x']), float(pos['y']), float(pos['z'])]
            msg.yaw = 0.0
            msg.timestamp = ts
            self.traj_pubs[i].publish(msg)

        if self.counter == 20:
            for i, sid in enumerate(self.system_ids):
                self._send_cmd(i, sid, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info('Offboard activado 🚁')

        if self.counter == 30:
            for i, sid in enumerate(self.system_ids):
                self._send_cmd(i, sid, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('Armando 🔥')
            self.armed = True

        self.counter += 1

    def _send_cmd(self, i, sid, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = sid
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pubs[i].publish(msg)


def get_formation_from_llm(user_command: str, num_uavs: int = 3) -> dict:
    client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

    prompt = f"""Eres un sistema de control de drones militarmente preciso. 
El usuario quiere que {num_uavs} drones formen una figura EXACTAMENTE como describe.

Comando del usuario: "{user_command}"

REGLAS ESTRICTAS:
- Si el usuario dice x=0 o y=0, usa EXACTAMENTE 0
- Si el usuario dice torre o vertical, usa z diferentes y x,y iguales
- Si el usuario dice horizontal, usa z igual y x,y diferentes
- Separa los drones al menos 0.9 metros entre sí
- z negativo = altura (z=-5 es 5m de altura)
- Interpreta LITERALMENTE lo que pide el usuario

Responde SOLO con JSON válido:
{{
  "figura": "nombre de la figura",
  "posiciones": [
    {{"uav": 0, "x": 0.0, "y": 0.0, "z": -5.0}},
    {{"uav": 1, "x": 0.0, "y": 0.0, "z": -8.0}},
    {{"uav": 2, "x": 0.0, "y": 0.0, "z": -11.0}}
  ]
}}"""

    response = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[{"role": "user", "content": prompt}],
        max_tokens=500,
        temperature=0.1
    )

    response_text = response.choices[0].message.content
    json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
    if json_match:
        response_text = json_match.group()

    return json.loads(response_text)


def main():
    rclpy.init()
    node = FormationNode()

    # ROS2 spin en un thread separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n🚁 Agente LLM para control de UAVs")
    print("Los drones despegarán automáticamente en ~3 segundos")
    print("Escribe una formación cuando estén volando")
    print("Escribe 'salir' para aterrizar\n")

    while True:
        try:
            comando = input("b>>> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if comando.lower() == 'salir':
            print("Aterrizando... 🛬")
            break

        if not comando:
            continue

        print(f"\n🤖 Procesando: '{comando}'...")

        formation = get_formation_from_llm(comando)

        print(f"\n✅ Figura: {formation['figura']}")
        print("📍 Posiciones:")
        for pos in formation['posiciones']:
            print(f"   UAV {pos['uav']}: x={pos['x']}, y={pos['y']}, z={pos['z']}")

        confirmar = input("\n¿Ejecutar? (s/n): ")
        if confirmar.lower() == 's':
            node.update_formation(formation['posiciones'])
            print(f"🚁 Formación '{formation['figura']}' activada!")

<<<<<<< HEAD

def execute_formation(posiciones: list):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand

    class FormationNode(Node):
        def __init__(self, posiciones):
            super().__init__('formation_node')
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            namespaces = ['', '/px4_1', '/px4_2']
            system_ids = [1, 2, 3]
            self.traj_pubs = []
            self.offboard_pubs = []
            self.cmd_pubs = []
            self.system_ids = system_ids

            for ns in namespaces:
                self.traj_pubs.append(self.create_publisher(
                    TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', qos))
                self.offboard_pubs.append(self.create_publisher(
                    OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', qos))
                self.cmd_pubs.append(self.create_publisher(
                    VehicleCommand, f'{ns}/fmu/in/vehicle_command', qos))

            self.posiciones = posiciones
            self.counter = 0
            self.timer = self.create_timer(0.1, self.send_setpoints)
            self.get_logger().info('Ejecutando formación 🚁')

        def send_setpoints(self):
            ts = int(self.get_clock().now().nanoseconds / 1000)

            for i, pos in enumerate(self.posiciones):
                # Offboard mode
                om = OffboardControlMode()
                om.position = True
                om.timestamp = ts
                self.offboard_pubs[i].publish(om)

                # Trajectory
                msg = TrajectorySetpoint()
                msg.position = [float(pos['x']), float(pos['y']), float(pos['z'])]
                msg.yaw = 0.0
                msg.timestamp = ts
                self.traj_pubs[i].publish(msg)

            # Retry offboard + arm every 30 ticks so PX4 accepts them whenever ready
            if self.counter >= 20 and self.counter % 30 == 20:
                for i, sid in enumerate(self.system_ids):
                    self._send_cmd(i, sid, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.get_logger().info('Offboard activado 🚁')

            if self.counter >= 25 and self.counter % 30 == 25:
                for i, sid in enumerate(self.system_ids):
                    self._send_cmd(i, sid, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                self.get_logger().info('Armando 🔥')

            self.counter += 1

        def _send_cmd(self, i, sid, command, param1=0.0, param2=0.0):
            from px4_msgs.msg import VehicleCommand
            msg = VehicleCommand()
            msg.command = command
            msg.param1 = param1
            msg.param2 = param2
            msg.target_system = sid
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.cmd_pubs[i].publish(msg)

    rclpy.init()
    node = FormationNode(posiciones)
    print("🚁 Enviando comandos de formación... (Ctrl+C para detener)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
=======
    rclpy.shutdown()
>>>>>>> 8c8d385 (cambios en el agente)


if __name__ == '__main__':
    main()