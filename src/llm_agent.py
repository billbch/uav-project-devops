import anthropic
import json
import math

def get_formation_from_llm(user_command: str, num_uavs: int = 3) -> list:
    from groq import Groq
    import os

    client = Groq(api_key=os.environ.get("GROQ_API_KEY"))

    prompt = f"""Eres un sistema de control de drones. El usuario quiere que {num_uavs} drones formen una figura.

Comando del usuario: "{user_command}"

Genera las posiciones (x, y, z) para cada dron en metros, partiendo desde el origen (0,0,0).
- Z negativo significa altura (ej: z=-5 es 5 metros de altura)
- Mantén los drones separados al menos 2 metros entre sí
- La formación debe ser visualmente reconocible

Responde SOLO con un JSON válido, sin explicaciones:
{{
  "figura": "nombre de la figura",
  "posiciones": [
    {{"uav": 0, "x": 0.0, "y": 0.0, "z": -5.0}},
    {{"uav": 1, "x": 2.0, "y": 0.0, "z": -5.0}},
    {{"uav": 2, "x": 1.0, "y": 2.0, "z": -5.0}}
  ]
}}"""

    response = client.chat.completions.create(
        model="llama-3.3-70b-versatile",
        messages=[{"role": "user", "content": prompt}],
        max_tokens=500,
        temperature=0.1
    )

    response_text = response.choices[0].message.content
    
    # Limpiar respuesta — extraer solo el JSON
    import re
    json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
    if json_match:
        response_text = json_match.group()
    
    formation = json.loads(response_text)
    return formation

def main():
    print("🚁 Agente LLM para control de UAVs")
    print("Escribe una formación (ej: 'triángulo', 'línea', 'estrella')")
    print("Escribe 'salir' para terminar\n")
    
    while True:
        comando = input(">>> ").strip()
        
        if comando.lower() == 'salir':
            break
            
        if not comando:
            continue
        
        print(f"\n🤖 Procesando: '{comando}'...")
        
        formation = get_formation_from_llm(comando)
        
        print(f"\n✅ Figura: {formation['figura']}")
        print("📍 Posiciones generadas:")
        for pos in formation['posiciones']:
            print(f"   UAV {pos['uav']}: x={pos['x']}, y={pos['y']}, z={pos['z']}")
        
        confirmar = input("\n¿Ejecutar esta formación? (s/n): ")
        if confirmar.lower() == 's':
            execute_formation(formation['posiciones'])


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
                durability=DurabilityPolicy.VOLATILE,
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

            # Armar y activar offboard
            if self.counter == 20:
                for i, sid in enumerate(self.system_ids):
                    self._send_cmd(i, sid, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.get_logger().info('Offboard activado 🚁')

            if self.counter == 30:
                for i, sid in enumerate(self.system_ids):
                    self._send_cmd(i, sid, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
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


if __name__ == '__main__':
    main()