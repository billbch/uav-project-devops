import anthropic
import json
import math

def get_formation_from_llm(user_command: str, num_uavs: int = 3) -> list:
    """Convierte lenguaje natural en coordenadas de formación usando Claude."""
    
    client = anthropic.Anthropic()
    
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

    message = client.messages.create(
        model="claude-opus-4-5",
        max_tokens=500,
        messages=[{"role": "user", "content": prompt}]
    )
    
    response_text = message.content[0].text
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
    """Ejecuta la formación mandando comandos a los drones via ROS2."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from px4_msgs.msg import TrajectorySetpoint

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
            self.publishers = []
            for ns in namespaces:
                pub = self.create_publisher(
                    TrajectorySetpoint,
                    f'{ns}/fmu/in/trajectory_setpoint',
                    qos
                )
                self.publishers.append(pub)
            
            self.posiciones = posiciones
            self.timer = self.create_timer(0.1, self.send_setpoints)
            self.get_logger().info('Ejecutando formación 🚁')

        def send_setpoints(self):
            for i, pos in enumerate(self.posiciones):
                msg = TrajectorySetpoint()
                msg.position = [float(pos['x']), float(pos['y']), float(pos['z'])]
                msg.yaw = 0.0
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.publishers[i].publish(msg)

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