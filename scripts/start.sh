#!/bin/bash
echo "ROS2 listo."
echo ""
echo "Para ejecutar el agente LLM:"
echo "  source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash"
echo "  python3 /scripts/llm_agent.py"
echo ""
echo "Nota: necesitas GROQ_API_KEY (via .env) para el agente LLM."
tail -f /dev/null