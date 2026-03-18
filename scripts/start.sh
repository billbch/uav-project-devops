#!/bin/bash
echo "Esperando que PX4 y DDS estén listos..."
sleep 150
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
echo "Lanzando agente LLM..."
export GROQ_API_KEY="${GROQ_API_KEY}"
python3 /scripts/llm_agent.py