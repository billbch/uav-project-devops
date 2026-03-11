#!/bin/bash
echo "Esperando que PX4 y DDS estén listos..."
sleep 20
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
echo "Lanzando script de despegue..."
python3 /scripts/takeoff.py
