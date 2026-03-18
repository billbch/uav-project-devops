#!/bin/bash
export PX4_GZ_MODELS=/px4/Tools/simulation/gz/models
export PX4_GZ_WORLDS=/px4/Tools/simulation/gz/worlds
export PX4_GZ_WORLD=default

cd /px4

echo "=== Lanzando UAV 0 con make (inicia Gazebo) ==="
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
make px4_sitl gz_x500 &

echo "=== Esperando que Gazebo arranque ==="
sleep 35

echo "=== Lanzando UAV 1 ==="
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="2,0,0,0,0,0" \
PX4_MICRODDS_UDP_PORT=8888 \
PX4_GZ_MODELS=/px4/Tools/simulation/gz/models \
PX4_GZ_WORLDS=/px4/Tools/simulation/gz/worlds \
PX4_GZ_WORLD=default \
PX4_GZ_STANDALONE=1 \
GZ_SIM_RESOURCE_PATH=/px4/Tools/simulation/gz/models \
./build/px4_sitl_default/bin/px4 \
./build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i 1 &

sleep 35

echo "=== Lanzando UAV 2 ==="
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL=x500 \
PX4_GZ_MODEL_POSE="4,0,0,0,0,0" \
PX4_MICRODDS_UDP_PORT=8888 \
PX4_GZ_MODELS=/px4/Tools/simulation/gz/models \
PX4_GZ_WORLDS=/px4/Tools/simulation/gz/worlds \
PX4_GZ_WORLD=default \
PX4_GZ_STANDALONE=1 \
GZ_SIM_RESOURCE_PATH=/px4/Tools/simulation/gz/models \
./build/px4_sitl_default/bin/px4 \
./build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i 2 &

echo "=== 3 drones lanzados ==="

echo "=== Esperando a que PX4 inicialice... ==="
sleep 120

echo "=== Aplicando parametros a todos los UAVs ==="
for i in 0 1 2; do
    /px4/build/px4_sitl_default/bin/px4-param --instance $i set COM_RCL_EXCEPT 7
    /px4/build/px4_sitl_default/bin/px4-param --instance $i set COM_RC_IN_MODE 1
done
echo "=== Parametros aplicados ✅ ==="

wait