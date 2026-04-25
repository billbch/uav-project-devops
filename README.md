## Agentic Multi-UAV Coordination

This project simulates **three UAVs** in a known corridor scenario and compares two coordination architectures:

- **Centralized control:** one ROS2 controller computes the mission route and formation setpoints for all UAVs.
- **Decentralized baseline:** each UAV runs a local controller and computes its own setpoint from fixed coordination rules.

The goal is to evaluate both approaches under the same **Gazebo/PX4/ROS2/Docker** setup using:

- Mission success
- Mission time
- Average formation error
- Maximum formation error

The final comparison shows that both approaches complete the corridor mission, while the centralized controller is faster and more accurate.

## Technology Roles

| Technology | Role |
|---|---|
| Docker Compose | Starts the complete reproducible stack. |
| Gazebo | Simulates the 3D world, UAV models, and corridor environment. |
| PX4 SITL | Runs one simulated flight controller per UAV. |
| Micro XRCE-DDS Agent | Bridges each PX4 instance to ROS2 topics. |
| ROS2 Jazzy | Runs the Python control and evaluation nodes. |
| `px4_msgs` | Provides ROS2 message types for PX4 offboard control. |
| Python | Implements centralized control, decentralized control, metrics, and experiment runner. |
| Groq / LLM | Optional interactive mission/formation generation in `llm_agent.py`; not used in the reproducible corridor comparison. |

## Main Components

| Path | Purpose |
|---|---|
| `src/llm_agent.py` | Centralized controller and optional interactive LLM demo. |
| `src/decentralized_agent.py` | Per-UAV local controllers for the decentralized baseline. |
| `src/experiment_runner.py` | Headless runner for centralized/decentralized corridor trials. |
| `src/metrics.py` | Shared pass/fail and metric computation. |
| `src/scenario.py` | Shared corridor, spawn, frame, and threshold constants. |
| `src/formation_flight.py` | Formation helpers, waypoint progress, and formation-error math. |
| `docker/inject_obstacle_into_default_world.py` | Injects corridor buildings into PX4's default Gazebo world. |
| `scripts/start_px4_multi.sh` | Starts the three PX4 SITL UAVs. |
| `uav_eval_analysis.ipynb` | Executed notebook with result tables and plots. |

## Scenario

The experiment is a **known-map corridor** task:

- Three UAVs start in line formation.
- The leader flies through a corridor along positive `x`.
- Followers maintain fixed offsets relative to the leader.
- Buildings are injected into Gazebo as two rows of obstacles with a gap around `y=0`.
- Planning and evaluation use Gazebo ENU coordinates: `x` East, `y` North, `z` Up.
- Cruise altitude is `z=+6 m`.

This project does **not** add SLAM or real camera/LiDAR perception. It focuses on coordination/control architecture under a fixed simulated environment.

## Reproduce The Evaluation

Run commands from the repo root in WSL:

```bash
cd ~/uav-project-devops
```

### Terminal 1: Clean Start

```bash
rm -rf ./uav_eval ./uav_eval_centralized ./uav_eval_decentralized ./uav_eval_failed
docker compose down --remove-orphans
docker container prune -f
docker builder prune -f
```

### Terminal 1: Build And Start

Rebuild `ros2` whenever Python files change, then start the full stack:

```bash
docker compose build ros2
docker compose up -d --force-recreate px4 dds_0 dds_1 dds_2 ros2
```

Wait about 3 minutes for PX4/Gazebo/DDS startup.

### Terminal 2: Centralized Trial

```bash
docker exec -it ros2_agent bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
rm -rf /tmp/uav_eval
python3 /scripts/experiment_runner.py \
  --mode centralized \
  --scenario corridor \
  --log-dir /tmp/uav_eval/centralized \
  --no-llm
```

Expected:

```text
Wrote /tmp/uav_eval/centralized/pose.csv
Wrote /tmp/uav_eval/centralized/summary.json
mission_success=True
```

### Terminal 1: Copy Centralized Results

Copy before restarting, because `/tmp` is inside the container:

```bash
mkdir -p ./uav_eval
docker cp ros2_agent:/tmp/uav_eval/centralized ./uav_eval/centralized
python3 -m json.tool ./uav_eval/centralized/summary.json
```

### Terminal 1: Restart For Decentralized

```bash
docker compose down --remove-orphans
docker compose up -d --force-recreate px4 dds_0 dds_1 dds_2 ros2
```

Wait about 3 minutes again.

### Terminal 2: Decentralized Trial

```bash
docker exec -it ros2_agent bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
python3 /scripts/experiment_runner.py \
  --mode decentralized \
  --scenario corridor \
  --log-dir /tmp/uav_eval/decentralized \
  --no-llm
```

Expected:

```text
Wrote /tmp/uav_eval/decentralized/pose.csv
Wrote /tmp/uav_eval/decentralized/summary.json
mission_success=True
```

### Terminal 1: Copy Decentralized Results

```bash
docker cp ros2_agent:/tmp/uav_eval/decentralized ./uav_eval/decentralized
python3 -m json.tool ./uav_eval/decentralized/summary.json
```

### Terminal 1: Verify Files

```bash
ls -R ./uav_eval
```

Expected:

```text
./uav_eval/centralized/pose.csv
./uav_eval/centralized/summary.json
./uav_eval/decentralized/pose.csv
./uav_eval/decentralized/summary.json
```

## Results

Final run:

| Controller | Mission success | Mission time | Avg formation error | Max formation error | Failure reasons |
|---|---:|---:|---:|---:|---|
| Centralized | True | 8.02 s | 0.047 m | 0.123 m | none |
| Decentralized | True | 12.08 s | 0.126 m | 0.260 m | none |

Both modes completed the same corridor mission without stale odometry, oscillation, or corridor violations.

![Metric comparison](uav_eval/figures/metric_comparison.png)

Interpretation:

- Centralized control is faster because one controller computes the global mission and all formation setpoints.
- Decentralized control succeeds, but takes longer and has higher formation error because each UAV acts through local fixed-rule coordination.
- Both errors are well below the `2 m` success threshold.

## Trajectory And Safety Plots

The XY plot shows all UAVs passing through the corridor gap between the buildings.

![XY paths](uav_eval/figures/xy_paths.png)

The formation-error plot shows both modes staying below the `2 m` threshold in the successful final run.

![Formation error](uav_eval/figures/formation_error.png)

The altitude/lateral-deviation plot shows the UAVs staying near `z=6 m` and near the corridor centerline.

![Altitude and lateral deviation](uav_eval/figures/altitude_lateral.png)

## Notebook Analysis

Open the executed notebook for interactive plots and report-ready figures:

```bash
uav_eval_analysis.ipynb
```

It contains:

- Summary table
- Metric comparison charts
- XY trajectory plots
- Formation-error plots
- Altitude/lateral-deviation plots
- Final conclusions

## Optional Interactive LLM Demo

The original interactive centralized agent is still available:

```bash
docker exec -it ros2_agent bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
python3 /scripts/llm_agent.py
```

Useful menu options:

- `5`: fixed corridor mission without LLM
- `6`: triangle formation plus corridor mission
- `1` / `2` / `3`: LLM-assisted formation/mission generation if `GROQ_API_KEY` is configured

Create `.env` only if using the LLM path:

```bash
GROQ_API_KEY=your_key_here
```

## Cleanup

After saving results:

```bash
docker compose down --remove-orphans
docker container prune -f
docker builder prune -f
```

More aggressive cleanup if disk is full:

```bash
docker system prune -af --volumes
docker builder prune -af
```

On Windows, reclaim WSL/Docker disk space after cleanup:

```powershell
wsl --shutdown
```

Then compact Docker's VHDX from an Administrator `diskpart` session if needed.

## Project Conclusion

This project completes a reproducible comparison between centralized and decentralized multi-UAV coordination in the same simulated corridor. Both controllers successfully fly the three-UAV formation through the corridor, reach the final waypoint, and remain within the configured safety and formation-error thresholds.

The results show the expected trade-off. The centralized controller performs better in this controlled known-map scenario because one node has global responsibility for the mission and all formation setpoints, producing faster completion and smaller formation error. The decentralized baseline is still successful, but it is slower and less precise because each UAV computes its own movement from local fixed rules.

Overall, the project demonstrates a complete DevOps-style robotics workflow: containerized simulation, repeatable experiments, automated metric collection, pass/fail evaluation, and report-ready plots. The final outputs provide enough evidence to explain, reproduce, and compare both coordination approaches clearly.