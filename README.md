## Agentic Multi-UAV Coordination (PX4 SITL + ROS2 + LLM)

This repo runs a 3-UAV PX4 SITL + Gazebo simulation in Docker, and provides a ROS2 (Jazzy) Python agent that:

- asks an LLM (Groq) for a **leader route** (waypoints)
- asks an LLM for **formation offsets** (followers relative to leader)
- publishes offboard setpoints so the **whole formation moves** while maintaining offsets

### Requirements

- Docker / Docker Compose
- WSLg working (for Gazebo UI on Windows)
- A Groq API key

### Setup

Create a `.env` file in the repo root:

```bash
GROQ_API_KEY=your_key_here
```

`.env` is ignored by git (see `.gitignore`).

### Run

Start the stack:

```bash
docker compose up --build
```

In another terminal, open the ROS2 container:

```bash
docker exec -it ros2_agent bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
python3 /scripts/llm_agent.py
```

### Tonight: one-shot repro (buildings + 3 drones + corridor)

Do this from the **repo root** in **WSL** (e.g. `cd ~/uav-project-devops`). Images bake in **world injection**, **DDS ports 8888/8889/8890** per drone (`scripts/start_px4_multi.sh`), and the **ROS2 agent** (`src/` → `/scripts/`). Old containers keep old behavior until you **down + build + recreate**.

**1 — Stack (wait ~3 minutes after `up` for `start_px4_multi.sh` sleeps)**

```bash
docker compose down
docker compose build px4 ros2
docker compose up -d --force-recreate px4 dds_0 dds_1 dds_2 ros2
```

**2 — Agent**

```bash
docker exec -it ros2_agent bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
python3 /scripts/llm_agent.py
```

**3 — Demo**

- Menu **5** → **s** — line formation, **(4,0,+6) → (40,0,+6)** through the gap (no LLM).  
- Or **6** → **s** — triangle + same path.

**4 — Logs sanity check**

- `[UAV_POSE] … odom_alliance_xyz`: **L / F1 / F2** should all move in **x** together (F1 ≈ L+2, F2 ≈ L+4 in **alliance** frame).  
- If **F2** stays near **(0,0,…)** forever, the **px4** image is still wrong (rebuild **px4**) or DDS agents are not up.

### Centralized vs decentralized corridor evaluation

This project treats the corridor as a **known-map simulation**: the obstacle geometry is fixed in Gazebo, and controllers use PX4 odometry plus fixed coordination rules. It does not add SLAM or camera/LiDAR perception.

The comparison keeps the same Gazebo/PX4/ROS2 stack and changes only the controller mode:

- `centralized`: the existing `FormationNode` acts as one global planner / central formation controller and computes all UAV setpoints.
- `decentralized`: three local controllers run independently; UAV0 follows the corridor route, UAV1 follows UAV0 with a fixed local rule, and UAV2 follows UAV1 with the same fixed local rule.

Rebuild and start the stack:

```bash
docker compose down
docker compose build px4 ros2
docker compose up -d --force-recreate px4 dds_0 dds_1 dds_2 ros2
```

After PX4 and DDS are ready, run the centralized trial:

```bash
docker exec -it ros2_agent bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
python3 /scripts/experiment_runner.py \
  --mode centralized \
  --scenario corridor \
  --log-dir /tmp/uav_eval/centralized \
  --no-llm
```

Reset the simulation or restart the stack so the next trial starts from the same initial conditions, then run:

```bash
python3 /scripts/experiment_runner.py \
  --mode decentralized \
  --scenario corridor \
  --log-dir /tmp/uav_eval/decentralized \
  --no-llm
```

Copy results back to the host:

```bash
docker cp ros2_agent:/tmp/uav_eval ./uav_eval
```

Each trial writes:

```text
uav_eval/<mode>/pose.csv
uav_eval/<mode>/summary.json
```

The `summary.json` file reports `mission_success`, `mission_time_sec`, `formation_error_avg_m`, `formation_error_max_m`, threshold values, and failure reasons. A run passes only when the leader reaches the final corridor waypoint, all UAVs stay inside the corridor-safe region, odometry remains valid, and max formation error stays under the configured threshold (`2.0 m` by default).

### Free SSD: Docker + WSL (quick)

**Docker (run in WSL or PowerShell, repo optional):**

```bash
docker compose down
docker system prune -af
docker builder prune -af
```

`prune -af` removes **all** unused images/containers/networks (not just this project). Drop the `f` if you want to keep unused images.

**WSL virtual disk (Windows — frees space Windows reports for WSL):**

1. Exit shells, then: `wsl --shutdown` (PowerShell or CMD as you).
2. Optional: **Disk Management** or **Optimize-VHD** on `ext4.vhdx` / `docker-desktop-data` (path varies; common under `%LOCALAPPDATA%\Docker\wsl\` and `\Packages\Canonical...\LocalState\`).
3. Or: **Docker Desktop → Settings → Resources → Disk image location** / **Clean / Purge data** if you use Desktop’s cleaner.

After heavy pruning, **next run** will **re-download/rebuild** images — that is normal.

### Usage

The agent prompts for:

- `formación>>>` (example: `triangulo`)
- `misión>>>` (example: `fly a square at 5m altitude starting at x=0 y=0`)

It will print the offsets and waypoint count, ask for confirmation, then start moving the formation.

### Buildings in Gazebo (default world)

PX4 loads the stock `**default`** world. At **Docker build**, `[docker/inject_obstacle_into_default_world.py](docker/inject_obstacle_into_default_world.py)` injects **six** static boxes (`building_obstacle_0` … `_5`) in two rows with a **corridor along +X** at **y≈0** (see script header for dimensions). Rebuild `**px4`** after editing that script, then **restart** the stack (see **Quick reproduce** above).

**Obstacle avoidance in the LLM agent is opt-in (default off).** Add to `.env` to enable:

```bash
UAV_OBSTACLE_AVOIDANCE=1
```

Optional tuning: `UAV_BUILDING_HALF_XY` (default `3`), `UAV_BUILDING_HALF_Z` (default `6`) — must match the injected box size.

When enabled, use a mission through **y≈0**, **x** from ~0 to ~45 at **z=+6** (default cruise, ENU-Up) to exercise the virtual rangefinders against those boxes.

### Multi-UAV: third drone “stuck”, odom near (0,0,0)

Compose runs **three** Micro XRCE-DDS agents on UDP **8888**, **8889**, **8890**. Each PX4 SITL instance must use the matching port (`PX4_MICRODDS_UDP_PORT`). If two drones share the same port, ROS2 only talks reliably to one of them; the other keeps bogus odometry and ignores offboard. See `[scripts/start_px4_multi.sh](scripts/start_px4_multi.sh)`. After changing it: `**docker compose build px4`** and restart the stack.

### Debug: log commanded vs estimated x,y

In `.env`:

```bash
UAV_LOG_POSE=1
```

Optional: only while a mission is running (after option 2/3):

```bash
UAV_LOG_POSE_MISSION_ONLY=1
```

Prefer **rebuild `ros2` + recreate container** over ad-hoc `docker cp`. Run the agent and watch `[UAV_POSE] cmd_alliance_xyz` vs `odom_alliance_xyz` — both are in **Gazebo ENU** (leader spawn = origin). Raw PX4 setpoints per drone differ because each SITL instance has its **own local NED origin** at its Gazebo pose (`initial_positions` / `start_px4_multi.sh`).