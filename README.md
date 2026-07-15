# Greenhouse Navigation — Antoniq Robotics Technical Assessment

ROS2 Humble · TurtleBot3 Burger · Nav2 · Gazebo Classic

> 📹 [Screen recording of full mission run](greenhouse_nav/media/result.mp4)

---

## Quick start

```bash
git clone https://github.com/monicapina/greenhouse-nav.git
cd greenhouse-nav/greenhouse_nav

sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  python3-colcon-common-extensions

source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

**Terminal 1 — Launch everything (Gazebo + robot + Nav2):**
```bash
conda deactivate
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch greenhouse_nav greenhouse.launch.py
```

**Terminal 2 — RViz (optional, for visualisation):**
```bash
conda deactivate && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run rviz2 rviz2 -d greenhouse_nav/nav2_default_view_2.rviz
```

**Terminal 3 — Run the mission:**
```bash
conda deactivate && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run greenhouse_nav mission_node
```

> The mission node publishes the initial pose automatically — no manual RViz interaction needed.

## Docker (bonus)

The Docker image installs all dependencies and builds the package automatically. Running it is equivalent to following the manual setup — no ROS2 installation needed on the host.

**Requirements:** Linux host, Nvidia GPU, display (X11).

**Step 1 — Build the image:**
```bash
cd greenhouse-nav
docker build -t greenhouse-nav .
```

**Step 2 — Allow Docker to access your display and run:**
```bash
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --env DISPLAY=$DISPLAY \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  greenhouse-nav
```

This starts Gazebo + the robot + Nav2 inside the container, with the GUI forwarded to your screen via X11.

**Step 3 — In a second terminal, run the mission:**
```bash
docker exec -it <container_id> /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 run greenhouse_nav mission_node"
```

> **Limitation:** Gazebo Classic requires an Nvidia GPU inside the container (`--gpus all`). Without it, Gazebo will not render correctly. Migrating to Gazebo Ignition would remove this requirement and allow fully headless execution.


---

## Repository structure

```
greenhouse_nav/
├── launch/greenhouse.launch.py      # Single launch: Gazebo + robot + Nav2
├── worlds/greenhouse.world          # 3 corridors + red obstacle in row 3
├── config/nav2_params.yaml          # Tuned Nav2 parameters
├── config/waypoints.yaml            # Boustrophedon waypoints
├── maps/greenhouse_map.*            # Occupancy grid map
├── greenhouse_nav/media/result.mp4                 # Screen recording
└── greenhouse_nav/mission_node.py   # Mission node
```

---

## Task 1 — Simulation setup

The world contains 3 parallel crop row corridors (~1 m wide, ~9 m long) built from box geometry. A red obstacle (0.4 × 1.2 m) blocks one row to trigger the recovery behaviour in Task 4.

The launch file starts everything with timed delays to avoid Gazebo race conditions:

- `t=0s` → Gazebo with greenhouse world
- `t=3s` → TurtleBot3 Burger spawned at `(0, 0)`
- `t=5s` → Nav2 full stack

**Map generation:** Generated programmatically from known world geometry using Python/PIL rather than SLAM. Cartographer was attempted but struggled with the symmetric parallel corridors — a known limitation of feature-based SLAM in repetitive environments with no distinctive landmarks. In production, SLAM Toolbox would be used on first run and the map saved for reuse.

---

## Task 2 — Row-following mission

The mission node drives the robot in a boustrophedon pattern across 5 rows using Nav2's `NavigateToPose` action. Waypoints are loaded from `config/waypoints.yaml` and can be edited without recompiling.

**Startup sequence:** The node publishes the initial pose to `/initialpose` automatically (retrying every 2s until AMCL confirms via `/amcl_pose`), then polls the `bt_navigator` lifecycle state until active before sending the first goal. This removes the need for manual RViz interaction and avoids race conditions with Nav2 startup.

**Progress reporting:**
- Logs report current waypoint, row completion, and any skips
- `/mission/current_waypoint` (PoseStamped) — current navigation target
- `/mission/planned_path` (Path) — full boustrophedon path, visible in RViz

---

## Task 3 — Nav2 tuning for narrow rows

The default Nav2 parameters are designed for open spaces. The 1 m corridors required several changes to avoid oscillation, wall collisions, and planning failures.

### Problem: robot too wide for the planner

**Default `robot_radius`: 0.22 m → changed to 0.11 m**

The default value is larger than the actual TurtleBot3 Burger footprint (0.105 m). With 0.22 m, the inflation layer would treat the robot as too wide to fit in the corridor, causing the planner to fail or hug the walls. Setting it to the real size gives the planner accurate information.

### Problem: walls inflated into the corridor centre

**Default `inflation_radius`: 0.55 m → changed to 0.15 m**

With a 1 m corridor and walls on both sides, an inflation radius of 0.55 m means the lethal zone extends 0.55 m from each wall — covering the entire corridor. No path can be found. Reducing it to 0.15 m keeps a safe buffer around the walls while leaving a navigable centre.

**`cost_scaling_factor`: 3.0 → 5.0**

A steeper cost gradient around the walls pushes the robot toward the corridor centre. With a low gradient, the robot drifts toward walls because the cost difference is small. With 5.0, the centre is clearly the lowest-cost path.

### Problem: local costmap sees adjacent corridors

**Local costmap size: 5 × 5 m → 2 × 2 m**

The default 5 × 5 m local costmap is wide enough to include the walls of adjacent corridors. This caused the controller to react to obstacles that were not in the current row, creating erratic steering. Reducing it to 2 × 2 m limits the controller's view to the immediate surroundings.

### Problem: controller oscillates in tight spaces

**Controller: DWB → RegulatedPurePursuitController**

DWB (Dynamic Window Approach) samples many velocity candidates and scores them, which causes oscillation in narrow corridors because small heading errors lead to large velocity corrections. RegulatedPurePursuitController follows the planned path directly using a lookahead point, producing smooth straight-line motion in corridors.

**`desired_linear_vel`: 0.22 m/s → 0.2 m/s**

Slightly lower speed gives the controller more time to react to corridor geometry and reduces overshoot at row ends.

**`lookahead_dist`: 0.6 m → 0.4 m**

A shorter lookahead distance makes the controller react faster to upcoming curves (e.g. row entries). At 0.6 m the robot would start turning too late and clip the corridor walls.

---

## Task 4 — Recovery behaviour

A red obstacle blocks one row. Recovery is implemented entirely at **mission-node level** using three complementary mechanisms:

**1. Stuck detection (primary)**
The node tracks the Euclidean distance from the robot's AMCL pose to the current goal waypoint. If this distance does not decrease by more than 5 cm within 15 seconds, the robot is considered blocked. Nav2's built-in `distance_remaining` feedback was not used because it resets on every replan and does not reliably reflect actual forward progress.

**2. Plan corridor-exit check (secondary)**
The node subscribes to Nav2's `/plan` topic. If the planned path exits the current row's Y corridor (±0.6 m) into an already-completed row, the goal is cancelled immediately. This catches cases where Nav2 tries to route around an obstacle through a previously traversed row.

**3. Nav2 STATUS_ABORTED (fallback)**
If Nav2 finds no path at all, the aborted status triggers the same skip logic.

**On skip — nearest-entry selection:**
When a row is skipped, the node finds the nearest unvisited row by evaluating **both endpoints** of each candidate row. The closer endpoint becomes the entry point and the other becomes the exit. This minimises travel distance after a detour — for example, if the robot is on the west side of the field after a blocked row, it enters the next row from the west even if the YAML defines it starting from the east.

**Trade-off — mission node vs custom Behavior Tree:**

| | Mission node (ours) | Custom BT |
|--|--|--|
| Complexity | Single Python class, easy to modify | Requires BT XML authoring |
| Reaction time | Detects blockage via distance tracking and plan monitoring | Can react at the BT tick rate |
| Maintainability | All logic in one file | Logic split across Python and XML |
| Live adaptation | Change threshold constants | Modify BT XML and relaunch |

Mission-node level is the right choice here: it keeps all recovery logic in one readable place and is easy to adapt live (e.g. "retry twice instead of skipping" is a small change to `_skip_to_nearest_unvisited`).

---

## Limitations and production notes

- **Localisation reliability:** AMCL occasionally fails to converge in highly symmetric corridors. Wheel odometry fusion (`robot_localization` EKF) or distinctive row markers would improve reliability. In production, SLAM Toolbox with a saved map would replace the programmatic map.
- **Stuck threshold:** `STUCK_TIME_THRESHOLD = 15s` is tuned for the current robot speed (0.2 m/s) and corridor length. It should scale with `row_length / desired_linear_vel` for other deployments.
- **Row entry direction:** The nearest-endpoint selection means the robot may traverse a row in the reverse direction from the YAML definition. This is intentional — the node handles both directions correctly regardless of which end is entered first.
- **Docker:** Requires Nvidia GPU passthrough. Gazebo Ignition would enable fully headless containers without GPU requirements.