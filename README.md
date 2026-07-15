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
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch greenhouse_nav greenhouse.launch.py
```

**Terminal 2 — RViz (optional, for visualisation):**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run rviz2 rviz2 -d greenhouse_nav/nav2_default_view_2.rviz
```

**Terminal 3 — Run the mission:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
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

## Tasks
 
### Task 1 — Simulation setup
 
The world has 5 parallel crop row corridors (~1 m wide, ~9 m long) built from simple box geometry. A red obstacle blocks one row to test the recovery behaviour in Task 4.
 
The launch file starts everything in order with timed delays:
 
- `t=0s` → Gazebo with the greenhouse world
- `t=3s` → TurtleBot3 Burger spawned at `(0, 0)`
- `t=5s` → Nav2 full stack
**Map:** Generated programmatically from the known world dimensions using Python/PIL. SLAM (Cartographer) was attempted but failed to converge in symmetric parallel corridors — a known limitation when there are no distinctive landmarks. In production, SLAM Toolbox would be used on first run and the map saved for reuse.
 
---
 
### Task 2 — Row-following mission
 
The mission node sends the robot through all 5 rows in a boustrophedon (snake) pattern using Nav2's `NavigateToPose` action:
 
```
Row 1 →   (0.5, 0.0) → (9.5, 0.0)
Row 2 ←   (9.5, 2.0) → (0.5, 2.0)
Row 3 →   (0.5, 4.0) → (9.5, 4.0)  ← obstacle here
Row 4 ←   (9.5, 6.0) → (0.5, 6.0)
Row 5 →   (0.5, 8.0) → (9.5, 8.0)
```
 
Waypoints are loaded from `config/waypoints.yaml` and can be changed without recompiling.
 
The node starts automatically: it publishes the robot's initial pose to AMCL (retrying every 2s until confirmed), then waits for `bt_navigator` to be active before sending the first goal. No manual RViz interaction needed.
 
Progress is reported via:
- Logs — current waypoint, row completions, and any skips
- `/mission/current_waypoint` (PoseStamped) — current target
- `/mission/planned_path` (Path) — full route, visible in RViz
---
 
### Task 3 — Nav2 tuning for narrow rows
 
The default Nav2 parameters are designed for open spaces and needed several adjustments for 1 m corridors.
 
#### The robot was too wide for the planner
**`robot_radius`: 0.22 m → 0.11 m**
The default is larger than the real TurtleBot3 Burger (0.105 m). With 0.22 m, the planner thought the robot couldn't fit in the corridor and either failed to find a path or hugged the walls. Setting the real size fixes this.
 
#### The walls were inflated into the middle of the corridor
**`inflation_radius`: 0.55 m → 0.15 m**
With 1 m corridors and 0.55 m inflation on each side, the entire corridor was marked as occupied — no path could be found. Reducing to 0.15 m leaves a clear navigable centre.
 
**`cost_scaling_factor`: 3.0 → 5.0**
A steeper cost gradient pushes the robot toward the centre of the corridor. With a low gradient, the robot drifts toward the walls because the cost difference is too small to matter.
 
#### The local costmap was too large
**Local costmap: 5 × 5 m → 2 × 2 m**
At 5 × 5 m, the local costmap included the walls of adjacent corridors, causing the controller to react to obstacles outside the current row. Reducing to 2 × 2 m limits its view to the immediate surroundings.
 
#### The controller oscillated in tight spaces
**Controller: DWB → RegulatedPurePursuitController**
DWB samples many possible velocities and scores them, which causes oscillation in narrow corridors. RegulatedPurePursuitController follows the path directly using a lookahead point, producing smooth straight-line motion.
 
**`desired_linear_vel`: 0.22 → 0.2 m/s**
Slightly slower speed gives the controller more time to react and reduces overshoot at row ends.
 
**`lookahead_dist`: 0.6 → 0.4 m**
A shorter lookahead makes the robot react earlier to upcoming turns. At 0.6 m it would start turning too late and clip the corridor walls.
 
---
 
### Task 4 — Recovery behaviour
 
A red obstacle blocks row 3. The robot detects the blockage, skips that row, and continues with the remaining ones. Recovery is handled entirely in the mission node using three mechanisms:
 
**1. Stuck detection**
The node measures the straight-line distance from the robot's position (from AMCL) to the current goal. If that distance doesn't decrease by more than 5 cm in 15 seconds, the robot is considered blocked and skips to the next row. Nav2's own `distance_remaining` feedback was not used because it resets on every replan and doesn't reliably reflect actual progress.
 
**2. Plan exit check**
The node monitors Nav2's planned path. If the plan routes through an already-completed row (trying to go around the obstacle), the goal is cancelled immediately.
 
**3. Nav2 abort fallback**
If Nav2 can't find any path at all, the `STATUS_ABORTED` result triggers the same skip.
 
**Row selection after skip**
When skipping, the node picks the nearest unvisited row by checking both endpoints of every remaining row. The closer endpoint becomes the entry point — so if the robot is on the west side, it enters from the west, minimising unnecessary travel.

Each row is tracked as a simple dictionary:

```python
{'start': int, 'end': int, 'entry': int, 'completed': bool}
```

- `start` / `end` — waypoint indices from the YAML
- `entry` — which endpoint the robot actually entered from (`-1` = not visited yet)
- `completed` — set to `True` once both endpoints are reached, or when the row is skipped
 
---


## Limitations
 
- **Localisation:** AMCL occasionally struggles in symmetric corridors with no distinctive landmarks. Odometry fusion or visual markers at row ends would improve reliability.
- **Stuck threshold:** 15s is tuned for this robot speed (0.2 m/s) and row length. For other setups it should scale with `row_length / speed`.
- **Row direction:** After a skip, the robot may traverse a row in the opposite direction from the YAML definition. The node handles both directions correctly.
- **Docker:** Requires Nvidia GPU passthrough. Gazebo Ignition would remove this requirement.
 