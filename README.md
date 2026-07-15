# Greenhouse Navigation — Antoniq Robotics Technical Assessment

ROS2 Humble · TurtleBot3 Burger · Nav2 · Gazebo Classic

> 📹 [Screen recording of full mission run](media/final_result.mp4)

---

## Installation

```bash
# Clone the repository
git clone https://github.com/monicapina/greenhouse-nav.git
cd greenhouse-nav/greenhouse_nav

# Install ROS2 dependencies
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  python3-colcon-common-extensions

# Build the package
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> **Note:** If using Miniconda, run `conda deactivate` before sourcing ROS2 to avoid Python version conflicts.

---

## Running

Every new terminal requires:
```bash
conda deactivate                                          # if using conda
source /opt/ros/humble/setup.bash                        # source ROS2
cd greenhouse-nav/greenhouse_nav && source install/setup.bash  # source package
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
```

**Terminal 1 — Launch everything (Gazebo + robot + Nav2):**
```bash
ros2 launch greenhouse_nav greenhouse.launch.py
```

**Terminal 2 — RViz (set initial pose with 2D Pose Estimate at x=0, y=0):**
```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**Terminal 3 — Run the mission:**
```bash
ros2 run greenhouse_nav mission_node
```

---

## Running with Docker (bonus)

Requires Nvidia GPU and Linux with display.

```bash
# Build
cd greenhouse-nav
docker build -t greenhouse-nav .

# Run
xhost +local:docker
docker run -it --rm \
  --gpus all \
  --env DISPLAY=$DISPLAY \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  greenhouse-nav
```

Then in a second terminal run the mission:
```bash
docker exec -it <container_id> /bin/bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /ros2_ws/install/setup.bash && \
   ros2 run greenhouse_nav mission_node"
```

> Gazebo Classic requires GPU access inside Docker. Migrating to Gazebo Ignition would provide better container support.

---

## Structure

```
greenhouse_nav/
├── launch/greenhouse.launch.py      # Gazebo + robot + Nav2
├── worlds/greenhouse.world          # 3 corridors + red obstacle
├── config/nav2_params.yaml          # Tuned Nav2 parameters
├── config/waypoints.yaml            # Boustrophedon waypoints
├── maps/greenhouse_map.*            # Occupancy grid map
└── greenhouse_nav/mission_node.py   # Mission node
```

---

## Task 1 — Simulation setup

The world contains 3 parallel crop row corridors (~1m wide) built from simple box geometry. A red obstacle blocks row 2 for Task 4.

The single launch file starts everything in sequence:
- t=0s → Gazebo with greenhouse world
- t=15s → TurtleBot3 Burger spawned at (0, 0)
- t=20s → Nav2 full stack

**Map note:** Generated programmatically from known world dimensions using Python/PIL. Cartographer SLAM was attempted but struggled with the symmetric parallel corridors — a known limitation of feature-based SLAM in repetitive environments. In production, SLAM Toolbox would be used on first run and the map saved for reuse.

---

## Task 2 — Row-following mission

The mission node sends the robot through the field in a boustrophedon (snake) pattern using Nav2's `NavigateToPose` action:

```
Row 1 →   (-2.0, 0.0) → (2.0, 0.0)
Row 2 ←   ( 2.0, 2.1) → (-2.0, 2.1)
Row 3 →   (-2.0, 4.2) → (2.0, 4.2)
```

Waypoints are loaded from `config/waypoints.yaml` — they can be edited without recompiling. To change the pattern, just update the YAML file.

Progress is reported via logs and published on `/mission/current_waypoint` (PoseStamped). The planned path is also published on `/mission/planned_path` and visible in RViz.

---

## Task 3 — Tuning for narrow rows

| Parameter | Default | Ours | Why |
|-----------|---------|------|-----|
| Controller | DWB | RegulatedPurePursuitController | DWB oscillates in tight spaces. Pure Pursuit follows the path smoothly. |
| `desired_linear_vel` | 0.22 m/s | 0.2 m/s | More time to correct heading in narrow corridors |
| `inflation_radius` | 0.55 m | 0.3 m | 0.55m inflates both walls into the center — no path found |
| `cost_scaling_factor` | 3.0 | 5.0 | Steeper gradient keeps the robot centered |
| `robot_radius` | 0.22 m | 0.11 m | Matches real TB3 Burger size (0.105m) |
| Local costmap | 5×5 m | 2×2 m | Prevents adjacent corridor walls from interfering |
| `lookahead_dist` | 0.6 m | 0.4 m | Faster reaction to corridor geometry |

---

## Task 4 — Recovery behaviour

A red obstacle blocks row 2. Recovery is implemented at **mission-node level** using a 90s timeout:

1. Robot attempts row 2
2. Nav2 runs its built-in recoveries (spin, backup, wait)
3. If Nav2 returns FAILED or 90s pass, the mission node cancels the goal and calls `_skip_to_next_row()`
4. The node jumps 2 waypoints forward and continues with row 3

**Trade-off — mission node vs custom Behavior Tree:**

| | Mission node (ours) | Custom BT |
|--|--|--|
| Complexity | Simple, easy to modify | Requires BT XML |
| Reaction time | Waits for Nav2 recovery cycle | Can react immediately |
| Maintainability | One Python method | Separate XML file |

Mission node level is sufficient here. A custom BT would be better if faster obstacle detection were needed (e.g. corridors under 0.5m).

---

## Limitations

- **Initial pose:** Must be set manually in RViz. Could be automated since the robot always spawns at (0, 0) — would be the first change for production.
- **Localisation:** AMCL struggles in symmetric corridors. Visual markers at row ends or wheel odometry fusion would improve reliability in production.
- **Recovery timeout:** 90s is conservative. Should be tuned to row length and robot speed in real deployments.
- **Docker:** Requires Nvidia GPU passthrough. Migrating to Gazebo Ignition would provide full headless container support.