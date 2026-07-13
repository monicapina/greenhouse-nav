# Greenhouse Navigation — Antoniq Robotics Technical Assessment

ROS2 Humble · TurtleBot3 Burger · Nav2 · Gazebo Classic

---

## Quick start

```bash
git clone https://github.com/monicapina/greenhouse-nav.git
cd greenhouse-nav/greenhouse_nav

sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* python3-colcon-common-extensions

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch greenhouse_nav greenhouse.launch.py
```

Then in a second terminal, set the initial pose in RViz (2D Pose Estimate at x=0, y=0):
```bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

Then run the mission:
```bash
ros2 run greenhouse_nav mission_node
```

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

3 parallel corridors (~1m wide), single launch file starts Gazebo, robot and Nav2 with timed delays (0s / 3s / 5s).

**Map:** Generated programmatically from known world geometry. Cartographer was attempted but struggled with symmetric corridors — a known SLAM limitation in repetitive environments.

---

## Task 2 — Row-following mission

Boustrophedon pattern via `NavigateToPose` action:

```
Row 1 →   (-2.0, 0.0) → (2.0, 0.0)
Row 2 ←   ( 2.0, 2.1) → (-2.0, 2.1)
Row 3 →   (-2.0, 4.2) → (2.0, 4.2)
```

Waypoints loaded from `config/waypoints.yaml` — editable without recompiling. Progress published on `/mission/current_waypoint`.

---

## Task 3 — Tuning for narrow rows

| Parameter | Default | Ours | Why |
|-----------|---------|------|-----|
| Controller | DWB | RegulatedPurePursuitController | DWB oscillates in tight spaces |
| `desired_linear_vel` | 0.22 m/s | 0.2 m/s | More control in narrow corridors |
| `inflation_radius` | 0.55 m | 0.3 m | 0.55m blocks the entire 1m corridor |
| `cost_scaling_factor` | 3.0 | 5.0 | Steeper gradient keeps robot centered |
| `robot_radius` | 0.22 m | 0.11 m | Matches real TB3 Burger size |
| Local costmap | 5×5 m | 2×2 m | Prevents adjacent walls from interfering |
| `lookahead_dist` | 0.6 m | 0.4 m | Faster reaction to corridor geometry |

---

## Task 4 — Recovery behaviour

Red obstacle blocks row 2. Recovery implemented at **mission-node level**: if Nav2 returns FAILED or 90s timeout is reached, the node skips 2 waypoints forward and continues with row 3.

**Trade-off vs custom Behavior Tree:**
- Mission node: simpler, easier to modify, but waits for Nav2's full recovery cycle
- Custom BT: faster reaction, but requires writing and debugging BT XML

---

## Limitations

- Initial pose must be set manually in RViz. Could be automated since robot always spawns at (0, 0).
- AMCL localisation is unreliable in symmetric corridors. Visual markers or odometry fusion would help in production.
- 90s timeout is conservative — should be tuned to row length and speed in real deployments.