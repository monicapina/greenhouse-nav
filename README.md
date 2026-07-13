# greenhouse-nav
# Greenhouse Navigation — Antoniq Robotics Technical Assessment

ROS2 Humble | TurtleBot3 Burger | Nav2 | Gazebo Classic

---

## Quick start

```bash
# 1. Clone
git clone https://github.com/YOUR_USERNAME/greenhouse-nav.git
cd greenhouse-nav/greenhouse_nav

# 2. Install dependencies
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3* \
  python3-colcon-common-extensions

# 3. Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 4. Launch everything (Gazebo + Nav2)
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch greenhouse_nav greenhouse.launch.py

# 5. Open RViz and set initial pose (2D Pose Estimate at x=0, y=0)
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz

# 6. Run the mission
ros2 run greenhouse_nav mission_node
```

---

## Package structure

```
greenhouse_nav/
├── launch/
│   └── greenhouse.launch.py      # Single launch file: Gazebo + robot + Nav2
├── worlds/
│   └── greenhouse.world          # 3 crop row corridors + red obstacle
├── config/
│   ├── nav2_params.yaml          # Tuned costmap and controller parameters
│   └── waypoints.yaml            # Boustrophedon waypoints (editable)
├── maps/
│   ├── greenhouse_map.pgm        # Occupancy grid map
│   └── greenhouse_map.yaml       # Map metadata
└── greenhouse_nav/
    └── mission_node.py           # Mission node: boustrophedon + recovery
```

---

## Task 1 — Simulation setup

The world contains 3 parallel crop row corridors (~1m wide each) built from simple box geometry. A red obstacle blocks row 2 for Task 4.

A single launch file starts everything:

```bash
ros2 launch greenhouse_nav greenhouse.launch.py
```

This launches Gazebo (t=0s), spawns the TurtleBot3 Burger (t=3s), and brings up Nav2 (t=5s). The delays ensure each component is ready before the next one starts.

**Map note:** The map was generated programmatically from the known world geometry rather than with SLAM. In a real greenhouse, SLAM Toolbox would be used on the first run and the map saved for reuse. Cartographer was attempted but struggled with the symmetric parallel corridors — a known limitation of feature-based SLAM in repetitive environments.

---

## Task 2 — Row-following mission

The mission node (`mission_node.py`) sends the robot through the field in a boustrophedon pattern using Nav2's `NavigateToPose` action:

```
Row 1 →    (x=-2.0, y=0.0)  →  (x=2.0, y=0.0)
Row 2 ←    (x=2.0,  y=2.1)  →  (x=-2.0, y=2.1)
Row 3 →    (x=-2.0, y=4.2)  →  (x=2.0, y=4.2)
```

Waypoints are loaded from `config/waypoints.yaml` — they can be changed without recompiling:

```yaml
waypoints:
  - x: -2.0
    y: 0.0
    yaw: 0.0
    label: "Row 1 - Start"
  ...
```

Progress is reported on every waypoint via logs and published on `/mission/current_waypoint` (PoseStamped).

---

## Task 3 — Tuning for narrow rows

Key changes from Nav2 defaults in `config/nav2_params.yaml`:

| Parameter | Default | Ours | Reason |
|-----------|---------|------|--------|
| Controller | DWB | RegulatedPurePursuitController | DWB oscillates in narrow corridors. Pure Pursuit follows the path smoothly. |
| `desired_linear_vel` | 0.22 m/s | 0.2 m/s | Slower speed gives more time to correct heading in tight spaces. |
| `inflation_radius` | 0.55 m | 0.3 m | With 1m corridors, 0.55m would inflate both walls into the center leaving no free space for the planner. |
| `cost_scaling_factor` | 3.0 | 5.0 | Steeper cost gradient keeps the robot centered in the corridor. |
| `robot_radius` | 0.22 m | 0.11 m | Matches the real TurtleBot3 Burger radius (0.105m). Default is too conservative and blocks narrow paths. |
| Local costmap size | 5×5 m | 2×2 m | Smaller window prevents walls from adjacent rows influencing the local planner. |
| `lookahead_dist` | 0.6 m | 0.4 m | Shorter lookahead makes the controller react faster to corridor geometry. |

---

## Task 4 — Recovery behaviour

A red obstacle blocks row 2. The mission node handles this at the **mission-node level** rather than via a custom Behavior Tree.

**How it works:**
1. Robot attempts row 2
2. Nav2 tries its built-in recoveries (spin, backup, wait)
3. If Nav2 returns FAILED, or if 90 seconds pass without reaching the waypoint (timeout), the mission node calls `_skip_to_next_row()`
4. The node jumps 2 waypoints forward (skipping the entire blocked row) and continues with row 3

**Trade-off — mission node vs Behavior Tree:**

| Approach | Pros | Cons |
|----------|------|------|
| Mission node (ours) | Simple, easy to modify, no XML | Robot waits for Nav2's full recovery cycle before timeout triggers |
| Custom BT | Faster reaction, more control | Requires writing and debugging BT XML |

For this use case the mission node approach is sufficient and easier to reason about. If faster recovery were needed (e.g. rows are 0.5m and the robot gets stuck immediately), a custom BT that detects blockage via costmap would be the better choice.

---

## Limitations and honest trade-offs

- **Map:** Generated programmatically from known world geometry. In production, SLAM Toolbox on first run would be more robust.
- **Initial pose:** Requires manual 2D Pose Estimate in RViz after launch. This could be automated by publishing to `/initialpose` in the launch file since the robot always spawns at x=0, y=0.
- **Recovery timeout:** Set to 90s to accommodate slow Nav2 recovery cycles. In a real greenhouse this should be tuned based on row length and robot speed.
- **Localization:** AMCL struggles in symmetric corridors. In production, wheel odometry fusion or visual markers at row ends would improve localisation reliability.

---

## Adapting to live changes

**"Rows are now 0.8m"**
- Reduce `inflation_radius` from 0.3 to 0.2 in `nav2_params.yaml`
- Reduce `desired_linear_vel` to 0.15 for more careful navigation
- No code changes needed

**"Skip recovery, retry twice instead"**
- In `_result_callback`, instead of calling `_skip_to_next_row()` immediately, add a retry counter per waypoint
- If retries < 2, call `_send_next_goal()` again; else call `_skip_to_next_row()`
- Change is isolated to one method, ~10 lines

**"Add a 4th row"**
- Add 2 entries to `config/waypoints.yaml`
- Add wall geometry to `worlds/greenhouse.world`
- No code changes needed