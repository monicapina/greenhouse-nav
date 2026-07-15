#!/usr/bin/env python3
"""
Mission node for greenhouse boustrophedon navigation.
Sends the robot through crop rows in a snake pattern using Nav2.

Row tracking:
- Each row has a start and end waypoint index, plus an 'entry' field that
  records which end the robot actually entered from (-1 = not yet visited).
  This allows entering a row from either end (nearest to the robot) and
  always exiting from the other end.
- A row is completed once both waypoints have been reached.

Recovery strategy:
- Stuck detection: if distance_remaining does not decrease by more than
  STUCK_DISTANCE_THRESHOLD for STUCK_TIME_THRESHOLD seconds, the robot is
  considered blocked and skips to the nearest unvisited row.
  The clock only starts after the first meaningful progress, so initial
  turns / spin-up do not trigger false positives.
- Fallback: Nav2 STATUS_ABORTED (no path found) also triggers a skip.
- Plan exit detection: if Nav2's planned path (/plan) exits the current
  row corridor into an already-completed row, cancel and skip.

Nearest-unvisited selection:
- Both endpoints of every unvisited row are considered. The closest one
  to the robot becomes the entry point; the other becomes the exit.

Localisation:
- Publish initial pose matching spawn_entity coordinates; retry every 2s.
- Poll bt_navigator lifecycle state until active before sending first goal.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from lifecycle_msgs.srv import GetState

import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory

# Corridor half-width (metres).
ROW_HALF_WIDTH = 0.6

# Robot spawn position — must match spawn_entity args in launch file.
SPAWN_X = 0.0
SPAWN_Y = 0.0

# Stuck detection.
STUCK_DISTANCE_THRESHOLD = 0.05   # metres of progress needed to reset the clock
STUCK_TIME_THRESHOLD     = 15.0    # seconds without progress → considered stuck


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._progress_pub = self.create_publisher(PoseStamped, '/mission/current_waypoint', 10)
        self._path_pub = self.create_publisher(Path, '/mission/planned_path', 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self._bt_check_client = self.create_client(GetState, '/bt_navigator/get_state')
        self._bt_check_timer = None

        self._waypoints = self._load_waypoints()

        # Each row: {'start': int, 'end': int, 'entry': int, 'completed': bool}
        # 'entry' is the waypoint index the robot entered from (-1 = not started).
        self._rows = [
            {'start': i * 2, 'end': i * 2 + 1, 'entry': -1, 'completed': False}
            for i in range(len(self._waypoints) // 2)
        ]

        self._current_index = 0      # waypoint index currently being navigated to
        self._failed_goals = 0
        self._goal_handle = None
        self._mission_started = False
        self._amcl_received = False
        self._path_skip_triggered = False

        self._robot_x = 0.0
        self._robot_y = 0.0

        # Stuck detection state
        self._last_distance_remaining = float('inf')
        self._last_progress_time = None

        self.create_timer(2.0, self._publish_planned_path)
        self.create_timer(1.0, self._check_if_stuck)

        self._nav_plan_sub = self.create_subscription(
            Path, '/plan', self._on_nav_plan, 10)

        self.get_logger().info('Mission node started. Waiting for Nav2...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Publishing initial pose and waiting for AMCL...')

        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)

        self._publish_initial_pose()
        self._pose_retry_timer = self.create_timer(2.0, self._publish_initial_pose)

    # ------------------------------------------------------------------
    # Row helpers
    # ------------------------------------------------------------------

    def _row_of(self, wp_idx):
        """Return the row dict that contains wp_idx, or None."""
        for row in self._rows:
            if wp_idx in (row['start'], row['end']):
                return row
        return None

    def _row_index_of(self, wp_idx):
        """Return the index of the row that contains wp_idx, or -1."""
        for i, row in enumerate(self._rows):
            if wp_idx in (row['start'], row['end']):
                return i
        return -1

    def _other_end(self, row, wp_idx):
        """Given one endpoint of a row, return the other."""
        return row['end'] if wp_idx == row['start'] else row['start']

    def _row_for_y(self, y):
        """Row index whose Y corridor contains y, or None."""
        for i, row in enumerate(self._rows):
            row_y = self._waypoints[row['start']]['y']
            if abs(y - row_y) <= ROW_HALF_WIDTH:
                return i
        return None

    # ------------------------------------------------------------------
    # Nav plan monitoring (corridor-exit detection)
    # ------------------------------------------------------------------

    def _on_nav_plan(self, msg):
        if not msg.poses or self._goal_handle is None or self._path_skip_triggered:
            return
        if self._current_index >= len(self._waypoints):
            return

        current_row_idx = self._row_index_of(self._current_index)
        if current_row_idx < 0:
            return
        row_y = self._waypoints[self._current_index]['y']

        if abs(self._robot_y - row_y) > ROW_HALF_WIDTH:
            return

        for pose in msg.poses:
            py = pose.pose.position.y
            if abs(py - row_y) <= ROW_HALF_WIDTH:
                continue
            target_row = self._row_for_y(py)
            if target_row is None or target_row == current_row_idx:
                continue
            if self._rows[target_row]['completed']:
                self._path_skip_triggered = True
                self.get_logger().warn(
                    f'Plan exits into completed row {target_row + 1} '
                    f'(y={py:.2f}) — skipping waypoint {self._current_index + 1}.')
                self._goal_handle.cancel_goal_async()
                self._goal_handle = None
                self._skip_to_nearest_unvisited()
                return

    # ------------------------------------------------------------------
    # AMCL / localisation
    # ------------------------------------------------------------------

    def _on_amcl_pose(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

        if not self._amcl_received:
            self._amcl_received = True
            self.get_logger().info(
                f'AMCL pose received at x={self._robot_x:.2f}, '
                f'y={self._robot_y:.2f}. Waiting for bt_navigator...')
            if self._pose_retry_timer:
                self._pose_retry_timer.cancel()
                self._pose_retry_timer = None
            self._bt_check_timer = self.create_timer(0.5, self._check_bt_navigator)

    def _publish_initial_pose(self):
        if self._amcl_received:
            if self._pose_retry_timer:
                self._pose_retry_timer.cancel()
                self._pose_retry_timer = None
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = SPAWN_X
        msg.pose.pose.position.y = SPAWN_Y
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Initial pose published at ({SPAWN_X}, {SPAWN_Y}). Waiting for AMCL...')

    # ------------------------------------------------------------------
    # bt_navigator readiness
    # ------------------------------------------------------------------

    def _check_bt_navigator(self):
        if not self._bt_check_client.service_is_ready():
            return
        future = self._bt_check_client.call_async(GetState.Request())
        future.add_done_callback(self._on_bt_state)

    def _on_bt_state(self, future):
        try:
            state = future.result().current_state.id
        except Exception:
            return
        if state == 3:
            self._bt_check_timer.cancel()
            self._bt_check_timer = None
            if not self._mission_started:
                self._mission_started = True
                self.get_logger().info('bt_navigator active. Starting mission.')
                self._send_next_goal()

    # ------------------------------------------------------------------
    # Waypoint loading
    # ------------------------------------------------------------------

    def _load_waypoints(self):
        try:
            pkg_dir = get_package_share_directory('greenhouse_nav')
            yaml_file = os.path.join(pkg_dir, 'config', 'waypoints.yaml')
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)
            waypoints = data['waypoints']
            self.get_logger().info(f'Loaded {len(waypoints)} waypoints from {yaml_file}')
            return waypoints
        except Exception as e:
            self.get_logger().warn(f'Could not load waypoints YAML: {e}. Using hardcoded.')
            return [
                {'x': 0.5, 'y': 4.0, 'yaw': 0.0,  'label': 'Row 3 - Start'},
                {'x': 9.5, 'y': 4.0, 'yaw': 0.0,  'label': 'Row 3 - End'},
                {'x': 9.5, 'y': 6.0, 'yaw': 3.14, 'label': 'Row 4 - Start'},
                {'x': 0.5, 'y': 6.0, 'yaw': 3.14, 'label': 'Row 4 - End'},
                {'x': 0.5, 'y': 8.0, 'yaw': 0.0,  'label': 'Row 5 - Start'},
                {'x': 9.5, 'y': 8.0, 'yaw': 0.0,  'label': 'Row 5 - End'},
            ]

    # ------------------------------------------------------------------
    # Nearest unvisited row selection
    # ------------------------------------------------------------------

    def _nearest_unvisited_entry(self):
        """
        Find the closest endpoint of any unvisited row.
        Both endpoints are considered — whichever is nearest becomes the entry,
        the other becomes the exit.
        Returns the waypoint index to navigate to first, or None if all done.
        """
        best_idx = None
        best_dist = float('inf')
        for row in self._rows:
            if row['completed']:
                continue
            for wp_idx in (row['start'], row['end']):
                wp = self._waypoints[wp_idx]
                dist = math.hypot(wp['x'] - self._robot_x, wp['y'] - self._robot_y)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = wp_idx
        return best_idx

    # ------------------------------------------------------------------
    # Stuck detection
    # ------------------------------------------------------------------

    def _reset_stuck_state(self):
        self._last_distance_remaining = float('inf')
        self._last_progress_time = None

    def _check_if_stuck(self):
        if self._goal_handle is None or self._path_skip_triggered:
            return
        if self._last_progress_time is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._last_progress_time
        if elapsed > STUCK_TIME_THRESHOLD:
            self.get_logger().warn(
                f'Robot stuck for {elapsed:.1f}s at waypoint '
                f'{self._current_index + 1} — obstacle detected. '
                f'Skipping to nearest unvisited row.')
            self._path_skip_triggered = True
            self._reset_stuck_state()
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self._skip_to_nearest_unvisited()

    # ------------------------------------------------------------------
    # Goal sending
    # ------------------------------------------------------------------

    def _send_next_goal(self):
        self._reset_stuck_state()

        if self._current_index >= len(self._waypoints):
            entry = self._nearest_unvisited_entry()
            if entry is not None:
                self._current_index = entry
            else:
                self.get_logger().info('Mission complete! All rows finished.')
                return

        wp = self._waypoints[self._current_index]
        self.get_logger().info(
            f'Navigating to waypoint {self._current_index + 1}/{len(self._waypoints)}: '
            f'{wp["label"]} (x={wp["x"]}, y={wp["y"]})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)

        self._progress_pub.publish(goal_msg.pose)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Goal rejected for waypoint {self._current_index + 1}. Skipping row...')
            self._skip_to_nearest_unvisited()
            return
        self._goal_handle = goal_handle
        self._path_skip_triggered = False   # new goal is live — safe to clear
        self.get_logger().info('Goal accepted by Nav2.')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            wp_idx = self._current_index
            row = self._row_of(wp_idx)

            self.get_logger().info(
                f'Reached waypoint {wp_idx + 1}: {self._waypoints[wp_idx]["label"]}')

            if row is None:
                # Shouldn't happen, but recover gracefully
                self._current_index += 1
                self._send_next_goal()
                return

            if row['entry'] < 0:
                # First endpoint of this row reached — record entry, go to other end
                row['entry'] = wp_idx
                self._current_index = self._other_end(row, wp_idx)
                self._send_next_goal()
            else:
                # Second endpoint reached — row complete, find next row
                row['completed'] = True
                self.get_logger().info(
                    f'Row {self._row_index_of(wp_idx) + 1} completed.')
                entry = self._nearest_unvisited_entry()
                if entry is not None:
                    self._current_index = entry
                    self._send_next_goal()
                else:
                    self.get_logger().info('Mission complete! All rows finished.')

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f'Nav2 aborted waypoint {self._current_index + 1}. '
                f'No path found — skipping to nearest unvisited row.')
            self._skip_to_nearest_unvisited()

        else:
            self.get_logger().info(
                f'Goal cancelled for waypoint {self._current_index + 1}.')

    # ------------------------------------------------------------------
    # Skip logic
    # ------------------------------------------------------------------

    def _skip_to_nearest_unvisited(self):
        """Mark current row as skipped and go to nearest unvisited row."""
        self._failed_goals += 1
        row = self._row_of(self._current_index)
        if row is not None:
            row['completed'] = True
            self.get_logger().warn(
                f'Row {self._row_index_of(self._current_index) + 1} skipped. '
                f'Total skipped: {self._failed_goals}')

        entry = self._nearest_unvisited_entry()
        if entry is None:
            self.get_logger().info('All rows completed or skipped. Mission complete.')
            return

        self.get_logger().warn(
            f'Nearest unvisited entry: waypoint {entry + 1} '
            f'({self._waypoints[entry]["label"]}).')
        self._current_index = entry
        self._skip_timer = self.create_timer(2.0, self._delayed_next_goal)

    def _delayed_next_goal(self):
        self._skip_timer.cancel()
        self._skip_timer = None
        self._send_next_goal()

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------

    def _feedback_callback(self, feedback_msg):
        # Use Euclidean distance from AMCL pose to goal instead of Nav2's
        # distance_remaining, which stalls when replanning is frequent.
        if self._current_index >= len(self._waypoints):
            return
        wp = self._waypoints[self._current_index]
        dist = math.hypot(wp['x'] - self._robot_x, wp['y'] - self._robot_y)
        self.get_logger().debug(f'Distance to goal: {dist:.2f} m')

        now = self.get_clock().now().nanoseconds / 1e9
        if dist < self._last_distance_remaining - STUCK_DISTANCE_THRESHOLD:
            self._last_distance_remaining = dist
            self._last_progress_time = now

    # ------------------------------------------------------------------
    # RViz path visualisation
    # ------------------------------------------------------------------

    def _publish_planned_path(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wp in self._waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
            pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)
            path_msg.poses.append(pose)
        self._path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()