#!/usr/bin/env python3
"""
Mission node for greenhouse boustrophedon navigation.
Sends the robot through crop rows in a snake pattern using Nav2.
Handles failed goals by skipping to the next row.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # Action client to send goals to Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publisher to report progress
        self._progress_pub = self.create_publisher(
            PoseStamped, '/mission/current_waypoint', 10)

        # Waypoints: list of (x, y, yaw, label)
        self._waypoints = self._load_waypoints()
        self._current_index = 0
        self._failed_goals = 0

        self.get_logger().info('Mission node started. Waiting for Nav2...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting mission.')

        # Start mission
        self._send_next_goal()

    def _load_waypoints(self):
        """
        Boustrophedon waypoints through 3 crop rows.
        Pattern: start of row1 -> end of row1 -> start of row2 -> end of row2 -> ...
        """
        waypoints = [
            # Row 1 - going forward (east)
            {'x': -2.5, 'y':  0.0, 'yaw': 0.0,  'label': 'Row 1 - Start'},
            {'x':  2.5, 'y':  0.0, 'yaw': 0.0,  'label': 'Row 1 - End'},
            # Row 2 - going backward (west)
            {'x':  2.5, 'y':  2.1, 'yaw': 3.14, 'label': 'Row 2 - Start'},
            {'x': -2.5, 'y':  2.1, 'yaw': 3.14, 'label': 'Row 2 - End'},
            # Row 3 - going forward (east)
            {'x': -2.5, 'y':  4.2, 'yaw': 0.0,  'label': 'Row 3 - Start'},
            {'x':  2.5, 'y':  4.2, 'yaw': 0.0,  'label': 'Row 3 - End'},
        ]
        return waypoints

    def _send_next_goal(self):
        """Send the next waypoint to Nav2."""

        if self._current_index >= len(self._waypoints):
            self.get_logger().info('Mission complete! All rows finished.')
            return

        wp = self._waypoints[self._current_index]
        self.get_logger().info(
            f'Navigating to waypoint {self._current_index + 1}/'
            f'{len(self._waypoints)}: {wp["label"]} '
            f'(x={wp["x"]}, y={wp["y"]})'
        )

        # Build the goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (simplified for 2D)
        import math
        goal_msg.pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)

        # Publish progress
        self._progress_pub.publish(goal_msg.pose)

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Called when Nav2 accepts or rejects our goal."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Goal rejected for waypoint {self._current_index + 1}. Skipping...')
            self._skip_to_next_row()
            return

        self.get_logger().info('Goal accepted by Nav2.')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        """Called when Nav2 finishes navigating to a goal."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Reached waypoint {self._current_index + 1}: '
                f'{self._waypoints[self._current_index]["label"]}')
            self._current_index += 1
            self._send_next_goal()

        else:
            self.get_logger().warn(
                f'Failed to reach waypoint {self._current_index + 1}: '
                f'{self._waypoints[self._current_index]["label"]}. '
                f'Status: {status}. Skipping row...')
            self._skip_to_next_row()

    def _skip_to_next_row(self):
        """
        Recovery behaviour: skip to the start of the next row.
        We skip 2 waypoints at a time because each row has 2 waypoints
        (start and end). This avoids getting stuck on a blocked row.
        """
        self._failed_goals += 1
        self.get_logger().warn(
            f'Recovery: skipping to next row. '
            f'Total failed goals so far: {self._failed_goals}')

        # Skip to next even index (start of next row)
        self._current_index = (self._current_index // 2 + 1) * 2
        self._send_next_goal()

    def _feedback_callback(self, feedback_msg):
        """Called periodically with navigation progress."""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Distance remaining: '
            f'{feedback.distance_remaining:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()