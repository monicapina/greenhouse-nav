#!/usr/bin/env python3
"""
Mission node for greenhouse boustrophedon navigation.
Sends the robot through crop rows in a snake pattern using Nav2.
Handles failed goals by skipping to the next row.
Recovery is implemented at mission-node level with a timeout mechanism.
Trade-off: simpler than a custom BT but robot waits for Nav2 recovery cycle
before the timeout triggers.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import yaml
import os
import math
from ament_index_python.packages import get_package_share_directory


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # Action client to send goals to Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') #action server name is 'navigate_to_pose'

        # Publisher to report progress
        self._progress_pub = self.create_publisher(
            PoseStamped, '/mission/current_waypoint', 10) #Publish the current waypoint to a topic for monitoring

        # Load waypoints from YAML file
        self._waypoints = self._load_waypoints()
        self._current_index = 0 
        self._failed_goals = 0 
        self._goal_handle = None

        # Timeout: cancel goal after 30 seconds if not reached
        self._timeout_seconds = 90.0
        self._timeout_timer = None

        self.get_logger().info('Mission node started. Waiting for Nav2...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting mission.')

        # Start mission
        self._send_next_goal()

    def _load_waypoints(self):
        """
        Load boustrophedon waypoints from config/waypoints.yaml.
        Falls back to hardcoded waypoints if file not found.
        """
        try:
            pkg_dir = get_package_share_directory('greenhouse_nav')
            yaml_file = os.path.join(pkg_dir, 'config', 'waypoints.yaml')

            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)

            waypoints = data['waypoints']
            self.get_logger().info(
                f'Loaded {len(waypoints)} waypoints from {yaml_file}')
            return waypoints

        except Exception as e:
            self.get_logger().warn(
                f'Could not load waypoints YAML: {e}. Using hardcoded waypoints.')
            return [
                {'x': -2.0, 'y': 0.0,  'yaw': 0.0,  'label': 'Row 1 - Start'},
                {'x':  2.0, 'y': 0.0,  'yaw': 0.0,  'label': 'Row 1 - End'},
                {'x':  2.0, 'y': 2.1,  'yaw': 3.14, 'label': 'Row 2 - Start'},
                {'x': -2.0, 'y': 2.1,  'yaw': 3.14, 'label': 'Row 2 - End'},
                {'x': -2.0, 'y': 4.2,  'yaw': 0.0,  'label': 'Row 3 - Start'},
                {'x':  2.0, 'y': 4.2,  'yaw': 0.0,  'label': 'Row 3 - End'},
            ]

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
        goal_msg.pose.header.frame_id = 'map' #Waypoints are defined in the map frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg() #Check the current time for the header stamp
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (simplified for 2D). Pitch and roll are zero for ground robots.
        goal_msg.pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0) 
        goal_msg.pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0) 

        # Publish progress. Current waypoint is published to a topic for monitoring.
        self._progress_pub.publish(goal_msg.pose)

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback #Information about the progress
        ) 
        self._send_goal_future.add_done_callback(self._goal_response_callback) #Nav2 will  accepts or rejects the goal

    def _goal_response_callback(self, future): #
        """Called when Nav2 accepts or rejects our goal."""
        goal_handle = future.result() # Retrieve the goal handle from the future result

        if not goal_handle.accepted: #Rejected goal. Skip to next row.
            self.get_logger().warn(
                f'Goal rejected for waypoint {self._current_index + 1}. Skipping...')
            self._skip_to_next_row()
            return

        # Save goal handle so we can cancel it if needed
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2.')

        # Start timeout timer
        self._timeout_timer = self.create_timer(
            self._timeout_seconds, self._timeout_callback) #Init timer

        self._result_future = goal_handle.get_result_async() #nav2 will return the result of the goal when it is finished or canceled. This is an asynchronous call that returns a future.
        self._result_future.add_done_callback(self._result_callback) #when result is ready, call the result callback to handle the result of the goal.

    def _timeout_callback(self):
        """Called if robot takes too long — cancel goal and skip row."""
        self.get_logger().warn(
            f'Timeout reached for waypoint {self._current_index + 1}. '
            f'Cancelling goal and skipping row...')

        # Cancel the timer
        self._timeout_timer.cancel()
        self._timeout_timer = None

        # Cancel the Nav2 goal
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        self._skip_to_next_row()

    def _result_callback(self, future):
        """Called when Nav2 finishes navigating to a goal."""

        # Cancel timeout timer if goal finished before timeout
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

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