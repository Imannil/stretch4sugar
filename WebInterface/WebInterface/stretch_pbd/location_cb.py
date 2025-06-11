#!/usr/bin/env python3
"""
pose_navigator.py — Navigate to saved poses
"""

import json
import pathlib
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

NAV_POSE_FILE = pathlib.Path.home() / 'Desktop/stretch_nav_poses.json'


class PoseNavigator(Node):
    """ROS 2 node that navigates to saved poses."""

    def __init__(self):
        super().__init__('pose_navigator')

        # Nav2 helper
        self.navigator = BasicNavigator()

        # Service for navigation
        self.create_service(
            Trigger,
            '/go_back',
            self._go_back_cb
        )

        self.get_logger().info('Waiting for Nav2...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('PoseNavigator ready - call /go_back to navigate')

    def _go_back_cb(self, request, response: Trigger.Response) -> Trigger.Response:
        """Navigate to the most recently saved pose."""
        # 1) ensure Nav2 is active (returns immediately if it already is)
        self.navigator.waitUntilNav2Active()

        self.navigator.clearAllCostmaps()              # remove stale obstacles
        self.navigator.changeControllerParams(         # tighten tolerances
        {'goal_checker.xy_goal_tolerance': 0.05,
         'goal_checker.yaw_goal_tolerance': 0.05})

        # 2) clear stale obstacles the robot left behind
        self.navigator.clearAllCostmaps()

        # 3) load the latest pose from disk
        saved_poses = self._load_poses()
        if not saved_poses:
            response.success = False
            response.message = 'No saved poses found'
            return response

        pose_data = saved_poses[-1]                 # newest entry
        pose_name = pose_data.get('name', '<unnamed>')

        # 4) build the goal
        goal = PoseStamped()
        goal.header.frame_id = pose_data.get('frame_id', 'map')
        goal.header.stamp = self.get_clock().now().to_msg()

        pos = pose_data['pose']['position']
        goal.pose.position.x = pos['x']
        goal.pose.position.y = pos['y']
        goal.pose.position.z = pos['z']

        ori = pose_data['pose']['orientation']
        goal.pose.orientation.x = ori['x']
        goal.pose.orientation.y = ori['y']
        goal.pose.orientation.z = ori['z']
        goal.pose.orientation.w = ori['w']

        self.get_logger().info(
            f'Going to “{pose_name}” at ({pos["x"]:.2f}, {pos["y"]:.2f})'
        )

        # 5) send goal and wait (≤ 60 s)
        self.navigator.goToPose(goal)
        start_time = time.time()
        while not self.navigator.isTaskComplete():
            if time.time() - start_time > 60.0:
                self.navigator.cancelTask()
                response.success = False
                response.message = 'Navigation timeout'
                return response
            time.sleep(0.1)

        # 6) report result
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            response.success = True
            response.message = f'Reached “{pose_name}”'
        else:
            response.success = False
            response.message = 'Navigation failed'

        return response


    def _load_poses(self) -> list[dict]:
        """Always return a *list* of pose-entries."""
        if NAV_POSE_FILE.exists():
            try:
                poses = json.loads(NAV_POSE_FILE.read_text())

                # convert old dict-format to list once
                if isinstance(poses, dict):
                    poses = sorted(
                        poses.values(),
                        key=lambda e: e.get('timestamp', 0)
                    )
                    self.get_logger().info(
                        f'Converted {len(poses)} old poses to list format'
                    )

                self.get_logger().info(f'Loaded {len(poses)} poses from disk')
                for entry in poses:                                   # ← safe
                    pos = entry["pose"]["position"]
                    self.get_logger().debug(
                        f'  - {entry["name"]}: '
                        f'({pos["x"]:.2f}, {pos["y"]:.2f})'
                    )
                return poses
            except json.JSONDecodeError as err:
                self.get_logger().error(f'Pose file corrupt: {err}')
        return []



def main():
    rclpy.init()
    node = PoseNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()