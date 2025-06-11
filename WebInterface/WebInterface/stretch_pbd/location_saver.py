#!/usr/bin/env python3
"""
pose_saver.py — Monitor AMCL and save poses on request
"""

import json
import pathlib
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

NAV_POSE_FILE = pathlib.Path.home() / 'Desktop/stretch_nav_poses.json'


class PoseSaver(Node):
    """ROS 2 node that monitors AMCL and saves poses on request."""

    def __init__(self):
        super().__init__('pose_saver')

        # QoS for AMCL subscription
        amcl_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Current AMCL pose
        self.current_pose: PoseWithCovarianceStamped | None = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_cb,
            amcl_qos
        )

        # Save pose trigger
        self.create_subscription(
            String,
            '/save_pose',
            self._save_pose_cb,
            10
        )

        self.saved_poses: list[dict] = self._load_poses() 
        self.get_logger().info('PoseSaver ready - monitoring AMCL')

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """Update current pose from AMCL."""
        self.current_pose = msg
        self.get_logger().debug(
            f'AMCL update: ({msg.pose.pose.position.x:.2f}, '
            f'{msg.pose.pose.position.y:.2f})'
        )

    def _save_pose_cb(self, msg: String) -> None:
        """Save current AMCL pose, replacing any previous entry."""
        data_parts = msg.data.strip().split(',')
        pose_name = data_parts[0] if data_parts else f'pose_{int(time.time())}'

        if self.current_pose is None:
            self.get_logger().error('Cannot save — no AMCL pose available')
            return

        entry = {
            'name': pose_name,
            'frame_id': self.current_pose.header.frame_id,
            'pose': {
                'position': {
                    'x': self.current_pose.pose.pose.position.x,
                    'y': self.current_pose.pose.pose.position.y,
                    'z': self.current_pose.pose.pose.position.z,
                },
                'orientation': {
                    'x': self.current_pose.pose.pose.orientation.x,
                    'y': self.current_pose.pose.pose.orientation.y,
                    'z': self.current_pose.pose.pose.orientation.z,
                    'w': self.current_pose.pose.pose.orientation.w,
                },
            },
            'timestamp': self.get_clock().now().nanoseconds,
        }

        # ——— replace the entire list with this single, newest entry ———
        self.saved_poses = [entry]

        self._dump_poses()
        self.get_logger().info(
            f'Saved "{pose_name}" at ({entry["pose"]["position"]["x"]:.2f}, '
            f'{entry["pose"]["position"]["y"]:.2f})'
        )


    def _load_poses(self) -> list[dict]:
        if NAV_POSE_FILE.exists():
            try:
                return json.loads(NAV_POSE_FILE.read_text())
            except json.JSONDecodeError as err:
                self.get_logger().warning(f'Pose file corrupt: {err}')
        return []                       # ← return an empty *list*

    def _dump_poses(self) -> None:
        NAV_POSE_FILE.write_text(json.dumps(self.saved_poses, indent=2))


def main():
    rclpy.init()
    node = PoseSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()