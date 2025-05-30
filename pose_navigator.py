#!/usr/bin/env python3
"""
pose_navigator.py
Save or navigate to a named pose for Hello Robot Stretch 3 using Nav2.

Usage
-----
1. Launch Nav2:

   ros2 launch stretch_nav2 navigation.launch.py \
        map:=/home/hello-robot/stretch_user/maps_group6/maps/map_group6.yaml

   • In RViz, press **Startup**.
   • Drop a **2D Pose Estimate** arrow.

2. While the robot is parked at the target spot:

   python3 pose_navigator.py --save room6

3. Later, send the robot back:

   python3 pose_navigator.py --goto room6
"""
import argparse
import json
import os
import sys
from copy import deepcopy

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from stretch_nav2.robot_navigator import BasicNavigator, TaskResult

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
POSE_FILE = os.path.expanduser("./Desktop/stretch_saved_location.json")
MAP_FRAME = "map"
BASE_FRAME = "base_footprint"       # Stretch publishes map → base_footprint

# --------------------------------------------------------------------------- #
# Helper functions
# --------------------------------------------------------------------------- #
def load_poses() -> dict:
    if os.path.isfile(POSE_FILE):
        with open(POSE_FILE, "r") as f:
            return json.load(f)
    return {}


def save_poses(data: dict) -> None:
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    with open(POSE_FILE, "w") as f:
        json.dump(data, f, indent=2)


# --------------------------------------------------------------------------- #
# Core routines
# --------------------------------------------------------------------------- #
def save_current_pose(name: str) -> None:
    """
    Capture the current map → base_footprint transform and store it under *name*.
    """
    rclpy.init()
    node = rclpy.create_node("pose_saver")

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    # Wait for the transform to arrive (max 10 s)
    timeout = Duration(seconds=10.0)
    start_time = node.get_clock().now()
    tf = None

    while rclpy.ok() and tf is None:
        try:
            tf = tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                rclpy.time.Time(),
                Duration(seconds=0.25),
            )
        except (LookupException, ExtrapolationException):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now() - start_time > timeout:
                node.get_logger().error(
                    "Transform not available after 10 s.\n"
                    "Is Nav2 active and did you set a 2D Pose Estimate?"
                )
                rclpy.shutdown()
                sys.exit(1)

    # Store pose
    pose_dict = {
        "x": tf.transform.translation.x,
        "y": tf.transform.translation.y,
        "z": tf.transform.translation.z,
        "qx": tf.transform.rotation.x,
        "qy": tf.transform.rotation.y,
        "qz": tf.transform.rotation.z,
        "qw": tf.transform.rotation.w,
    }

    poses = load_poses()
    poses[name] = pose_dict
    save_poses(poses)
    node.get_logger().info(f"Saved pose '{name}' to {POSE_FILE}")
    rclpy.shutdown()



def goto_pose(name: str) -> None:
    """
    Navigate Stretch to the pose stored under *name*.
    """
    poses = load_poses()
    if name not in poses:
        print(f"Pose '{name}' not found. Run --save {name} first.")
        sys.exit(1)

    target = poses[name]

    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    pose = PoseStamped()
    pose.header.frame_id = MAP_FRAME
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = target["x"]
    pose.pose.position.y = target["y"]
    pose.pose.position.z = target["z"]
    pose.pose.orientation.x = target["qx"]
    pose.pose.orientation.y = target["qy"]
    pose.pose.orientation.z = target["qz"]
    pose.pose.orientation.w = target["qw"]

    navigator.goToPose(pose)
    navigator.get_logger().info(f"Heading to '{name}' …")

    while not navigator.isTaskComplete():
        rclpy.spin_once(navigator, timeout_sec=0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info("Reached destination!")
    else:
        navigator.get_logger().info(f"Navigation result: {result}")
    rclpy.shutdown()


# --------------------------------------------------------------------------- #
# CLI entry‑point
# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--save", metavar="NAME", help="save current pose under NAME")
    group.add_argument("--goto", metavar="NAME", help="navigate to saved pose NAME")
    args = parser.parse_args()

    if args.save:
        save_current_pose(args.save)
    else:
        goto_pose(args.goto)


if __name__ == "__main__":
    main()
