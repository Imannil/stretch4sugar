#!/usr/bin/env python3

# Here is the Stretch4Sugar's Base alignment node for arm_back ArUco marker
# This node aligns the robot base 0.50m in front of the arm_back marker,
# rotates to face it, and finalizes orientation using TF data.
# Used to prep the robot for accurate glucometer arm approach.

import sys
import time
from math import atan2, sqrt
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException, Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_matrix
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus


class AlignToAruco(Node):
    def __init__(self, node, trans_base: TransformStamped, offset=0.50):
        self.node = node
        self.trans_base = trans_base
        self.offset = offset

        self.trajectory_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
        )

        if not self.trajectory_client.wait_for_server(timeout_sec=60.0):
            self.node.get_logger().error("Unable to connect to trajectory server.")
            sys.exit()

    def compute_difference(self):
        # Step 1: Extract marker pose from TF
        x, y, z, w = (
            self.trans_base.transform.rotation.x,
            self.trans_base.transform.rotation.y,
            self.trans_base.transform.rotation.z,
            self.trans_base.transform.rotation.w,
        )
        R = quaternion_matrix((x, y, z, w))

        # Step 2: Apply offset vector (robot stops 0.50m in front of the marker)
        P_dash = np.array([[0], [-self.offset], [0], [1]])
        P = np.array([
            [self.trans_base.transform.translation.x],
            [self.trans_base.transform.translation.y],
            [0],
            [1],
        ])
        X = np.matmul(R, P_dash)
        P_base = X + P

        # Step 3: Compute target relative pose
        base_position_x = P_base[0, 0]
        base_position_y = P_base[1, 0]

        phi = atan2(base_position_y, base_position_x)
        dist = sqrt(base_position_x**2 + base_position_y**2)

        _, _, z_rot_base = euler_from_quaternion([x, y, z, w])
        manual_offset = 0.25
        z_rot_base = -phi + z_rot_base + np.pi + manual_offset

        return phi, dist, z_rot_base

    def align_to_marker(self):
        # Compute movement values based on transform
        phi, dist, final_theta = self.compute_difference()

        def send_base_goal_blocking(joint_name, inc):
            point = JointTrajectoryPoint()
            point.positions = [inc]
            point.time_from_start = Duration(seconds=5.0).to_msg()

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [joint_name]
            goal.trajectory.points = [point]

            self.node.get_logger().info(f"[{joint_name}] Sending goal: {inc:.3f}")
            send_goal_future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.node.get_logger().error(f"Goal for {joint_name} was rejected!")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            result = result_future.result()

            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.node.get_logger().warn(
                    f"Goal for {joint_name} did not succeed: status {result.status}"
                )
            else:
                self.node.get_logger().info(f"Goal for {joint_name} succeeded.")

        send_base_goal_blocking("rotate_mobile_base", phi)
        send_base_goal_blocking("translate_mobile_base", dist)
        send_base_goal_blocking("rotate_mobile_base", final_theta)


def main():
    rclpy.init()
    node = Node("align_to_aruco_node")

    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)

    timeout_sec = 120.0
    start_time = time.time()

    node.declare_parameter("aruco_tag_name", "arm_back")
    aruco_tag_name = node.get_parameter("aruco_tag_name").get_parameter_value().string_value

    trans_base = None
    node.get_logger().info(f"Waiting for transform: base_link → {aruco_tag_name}")

    # Wait for transform from base_link to marker
    while rclpy.ok() and (time.time() - start_time < timeout_sec):
        try:
            if tf_buffer.can_transform("base_link", aruco_tag_name, Time()):
                trans_base = tf_buffer.lookup_transform("base_link", aruco_tag_name, Time())
                node.get_logger().info(f"Found transform to marker: {aruco_tag_name}")
                break
        except TransformException as ex:
            node.get_logger().warn(f"[Stretch4Sugar] TF lookup failed: {ex}")

        rclpy.spin_once(node)
        time.sleep(1)

    if trans_base is None:
        node.get_logger().error(f"[Stretch4Sugar] Timeout waiting for transform to {aruco_tag_name}")
        rclpy.shutdown()
        return

    try:
        align = AlignToAruco(node=node, trans_base=trans_base, offset=0.50)
        node.get_logger().info("Executing alignment...")
        align.align_to_marker()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()