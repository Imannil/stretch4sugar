# #!/usr/bin/env python3

# import sys
# import json
# import time
# from math import atan2, sqrt
# from pathlib import Path

# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# from rclpy.duration import Duration
# from rclpy.action import ActionClient
# from action_msgs.msg import GoalStatus

# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint


# POSE_FILE = Path.home() / "Desktop/stretch_saved_poses.json"
# POSE_NAME = "patient_arm_avg"
# OFFSET = 0.75


# class AlignToSavedPose(Node):
#     def __init__(self):
#         super().__init__("align_to_saved_pose")
#         self.client = ActionClient(
#             self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory"
#         )
#         if not self.client.wait_for_server(timeout_sec=30.0):
#             self.get_logger().error("Could not connect to trajectory server.")
#             sys.exit(1)

#     def load_pose(self):
#         with open(POSE_FILE, "r") as f:
#             poses = json.load(f)
#         if POSE_NAME not in [p["name"] for p in poses]:
#             self.get_logger().error(f"Pose '{POSE_NAME}' not found.")
#             sys.exit(1)
#         return poses[0]["transform"]  # Simplified, since we only store one

#     def compute_alignment(self, transform):
#         # Extract position
#         x = transform["translation"]["x"]
#         y = transform["translation"]["y"]

#         # Orientation (not used for translation)
#         qx = transform["rotation"]["x"]
#         qy = transform["rotation"]["y"]
#         qz = transform["rotation"]["z"]
#         qw = transform["rotation"]["w"]

#         # Create offset transformation in marker frame
#         from tf_transformations import quaternion_matrix, euler_from_quaternion

#         R = quaternion_matrix([qx, qy, qz, qw])
#         P_dash = np.array([[0], [-OFFSET], [0], [1]])
#         P = np.array([[x], [y], [0], [1]])
#         goal = np.matmul(R, P_dash) + P
#         goal_x, goal_y = goal[0, 0], goal[1, 0]

#         phi = atan2(goal_y, goal_x)
#         dist = sqrt(goal_x**2 + goal_y**2)
#         _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
#         theta_final = -phi + yaw + np.pi

#         return phi, dist, theta_final

#     def send_goal(self, joint_name, position):
#         point = JointTrajectoryPoint()
#         point.positions = [position]
#         point.time_from_start = Duration(seconds=4.0).to_msg()

#         goal = FollowJointTrajectory.Goal()
#         goal.trajectory.joint_names = [joint_name]
#         goal.trajectory.points = [point]

#         self.get_logger().info(f"Sending goal to {joint_name}: {position:.3f}")
#         future = self.client.send_goal_async(goal)
#         rclpy.spin_until_future_complete(self, future)
#         goal_handle = future.result()

#         if not goal_handle.accepted:
#             self.get_logger().error(f"Goal to {joint_name} rejected.")
#             return

#         result_future = goal_handle.get_result_async()
#         rclpy.spin_until_future_complete(self, result_future)
#         result = result_future.result()

#         if result.status != GoalStatus.STATUS_SUCCEEDED:
#             self.get_logger().warn(f"{joint_name} goal failed: {result.status}")
#         else:
#             self.get_logger().info(f"{joint_name} goal succeeded.")

#     def align(self):
#         transform = self.load_pose()
#         phi, dist, final_theta = self.compute_alignment(transform)

#         self.send_goal("rotate_mobile_base", phi)
#         self.send_goal("translate_mobile_base", dist)
#         self.send_goal("rotate_mobile_base", final_theta)


# def main():
#     rclpy.init()
#     node = AlignToSavedPose()
#     try:
#         node.align()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3

import sys
import json
import time
from math import atan2, sqrt
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf_transformations import quaternion_matrix, euler_from_quaternion

POSE_FILE = Path.home() / "Desktop/stretch_saved_poses.json"
POSE_NAME = "base_right"  # match the marker name in your pose logger
OFFSET = 0.75  # meters behind the marker in its negative Y axis


class AlignToSavedPose(Node):
    def __init__(self):
        super().__init__("align_to_saved_pose")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/stretch_controller/follow_joint_trajectory"
        )
        self.get_logger().info("Waiting for trajectory server...")
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("Could not connect to trajectory server.")
            sys.exit(1)
        self.get_logger().info("Trajectory server connected.")

    def load_pose(self):
        if not POSE_FILE.exists():
            self.get_logger().error(f"Pose file not found: {POSE_FILE}")
            sys.exit(1)

        with open(POSE_FILE, "r") as f:
            poses = json.load(f)

        for pose in poses:
            if pose["name"] == POSE_NAME:
                self.get_logger().info(f"Found pose: {POSE_NAME}")
                return pose["transform"]

        self.get_logger().error(f"Pose '{POSE_NAME}' not found in file.")
        sys.exit(1)

    def compute_alignment(self, transform):
        x = transform["translation"]["x"]
        y = transform["translation"]["y"]
        qx = transform["rotation"]["x"]
        qy = transform["rotation"]["y"]
        qz = transform["rotation"]["z"]
        qw = transform["rotation"]["w"]

        R = quaternion_matrix([qx, qy, qz, qw])
        P_dash = np.array([[0], [-OFFSET], [0], [1]])  # offset behind marker
        P = np.array([[x], [y], [0], [1]])
        goal = np.matmul(R, P_dash) + P
        goal_x, goal_y = goal[0, 0], goal[1, 0]

        phi = atan2(goal_y, goal_x)
        dist = sqrt(goal_x**2 + goal_y**2)
        _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
        theta_final = -phi + yaw + np.pi

        self.get_logger().info(f"Computed alignment:")
        self.get_logger().info(f"  - Goal offset: ({goal_x:.3f}, {goal_y:.3f})")
        self.get_logger().info(f"  - Rotate: {phi:.3f}, Translate: {dist:.3f}, Final Rotate: {theta_final:.3f}")

        return phi, dist, theta_final

    def send_goal(self, joint_name, position):
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(seconds=4.0).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint_name]
        goal.trajectory.points = [point]

        self.get_logger().info(f"Sending goal to {joint_name}: {position:.3f}")
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to {joint_name} rejected.")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"{joint_name} goal failed: status {result.status}")
        else:
            self.get_logger().info(f"{joint_name} goal succeeded.")

    def align(self):
        transform = self.load_pose()
        phi, dist, theta = self.compute_alignment(transform)

        self.send_goal("rotate_mobile_base", phi)
        self.send_goal("translate_mobile_base", dist)
        self.send_goal("rotate_mobile_base", theta)


def main():
    rclpy.init()
    node = AlignToSavedPose()
    try:
        node.align()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()





    # ros2 launch stretch_core stretch_driver.launch.py mode:=position
    # chmod +x ~/team6/aruco_marker_alignment.py
    # python3 ~/team6/aruco_marker_alignment.py