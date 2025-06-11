#!/usr/bin/env python3

#pose_navigator.py

import json
import pathlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from rclpy.executors import MultiThreadedExecutor
import time


# Path to saved navigation poses
SCRIPT_DIR      = pathlib.Path(__file__).resolve().parent
NAV_POSE_FILE   = SCRIPT_DIR / 'stretch_nav_poses.json'


class PoseNavigator(Node):
    def __init__(self):
        super().__init__('pose_navigator')
        
        # Navigator instance
        self.navigator = BasicNavigator()
        
        # TF2 for getting current robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Current pose from AMCL
        self.current_pose = None
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )
        
        # Services
        self.save_pose_sub = self.create_subscription(
            String,
            '/save_pose',
            self._save_pose_callback,
            10
        )
        
        self.go_back_srv = self.create_service(
            Trigger,
            '/go_back',
            self._go_back_callback
        )
        
        # Load existing poses
        self.saved_poses = self._load_poses()
        
        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for Nav2 to be ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready! Pose Navigator initialized.')
        
    def _amcl_pose_callback(self, msg):
        """Store the current AMCL pose estimate"""
        self.current_pose = msg
        
    def _get_current_pose_from_tf(self):
        """Get current robot pose from TF tree (map -> base_link)"""
        try:
            # Get transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Convert to PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot pose from TF: {ex}')
            return None
            
    def _save_pose_callback(self, msg):
        """Save current navigation pose to JSON file"""
        try:
            # Parse the message to get pose name
            if ',' in msg.data:
                pose_name = msg.data.split(',')[0].strip()
            else:
                pose_name = msg.data.strip()
                
            # Get current pose from TF or AMCL
            current_pose = self._get_current_pose_from_tf()
            
            if current_pose is None and self.current_pose is not None:
                # Fallback to AMCL pose if TF fails
                current_pose = PoseStamped()
                current_pose.header = self.current_pose.header
                current_pose.pose = self.current_pose.pose.pose
                
            if current_pose is None:
                self.get_logger().error('Unable to get current robot pose')
                return
                
            # Create pose data
            pose_data = {
                'name': pose_name,
                'pose': {
                    'position': {
                        'x': current_pose.pose.position.x,
                        'y': current_pose.pose.position.y,
                        'z': current_pose.pose.position.z
                    },
                    'orientation': {
                        'x': current_pose.pose.orientation.x,
                        'y': current_pose.pose.orientation.y,
                        'z': current_pose.pose.orientation.z,
                        'w': current_pose.pose.orientation.w
                    }
                },
                'timestamp': self.get_clock().now().nanoseconds#!/usr/bin/env python3
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
            }
            
            # Save to dictionary (overwrite if exists)
            self.saved_poses[pose_name] = pose_data
            
            # Write to file
            self._save_poses()
            
            self.get_logger().info(
                f'Saved navigation pose "{pose_name}" at position '
                f'({current_pose.pose.position.x:.3f}, {current_pose.pose.position.y:.3f})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to save pose: {e}')
            
    def _go_back_callback(self, request, response):
        """Navigate back to the most recently saved pose"""
        try:
            if not self.saved_poses:
                response.success = False
                response.message = 'No saved poses found'
                return response
                
            # Get the most recent pose (last saved)
            pose_name = list(self.saved_poses.keys())[-1]
            pose_data = self.saved_poses[pose_name]
            
            # Create PoseStamped for navigation
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose_data['pose']['position']['x']
            goal_pose.pose.position.y = pose_data['pose']['position']['y']
            goal_pose.pose.position.z = pose_data['pose']['position']['z']
            goal_pose.pose.orientation.x = pose_data['pose']['orientation']['x']
            goal_pose.pose.orientation.y = pose_data['pose']['orientation']['y']
            goal_pose.pose.orientation.z = pose_data['pose']['orientation']['z']
            goal_pose.pose.orientation.w = pose_data['pose']['orientation']['w']
            
            self.get_logger().info(
                f'Navigating back to pose "{pose_name}" at '
                f'({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})'
            )
            
            # Send navigation goal
            self.navigator.goToPose(goal_pose)
            
            # Wait for navigation to complete (with timeout)
            start_time = time.time()
            timeout = 60.0  # 60 second timeout
            
            while not self.navigator.isTaskComplete():
                if time.time() - start_time > timeout:
                    self.navigator.cancelTask()
                    response.success = False
                    response.message = 'Navigation timeout'
                    return response
                    
                # Check feedback periodically
                feedback = self.navigator.getFeedback()
                if feedback:
                    self.get_logger().debug(
                        f'Distance remaining: {feedback.distance_remaining:.2f}m'
                    )
                    
                time.sleep(0.1)
                
            # Check result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                response.success = True
                response.message = f'Successfully navigated to pose "{pose_name}"'
            elif result == TaskResult.CANCELED:
                response.success = False
                response.message = 'Navigation was canceled'
            elif result == TaskResult.FAILED:
                response.success = False
                response.message = 'Navigation failed'
            else:
                response.success = False
                response.message = 'Unknown navigation result'
                
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')
            response.success = False
            response.message = f'Navigation error: {e}'
            
        return response
        
    def go_to_named_pose(self, pose_name):
        """Navigate to a specific named pose"""
        if pose_name not in self.saved_poses:
            self.get_logger().error(f'Pose "{pose_name}" not found')
            return False
            
        pose_data = self.saved_poses[pose_name]
        
        # Create PoseStamped for navigation
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_data['pose']['position']['x']
        goal_pose.pose.position.y = pose_data['pose']['position']['y']
        goal_pose.pose.position.z = pose_data['pose']['position']['z']
        goal_pose.pose.orientation.x = pose_data['pose']['orientation']['x']
        goal_pose.pose.orientation.y = pose_data['pose']['orientation']['y']
        goal_pose.pose.orientation.z = pose_data['pose']['orientation']['z']
        goal_pose.pose.orientation.w = pose_data['pose']['orientation']['w']
        
        self.get_logger().info(
            f'Navigating to pose "{pose_name}" at '
            f'({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})'
        )
        
        return self.navigator.goToPose(goal_pose)
        
    def _load_poses(self):
        """Load saved poses from JSON file"""
        if NAV_POSE_FILE.exists():
            try:
                data = json.loads(NAV_POSE_FILE.read_text())
                self.get_logger().info(f'Loaded {len(data)} saved poses')
                return data
            except Exception as e:
                self.get_logger().warning(f'Failed to load poses: {e}')
                return {}
        return {}
        
    def _save_poses(self):
        """Save poses to JSON file"""
        try:
            NAV_POSE_FILE.write_text(json.dumps(self.saved_poses, indent=2))
        except Exception as e:
            self.get_logger().error(f'Failed to save poses to file: {e}')


def main():
    rclpy.init()
    
    node = PoseNavigator()
    
    # Use multi-threaded executor for handling callbacks during navigation
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(node.navigator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
