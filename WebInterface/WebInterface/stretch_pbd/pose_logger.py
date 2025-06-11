import json, pathlib, rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState

POSE_FILE = pathlib.Path.home() / './Desktop/stretch_saved_poses.json'

class FrameListener(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.declare_parameter('target_frame', 'link_grasp_center')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store current joint values
        self.joint_lift = 0.0
        self.arm_extension = 0.0
        self.gripper_aperture = 0.0
        self.wrist_yaw = 0.0      # Gripper rotation
        self.wrist_pitch = 0.0    # Gripper up/down pitch
        
        # Subscribe to joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/stretch/joint_states',
            self._joint_states_cb,
            10
        )

        # Load any saved poses
        self.poses = self._load_from_disk()

        # Subscriber to trigger pose save
        self.create_subscription(
            String,
            '/save_pose',
            self._save_pose_cb,
            10
        )

    def _joint_states_cb(self, msg: JointState):
        """Store arm extension, gripper, and wrist orientation values from joint states"""
        try:
            # current lift position (prismatic joint)
            if 'joint_lift' in msg.name:
                idx = msg.name.index('joint_lift')
                self.joint_lift = msg.position[idx]

            # Get arm extension
            if 'wrist_extension' in msg.name:
                idx = msg.name.index('wrist_extension')
                self.arm_extension = msg.position[idx]
            
            # Get gripper aperture
            if 'gripper_aperture' in msg.name:
                idx = msg.name.index('gripper_aperture')
                self.gripper_aperture = msg.position[idx]
            elif 'joint_gripper_finger_left' in msg.name:
                idx = msg.name.index('joint_gripper_finger_left')
                self.gripper_aperture = msg.position[idx] * 2.0
            
            # Get wrist yaw (gripper rotation)
            if 'joint_wrist_yaw' in msg.name:
                idx = msg.name.index('joint_wrist_yaw')
                self.wrist_yaw = msg.position[idx]
            
            # Get wrist pitch (gripper up/down)
            if 'joint_wrist_pitch' in msg.name:
                idx = msg.name.index('joint_wrist_pitch')
                self.wrist_pitch = msg.position[idx]
                
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Joint extraction error: {e}')

    def _save_pose_cb(self, msg: String):
        """Save current transform and joint values including gripper orientation"""
        print("Saving pose...")
        try:
            name, frame = [p.strip() for p in msg.data.split(',')]
        except ValueError:
            self.get_logger().error('Expected "<name>,<frame>", got "%s"', msg.data)
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                frame,
                self.target_frame,
                Time())
        except TransformException as ex:
            self.get_logger().warning('TF lookup failed: %s', ex)
            return

        # Create pose dict with all joint values including gripper orientation
        pose_data = {
            'name': name,
            'frame': frame,
            'transform': {
                'translation': {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z,
                },
                'rotation': {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w,
                }
            },
            'joint_lift':      self.joint_lift, 
            'arm_extension': self.arm_extension,
            'gripper_aperture': self.gripper_aperture,
            'wrist_yaw': self.wrist_yaw,        # Gripper rotation
            'wrist_pitch': self.wrist_pitch     # Gripper up/down pitch
        }
        
        self.poses = [pose_data]  # Replace with single pose for now
        
        self._save_to_disk()
        self.get_logger().info(
            f'Saved pose "{name}" in frame {frame} with:\n'
            f'  lift={self.joint_lift:.3f}\n'
            f'  arm_extension={self.arm_extension:.3f}\n'
            f'  gripper_aperture={self.gripper_aperture:.3f}\n'
            f'  wrist_yaw={self.wrist_yaw:.3f} (rotation)\n'
            f'  wrist_pitch={self.wrist_pitch:.3f} (pitch)'
        )

    def _load_from_disk(self):
        if POSE_FILE.exists():
            return json.loads(POSE_FILE.read_text())
        return []

    def _save_to_disk(self):
        POSE_FILE.write_text(json.dumps(self.poses, indent=2))


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()