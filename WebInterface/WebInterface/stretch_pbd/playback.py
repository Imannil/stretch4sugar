#!/usr/bin/env python3
# playback.py

import json
import pathlib
import rclpy
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import hello_helpers.hello_misc as hm
from rclpy.executors import MultiThreadedExecutor


# Path to saved poses
POSE_FILE = pathlib.Path.home() / './Desktop/stretch_saved_poses.json'


class PosePlayer(hm.HelloNode):
    def __init__(self):
        super().__init__()

    def run(self):
        self.main('pose_player', 'pose_player', wait_for_first_pointcloud=False)
        self.poses = json.loads(POSE_FILE.read_text())
        print(f"Loaded {len(self.poses)} poses from {POSE_FILE}")

        self.create_service(Trigger, '/pose_player', self._play_cb)
        self.get_logger().info('PosePlayer ready — call /pose_player to start')

    def _play_cb(self, request, response):
        try:
        
            self.poses = json.loads(POSE_FILE.read_text())

            for p in self.poses:
                pose = self._dict_to_ps(p)
                self._goto(pose, p)

            response.success = True
            response.message = 'Pose sequence complete'
        except Exception as e:
            self.get_logger().error(f"Playback failed: {e}")
            response.success = False
            response.message = f"Playback failed: {e}"
        return response


    def _goto(self, ps, pose_dict):
        cmd = {}

        if 'joint_lift' in pose_dict:
            cmd['joint_lift'] = pose_dict['joint_lift']

        for src, dst in [
            ('arm_extension',    'wrist_extension'),
            ('gripper_aperture', 'gripper_aperture'),
            ('wrist_yaw',        'joint_wrist_yaw'),
            ('wrist_pitch',      'joint_wrist_pitch'),
        ]:
            if src in pose_dict:
                cmd[dst] = pose_dict[src]

        self.get_logger().info(f'Sending command: {cmd}')
        self.move_to_pose(cmd, blocking=True)
        self.get_logger().info('Movement completed')

    @staticmethod
    def _dict_to_ps(d):
        ps = PoseStamped()
        ps.header.frame_id = d['frame']
        ps.pose.position.x = d['transform']['translation']['x']
        ps.pose.position.y = d['transform']['translation']['y']
        ps.pose.position.z = d['transform']['translation']['z']
        return ps


def main():
    node = PosePlayer()
    node.run()

    exec_ = MultiThreadedExecutor(num_threads=2)
    exec_.add_node(node)

    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec_.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()