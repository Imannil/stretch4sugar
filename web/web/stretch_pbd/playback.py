#!/usr/bin/env python3
import json
import pathlib
import rclpy
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import hello_helpers.hello_misc as hm  # HelloNode base class


# Path to saved poses
POSE_FILE = pathlib.Path.home() / 'Desktop/stretch_saved_poses.json'


class PosePlayer(hm.HelloNode):
    def __init__(self):
        super().__init__()

    def run(self):
        self.main('pose_player', 'pose_player', wait_for_first_pointcloud=False)
        self.poses = json.loads(POSE_FILE.read_text())
        print(self.poses)
        print(f"{len(self.poses)} poses...")

        self.create_service(Trigger, '/pose_player', self._play_cb)
        self.get_logger().info('PosePlayer ready — call /pose_player to start')

    def _play_cb(self, request, response):
        try:
            for p in self.poses:
                pose = self._dict_to_ps(p)
                self._goto(pose)

            response.success = True
            response.message = 'Pose sequence complete'
        except Exception as e:
            self.get_logger().error(f"Playback failed: {e}")
            response.success = False
            response.message = f"Playback failed: {e}"
        return response

    def _goto(self, ps: PoseStamped):
        x, y, z = ps.pose.position.x, ps.pose.position.y, ps.pose.position.z
        print(f"x, y, z {x}, {y}, {z}")
        cmd = {
            'translate_mobile_base': max(-1.0, min(1.0, x)),
            'joint_arm'         : max(0.0, min(0.5, y)),
            'joint_lift'           : max(0.1, min(1.2, z)),
        }
        self.move_to_pose(cmd)
        print('after move to pose')

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
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
