import json, pathlib, rclpy
from rclpy.node        import Node
from rclpy.time        import Time
from std_msgs.msg      import String
from tf2_ros           import TransformException
from tf2_ros.buffer    import Buffer
from tf2_ros.transform_listener import TransformListener

POSE_FILE = pathlib.Path.home() / './Desktop/stretch_saved_poses.json'

class FrameListener(Node):
    def __init__(self):
        super().__init__('stretch_tf_listener')

        self.declare_parameter('target_frame', 'link_grasp_center')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # load any saved poses -----------------------------
        self.poses = self._load_from_disk()

        # subscriber to trigger pose save ------------------
        self.create_subscription(
            String,                                    #  "Pose A,base_link"
            '/save_pose',
            self._save_pose_cb,
            10
        )

        # ---------- timer -------------------------------
        # time_period = 1.0   # seconds
        # self.timer = self.create_timer(time_period, self.on_timer)

    # ---------------------------------------------------------------------
    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel   = 'fk_link_mast'

        try:
            now   = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.get_logger().info(
            f'live pose of {from_frame_rel} w.r.t. {to_frame_rel}: {trans.transform.translation.x:.3f}, '
            f'{trans.transform.translation.y:.3f}, {trans.transform.translation.z:.3f}')

    # ---------------------------------------------------------------------
    def _save_pose_cb(self, msg: String):
        """Store the current grasp‑frame transform when a message arrives."""
        print("callback")
        try:
            name, frame = [p.strip() for p in msg.data.split(',')]
        except ValueError:
            self.get_logger().error('Expected "<name>,<frame>", got "%s"', msg.data)
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                frame,
                self.target_frame,    # source = link_grasp_center
                Time())
        except TransformException as ex:
            self.get_logger().warning('TF lookup failed: %s', ex)
            return

        self.poses.append({
            'name'     : name,
            'frame'    : frame,
            'transform': {
                'translation': {
                    'x': trans.transform.translation.x,
                    'y': trans.transform.translation.y,
                    'z': trans.transform.translation.z,
                },
                'rotation'   : {
                    'x': trans.transform.rotation.x,
                    'y': trans.transform.rotation.y,
                    'z': trans.transform.rotation.z,
                    'w': trans.transform.rotation.w,
                }
            }
        })
        self._save_to_disk()
        self.get_logger().info(f'Saved pose “{name}” in frame {frame}')

    # ---------- helpers to load / save ------------------------------------
    def _load_from_disk(self):
        if POSE_FILE.exists():
            return json.loads(POSE_FILE.read_text())
        return []

    def _save_to_disk(self):
        POSE_FILE.write_text(json.dumps(self.poses, indent=2))

# -------------------------------------------------------------------------
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

