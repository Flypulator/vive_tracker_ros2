import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class ViveWorld(Node):

    def __init__(self):
        super().__init__('vive_world')
        with open('../vive_config.yaml', 'r') as file:
            self.vive_config = yaml.safe_load(file)

        timer_period = 1 / self.vive_config['rate']  # seconds
        self.timer = self.create_timer(timer_period, self.broadcast_timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

    def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'vive_world'

        [offset_pos, offset_quat] = self.vive_config['world_frame_pose_offset']
        t.transform.translation.x = float(offset_pos[0])
        t.transform.translation.y = float(offset_pos[1])
        t.transform.translation.z = float(offset_pos[2])
        t.transform.rotation.x = float(offset_quat[0])
        t.transform.rotation.y = float(offset_quat[1])
        t.transform.rotation.z = float(offset_quat[2])
        t.transform.rotation.w = float(offset_quat[3])

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    vive_world = ViveWorld()
    try:
        print("Starting to spin ViveWorld node")
        rclpy.spin(vive_world)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
