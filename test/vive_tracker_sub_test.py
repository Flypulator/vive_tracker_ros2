import pytest

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ViveTrackerTestSubscriber(Node):
    def __init__(self):
        super().__init__('vive_tracker_test_subscriber')
        self.subscription = self.create_subscription(Odometry, 'LHR_AEB9F7F5_odom', self.listener_callback,
                                                     qos_profile_sensor_data)

    def listener_callback(self, msg):
        log_str = f"x: {msg.pose.pose.position.x},y: {msg.pose.pose.position.y},z: {msg.pose.pose.position.z}"
        self.get_logger().info(log_str)


@pytest.mark.copyright
@pytest.mark.linter
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ViveTrackerTestSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
