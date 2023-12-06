import pytest

import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ViveTrackerTestSubscriber(Node):
    def __init__(self):
        super().__init__('vive_tracker_test_subscriber')
        topic_name = 'drone_tracker'
        self.subscription_string = self.create_subscription(String, topic_name, self.listener_callback_string, qos_profile_sensor_data)
        self.subscription_odom = self.create_subscription(Odometry, f"{topic_name}_odom", self.listener_callback_odom, qos_profile_sensor_data)
        print(f"Subscription to device {topic_name} set up!")

    def listener_callback_string(self, msg):
        log_str = msg.data
        self.get_logger().info(log_str)

    def listener_callback_odom(self, msg):
        log_str = f"omega_x: {msg.twist.twist.angular.x}, omega_y: {msg.twist.twist.angular.y}, omega_z: {msg.twist.twist.angular.z}"
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
