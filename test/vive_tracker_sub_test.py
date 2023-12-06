import pytest

import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class ViveTrackerTestSubscriber(Node):
    def __init__(self):
        super().__init__('vive_tracker_test_subscriber')
        topic_name = 'LHR_AEB9F7F5'
        self.subscription = self.create_subscription(String, topic_name, self.listener_callback,
                                                     qos_profile_sensor_data)
        print(f"Subscription to device {topic_name} set up!")

    def listener_callback(self, msg):
        log_str = msg.data
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
