#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.qos import QoSProfile


def main():
    rclpy.init()

    node = rclpy.create_node('vive_tracker_reset')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    tf_count = 0
    zero_tf = tf_buffer.lookup_transform('vive_world', 'vive_world', node.get_clock().now())
    br = tf2_ros.TransformBroadcaster(node)

    all_topics = node.get_topic_names_and_types()
    all_topic_names = [i[0] for i in all_topics]
    vive_topics = [topic_name for topic_name in all_topic_names if "/vive/" in topic_name]
    tracker_name = ""
    # Get the first tracked device that isn't a reference frame.
    for topic_name in vive_topics:
        if "LHB" not in topic_name:
            tracker_name = topic_name.replace('/vive/', '').replace('_odom', '')
            print(tracker_name)
            break

    rate = node.create_rate(1.0)

    while rclpy.ok():
        if tracker_name == "":
            continue

        try:
            transform_stamped = tf_buffer.lookup_transform('vive_world', tracker_name, node.get_clock().now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as lookup_transform_error:
            continue

        if tf_count < 10:
            transform_stamped = tf_buffer.lookup_transform('vive_world', tracker_name, node.get_clock().now())
            br.sendTransform(transform_stamped)
            inv_transform = transform_stamped.transform
            inv_transform.translation.x = -inv_transform.translation.x
            inv_transform.translation.y = -inv_transform.translation.y
            inv_transform.translation.z = -inv_transform.translation.z
            inv_transform.rotation.x = -inv_transform.rotation.x
            inv_transform.rotation.y = -inv_transform.rotation.y
            inv_transform.rotation.z = -inv_transform.rotation.z
            inv_transform.rotation.w = -inv_transform.rotation.w
            tf_count += 1

        rate.sleep()

    br.sendTransform(inv_transform, 'vive_world', 'vive_world_orientation')

    while rclpy.ok():
        br.sendTransform(transform_stamped)
        rclpy.spin_once(node)


if __name__ == '__main__':
    main()
