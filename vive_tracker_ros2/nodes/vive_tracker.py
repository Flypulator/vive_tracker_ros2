import math
import warnings

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import tf2_ros
import vive_tracker_ros2.triad_openvr
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def vive_tracker():
    rclpy.init()

    node = rclpy.create_node('vive_tracker_frame')
    broadcaster = {}
    publisher = {}
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    rate = node.create_rate(30)  # 30hz

    try:
        v = vive_tracker_ros2.triad_openvr.TriadOpenVr()
    except Exception as ex:
        if (type(ex).__name__ == 'OpenVRError') and (
                ex.args[0] == 'VRInitError_Init_HmdNotFoundPresenceFailed (error number 126)'):
            print('Cannot find the tracker.')
            print('Is SteamVR running?')
            print('Is the Vive Tracker turned on, connected, and paired with SteamVR?')
            print('Are the Lighthouse Base Stations powered and in view of the Tracker?\n\n')
        else:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
        # print(ex.args)
        quit()

    v.print_discovered_objects()
    P = np.mat([[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
    p_cov = np.zeros((6, 6))
    # position covariance
    p_cov[0:2, 0:2] = P[0:2, 0:2]
    # orientation covariance for Yaw
    # x and Yaw
    p_cov[5, 0] = p_cov[0, 5] = P[2, 0]
    # y and Yaw
    p_cov[5, 1] = p_cov[1, 5] = P[2, 1]
    # Yaw and Yaw
    p_cov[5, 5] = P[2, 2]

    p_cov[0, :] = [0.0000349162103240595, -0.0000018202960310455, -0.0000339898160507969, -0.0000081126791170800,
                   0.0000001353045808767, 0.0000032202291901186]
    p_cov[1, :] = [-0.0000018202960310455, 0.0000011910722363973, 0.0000020423436706964, 0.0000010961526869235,
                   -0.0000000333091396801, -0.0000001408541892558]
    p_cov[2, :] = [-0.0000339898160507969, 0.0000020423436706964, 0.0000341312090595451, 0.0000060715616751347,
                   -0.0000000237628610568, -0.0000029217229365340]
    p_cov[3, :] = [-0.0000081126791170800, 0.0000010961526869235, 0.0000060715616751347, 0.0000165832615351042,
                   -0.0000004759697840205, -0.0000024486872043021]
    p_cov[4, :] = [0.0000001353045808767, -0.0000000333091396801, -0.0000000237628610568, -0.0000004759697840205,
                   0.0000003366392930324, -0.0000000030521109214]
    p_cov[5, :] = [0.0000032202291901186, -0.0000001408541892558, -0.0000029217229365340, -0.0000024486872043021,
                   -0.0000000030521109214, 0.0000007445433570531]

    while rclpy.ok():
        # if (deviceCount != v.get_device_count()):
        #    v = triad_openvr.triad_openvr()
        #    deviceCount = v.get_device_count()
        # For each Vive Device
        for deviceName in v.devices:
            current_time = node.get_clock().now()
            device = v.devices[deviceName]
            publish_name_str = device.get_serial().replace("-", "_")

            # skip null devices
            if 'Null' in device.get_serial():
                continue

            # Broadcast the transformation
            [x, y, z, qx, qy, qz, qw] = device.get_pose_quaternion()
            orientation = Rotation.from_quat([qx, qy, qz, qw])

            if deviceName not in broadcaster:
                broadcaster[deviceName] = tf2_ros.TransformBroadcaster(node)

            # Rotate Vive Trackers 180, so Z+ comes out on the top of the Tracker
            if "LHR" in device.get_serial():
                orientation = Rotation.from_euler("X", 180, degrees=True) * orientation

            tfs = TransformStamped()
            tfs.header.stamp = current_time.to_msg()
            tfs.header.frame_id = "vive_world"
            tfs._child_frame_id = publish_name_str
            tfs.transform.translation.x = x
            tfs.transform.translation.y = y
            tfs.transform.translation.z = z

            tfs.transform.rotation.x = orientation.as_quat()[0]
            tfs.transform.rotation.y = orientation.as_quat()[1]
            tfs.transform.rotation.z = orientation.as_quat()[2]
            tfs.transform.rotation.w = orientation.as_quat()[3]

            broadcaster[deviceName].sendTransform(tfs)

            # Publish a topic with euler angles as print out
            [x, y, z, roll, pitch, yaw] = device.get_pose_euler()

            if deviceName not in publisher:
                publisher[deviceName] = node.create_publisher(String, publish_name_str, 10)

            string_msg = String()
            string_msg.data = ('  X: ' + str(x) + '  Y: ' + str(y) + '  Z: ' + str(z) +
                               '  Pitch: ' + str(pitch) + '  Roll: ' + str(roll) + '  Yaw: ' + str(yaw))
            publisher[deviceName].publish(string_msg)

            #
            if "reference" not in deviceName:
                # create publisher
                if deviceName + "_odom" not in publisher:
                    publisher[deviceName + "_odom"] = node.create_publisher(Odometry, publish_name_str + "_odom", 50)
                    publisher[deviceName + "_pose"] = node.create_publisher(PoseWithCovarianceStamped,
                                                                            publish_name_str + "_pose", 10)

                # create odometry message
                odom = Odometry()
                odom.header.stamp = current_time.to_msg()
                odom.header.frame_id = "vive_world"
                # set the pose
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
                odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
                # set the velocity
                odom.child_frame_id = publish_name_str
                [vx, vy, vz, v_roll, v_pitch, v_yaw] = device.get_velocities()
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = vx, vy, vz
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = v_roll, v_pitch, v_yaw
                # set covariance
                # This is all wrong but close enough for now
                odom.pose.covariance = tuple(p_cov.ravel().tolist())
                odom.twist.covariance = tuple(p_cov.ravel().tolist())

                # Create a pose with covariance stamped topic
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = current_time.to_msg()
                pose.header.frame_id = "vive_world"
                # set the pose
                pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = x, y, z
                pose.pose.pose.orientation.x, pose.pose.pose.orientation.y = qx, qy
                pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = qz, qw
                # set covariance
                pose.pose.covariance = tuple(p_cov.ravel().tolist())

                # publish all messages
                if np.any([x, y, z, vx, vy, vz, v_roll, v_pitch, v_yaw]):  # only publish if at least one value != 0
                    # publish the message
                    try:
                        publisher[deviceName + "_odom"].publish(odom)
                        publisher[deviceName + "_pose"].publish(pose)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        warnings.warn("Publishing odometry and pose for device " + deviceName + " failed.")
                        continue

        # rate.sleep()


if __name__ == '__main__':
    vive_tracker()
