import warnings

import numpy as np
import yaml

import rclpy
import tf2_ros
import vive_tracker_ros2.triad_openvr
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class ViveTracker(Node):
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

    def __init__(self):
        super().__init__('vive_tracker_frame')
        with open('vive_config.yaml', 'r') as file:
            self.vive_config = yaml.safe_load(file)

        timer_period = 1 / self.vive_config['rate']  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.broadcaster = {}
        self.publisher = {}

        try:
            self.v = vive_tracker_ros2.triad_openvr.TriadOpenVr()
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

        self.v.print_discovered_objects()

        print("ViveTracker setup complete!")

    def timer_callback(self):
        for device_name in self.v.devices:
            current_time = self.get_clock().now()
            device = self.v.devices[device_name]

            # skip null devices
            if 'Null' in device.serial:
                continue

            # get pose
            [x, y, z] = device.get_position()
            orientation = device.get_orientation()
            [qx, qy, qz, qw] = orientation.as_quat()

            # Broadcast the transformation
            if device_name not in self.broadcaster:
                self.broadcaster[device_name] = tf2_ros.TransformBroadcaster(self)

            tfs = TransformStamped()
            tfs.header.stamp = current_time.to_msg()
            tfs.header.frame_id = "vive_world"  # TODO: change to "world"
            tfs._child_frame_id = device.alias
            tfs.transform.translation.x = x
            tfs.transform.translation.y = y
            tfs.transform.translation.z = z
            tfs.transform.rotation.x = qx
            tfs.transform.rotation.y = qy
            tfs.transform.rotation.z = qz
            tfs.transform.rotation.w = qw

            self.broadcaster[device_name].sendTransform(tfs)

            # Publish a topic with euler angles as print out
            [yaw, pitch, roll] = orientation.as_euler("zyx")

            if device_name not in self.publisher:
                self.publisher[device_name] = self.create_publisher(String, device.alias, qos_profile_sensor_data)

            string_msg = String()
            string_msg.data = ('  X: ' + str(x) + '  Y: ' + str(y) + '  Z: ' + str(z) +
                               '  Yaw: ' + str(yaw) + '  Pitch: ' + str(pitch) + '  Roll: ' + str(roll))
            self.publisher[device_name].publish(string_msg)

            # publish odometry and pose for all devices but the lighthouses
            if "reference" not in device_name:
                # create publisher
                if device_name + "_odom" not in self.publisher:
                    self.publisher[device_name + "_odom"] = self.create_publisher(Odometry, device.alias + "_odom",
                                                                                  qos_profile_sensor_data)
                    self.publisher[device_name + "_pose"] = self.create_publisher(PoseWithCovarianceStamped,
                                                                                  device.alias + "_pose",
                                                                                  qos_profile_sensor_data)
                # get twist
                [vx, vy, vz, omega_x, omega_y, omega_z] = device.get_twist()

                # create odometry message
                odom = Odometry()
                odom.header.stamp = current_time.to_msg()
                odom.header.frame_id = "vive_world"
                # set the pose
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                # set the velocity
                odom.child_frame_id = device.alias
                odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = vx, vy, vz
                odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = omega_x, omega_y, omega_z
                # set covariance
                # This is all wrong but close enough for now
                odom.pose.covariance = tuple(self.p_cov.ravel().tolist())
                odom.twist.covariance = tuple(self.p_cov.ravel().tolist())

                # Create a pose with covariance stamped topic
                pose = PoseWithCovarianceStamped()
                pose.header.stamp = current_time.to_msg()
                pose.header.frame_id = "vive_world"
                # set the pose
                pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = x, y, z
                pose.pose.pose.orientation.x = qx
                pose.pose.pose.orientation.y = qy
                pose.pose.pose.orientation.z = qz
                pose.pose.pose.orientation.w = qw
                # set covariance
                pose.pose.covariance = tuple(self.p_cov.ravel().tolist())

                # publish all messages
                if np.any([x, y, z, vx, vy, vz, omega_x, omega_y, omega_z]):  # only publish if at least one value != 0
                    # publish the message
                    try:
                        self.publisher[device_name + "_odom"].publish(odom)
                        self.publisher[device_name + "_pose"].publish(pose)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        warnings.warn("Publishing odometry and pose for device " + device.alias + " failed.")
                        continue


def main(args=None):
    rclpy.init(args=args)

    vive_tracker = ViveTracker()

    try:
        print("Starting to spin ViveTracker node")
        rclpy.spin(vive_tracker)
    except KeyboardInterrupt:
        pass

    vive_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
