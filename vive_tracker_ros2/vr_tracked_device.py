import time

import numpy as np
import openvr
from scipy.spatial.transform import Rotation

from vive_tracker_ros2.pose_sample_buffer import PoseSampleBuffer


class VrTrackedDevice:
    def __init__(self, vr_obj, index, device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj

    def get_serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)

    def get_model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String)

    def sample(self, num_samples, sample_rate):
        interval = 1 / sample_rate
        rtn = PoseSampleBuffer()
        sample_start = time.time()
        for i in range(num_samples):
            start = time.time()
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                           openvr.k_unMaxTrackedDeviceCount)
            rtn.append(pose[self.index].mDeviceToAbsoluteTracking, time.time() - sample_start)
            sleep_time = interval - (time.time() - start)
            if sleep_time > 0:
                time.sleep(sleep_time)
        return rtn

    def get_pose_matrix(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                       openvr.k_unMaxTrackedDeviceCount)
        pose_matrix = np.asarray(
            [[pose[self.index].mDeviceToAbsoluteTracking[i][j] for i in range(3)] for j in range(4)]).T
        return pose_matrix

    def get_position(self):
        pose_matrix = self.get_pose_matrix()
        [x, y, z] = pose_matrix[:, 3].T
        return [x, y, z]

    def get_orientation(self, flip_z_axis=False):
        pose_matrix = self.get_pose_matrix()
        orientation = Rotation.from_matrix(pose_matrix[:3, :3])
        if flip_z_axis:
            orientation = Rotation.from_euler("X", 180, degrees=True) * orientation
        return orientation

    def get_twist(self, flip_z_axis=False):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                       openvr.k_unMaxTrackedDeviceCount)
        [v_x, v_y, v_z] = pose[self.index].vVelocity
        [omega_x, omega_y, omega_z] = pose[self.index].vAngularVelocity
        if flip_z_axis:
            [omega_x, omega_y, omega_z] = [omega_x, -omega_y, -omega_z]
        return [v_x, v_y, v_z, omega_x, omega_y, omega_z]

    def is_connected(self):
        tracking = self.vr.isTrackedDeviceConnected(self.index)
        return tracking
