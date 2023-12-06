import time

import numpy as np
import openvr
import yaml
from scipy.spatial.transform import Rotation


class VrTrackedDevice:
    def __init__(self, vr_obj, index, device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj

        # load config file
        with open('vive_config.yaml', 'r') as file:
            self.vive_config = yaml.safe_load(file)

    @property
    def serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)

    @property
    def alias(self):
        if self.serial in self.vive_config['device_alias']:
            return self.vive_config['device_alias'][self.serial]
        else:
            return self.serial.replace("-", "_")

    @property
    def model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModelNumber_String)

    def _get_raw_pose_matrix(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                       openvr.k_unMaxTrackedDeviceCount)
        pose_matrix = np.asarray(
            [[pose[self.index].mDeviceToAbsoluteTracking[i][j] for i in range(3)] for j in range(4)]).T
        return pose_matrix

    def get_position(self, frame="vive_world"):
        """get position of device with offset from vive_config.yaml applied"""
        pose_matrix = self._get_raw_pose_matrix()
        [x, y, z] = pose_matrix[:, 3].T
        # apply position offset from config file
        if self.alias in self.vive_config['frames_pose_offset']:
            [offset_pos, _] = self.vive_config['frames_pose_offset'][self.alias]
            orientation = self.get_orientation(frame=frame)
            offset_pos = orientation.apply(offset_pos)
            [x, y, z] = [x+offset_pos[0], y+offset_pos[1], z+offset_pos[2]]
        return [x, y, z]

    def get_orientation(self, frame="vive_world"):
        """get orientation of device with offset from vive_config.yaml applied"""
        pose_matrix = self._get_raw_pose_matrix()
        orientation = Rotation.from_matrix(pose_matrix[:3, :3])
        # apply orientation offset from config file
        if self.alias in self.vive_config['frames_pose_offset']:
            [_, offset_quat] = self.vive_config['frames_pose_offset'][self.alias]
            orientation = orientation * Rotation.from_quat(offset_quat)
        return orientation

    def get_twist(self, frame="vive_world"):
        """get twist of device"""
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                       openvr.k_unMaxTrackedDeviceCount)
        [v_x, v_y, v_z] = pose[self.index].vVelocity
        [omega_x, omega_y, omega_z] = pose[self.index].vAngularVelocity
        return [v_x, v_y, v_z, omega_x, omega_y, omega_z]

    def is_connected(self):
        tracking = self.vr.isTrackedDeviceConnected(self.index)
        return tracking


class VrTrackingReference(VrTrackedDevice):
    def get_mode(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModeLabel_String).upper()
