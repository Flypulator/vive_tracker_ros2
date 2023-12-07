import numpy as np
import openvr
import yaml
from scipy.spatial.transform import Rotation

from vive_tracker_ros2.frame import WorldFrame, Frame


class VrTrackedDevice:
    def __init__(self, vr_obj, index, device_class, vive_world_frame):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj
        self.vive_world_frame = vive_world_frame

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

    def get_device_frame(self, reference_frame=WorldFrame()):
        """
        get full pose of device as frame object, relative to specified reference frame,
        with offset from vive_config.yaml applied
        """
        # get device pose in vive_world frame
        pose_matrix = self._get_raw_pose_matrix()
        pos_offset = pose_matrix[:, 3].T
        rotation = Rotation.from_matrix(pose_matrix[:3, :3])

        # create device frame relative to vive_world
        device_frame_in_vive_world_frame = Frame(
            self.alias,
            self.vive_world_frame,
            pos_offset,
            rotation
        )
        # apply offset from config file
        if self.alias in self.vive_config['frames_pose_offset']:
            [offset_pos, offset_quat] = self.vive_config['frames_pose_offset'][self.alias]
            device_frame_in_vive_world_frame = Frame(
                f"{self.alias}_with_offset",
                device_frame_in_vive_world_frame,
                offset_pos, Rotation.from_quat(offset_quat)
            )

        # return device frame relative to the reference frame specified as func arg
        device_frame_relative_to_specified = reference_frame.relative_frame(device_frame_in_vive_world_frame)
        return device_frame_relative_to_specified

    def get_position(self, reference_frame=WorldFrame()):
        """get position of device in specified reference frame, with offset from vive_config.yaml applied"""
        device_frame = self.get_device_frame(reference_frame)
        return device_frame.pos_offset

    def get_orientation(self, reference_frame=WorldFrame()):
        """get orientation of device in specified reference frame, with offset from vive_config.yaml applied"""
        device_frame = self.get_device_frame(reference_frame)
        return device_frame.rotation

    def get_twist(self, reference_frame=WorldFrame()):
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
