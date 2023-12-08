import numpy as np
import openvr
from scipy.spatial.transform import Rotation

from src import config_file_util
from src.frame import WorldFrame, Frame


class VrTrackedDevice:
    def __init__(self, vr_obj, index, device_class, vive_world_frame):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj
        self.vive_world_frame = vive_world_frame

        # load config file
        self.vive_config = config_file_util.get_config()

    @property
    def serial(self):
        """read out device serial from API"""
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_SerialNumber_String)

    @property
    def alias(self):
        """assign an alias for device, uses config file or device serial"""
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

        Args:
            reference_frame: frame in which device frame will be evaluated
        """
        # get device pose in vive_world frame
        pose_matrix_in_vive_world_frame = self._get_raw_pose_matrix()
        pos_offset_in_vive_world_frame = pose_matrix_in_vive_world_frame[:, 3].T
        rotation_in_vive_world_frame = Rotation.from_matrix(pose_matrix_in_vive_world_frame[:3, :3])

        # create device frame relative to vive_world
        device_frame_in_vive_world_frame = Frame(
            frame_name=self.alias,
            reference_frame=self.vive_world_frame,
            pos_offset=pos_offset_in_vive_world_frame,
            rotation=rotation_in_vive_world_frame
        )

        # apply offset from config file
        if self.alias in self.vive_config['frames_pose_offset']:
            [offset_pos, offset_quat] = self.vive_config['frames_pose_offset'][self.alias]
            device_frame_with_offset = Frame(
                frame_name=f"{self.alias}_with_offset",
                reference_frame=device_frame_in_vive_world_frame,
                pos_offset=offset_pos,
                rotation=Rotation.from_quat(offset_quat)
            )
        else:
            device_frame_with_offset = device_frame_in_vive_world_frame

        # return device frame relative to the reference frame specified as func arg
        device_frame_relative_to_ref_frame = reference_frame.relative_frame(device_frame_with_offset)
        return device_frame_relative_to_ref_frame

    def get_position(self, reference_frame=WorldFrame()):
        """
        get position of device in specified reference frame, with offset from vive_config.yaml applied

        Args:
            reference_frame: frame in which device frame will be evaluated
        """
        device_frame = self.get_device_frame(reference_frame)
        return device_frame.pos_offset

    def get_orientation(self, reference_frame=WorldFrame()):
        """
        get orientation of device in specified reference frame, with offset from vive_config.yaml applied

        Args:
            reference_frame: frame in which device frame will be evaluated
        """
        device_frame = self.get_device_frame(reference_frame)
        return device_frame.rotation

    def get_velocity(self, reference_frame=WorldFrame()):
        """
        get velocity of device

        Args:
            reference_frame: frame in which velocity will be evaluated
        """
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                       openvr.k_unMaxTrackedDeviceCount)
        vel_in_vive_world_frame = np.array([velElem for velElem in pose[self.index].vVelocity])

        vel_in_ref_frame = reference_frame.vel_in_frame_coordinates(
            vel_in_vive_world_frame, self.vive_world_frame)

        return vel_in_ref_frame

    def get_angular_velocity(self, reference_frame=None):
        """
        get angular velocity of device. Default reference frame is moving body frame of device.

        Args:
            reference_frame: frame in which angular velocity will be evaluated. Default is moving body frame of device
        """
        if reference_frame is None:
            reference_frame = self.get_device_frame(WorldFrame())

        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                       openvr.k_unMaxTrackedDeviceCount)
        ang_vel_in_vive_world_frame = np.array([velElem for velElem in pose[self.index].vAngularVelocity])

        ang_vel_in_ref_frame = reference_frame.ang_vel_in_frame_coordinates(
            ang_vel_in_vive_world_frame, self.vive_world_frame)

        return ang_vel_in_ref_frame

    def is_connected(self):
        """check connectivity of device via OpenVR API"""
        tracking = self.vr.isTrackedDeviceConnected(self.index)
        return tracking


class VrTrackingReference(VrTrackedDevice):
    def get_mode(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModeLabel_String).upper()
