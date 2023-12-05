import time

import openvr

from vive_tracker_ros2.pose_sample_buffer import pose_sample_buffer
from vive_tracker_ros2.util_funcs import convert_to_euler, convert_to_quaternion


class VrTrackedDevice:
    def __init__(self,vr_obj,index,device_class):
        self.device_class = device_class
        self.index = index
        self.vr = vr_obj

    def get_serial(self):
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_SerialNumber_String).decode('utf-8')

    def get_model(self):
        return self.vr.getStringTrackedDeviceProperty(self.index,openvr.Prop_ModelNumber_String).decode('utf-8')

    def sample(self,num_samples,sample_rate):
        interval = 1/sample_rate
        rtn = pose_sample_buffer()
        sample_start = time.time()
        for i in range(num_samples):
            start = time.time()
            pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
            rtn.append(pose[self.index].mDeviceToAbsoluteTracking,time.time()-sample_start)
            sleep_time = interval- (time.time()-start)
            if sleep_time>0:
                time.sleep(sleep_time)
        return rtn

    def get_pose_euler(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        return convert_to_euler(pose[self.index].mDeviceToAbsoluteTracking)

    def get_pose_quaternion(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        return convert_to_quaternion(pose[self.index].mDeviceToAbsoluteTracking)

    def get_pose_matrix(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        return pose[self.index].mDeviceToAbsoluteTracking

    def get_velocities(self):
        pose = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        [v_x, v_y, v_z] = pose[self.index].vVelocity
        [a_x, a_y, a_z] = pose[self.index].vAngularVelocity
        return [v_x, v_y, v_z, a_x, a_y, a_z]
    def is_connected(self):
        tracking = self.vr.isTrackedDeviceConnected(self.index)
        return tracking
