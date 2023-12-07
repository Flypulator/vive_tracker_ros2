import openvr
import yaml
from scipy.spatial.transform import Rotation

from vive_tracker_ros2.frame import Frame, WorldFrame
from vive_tracker_ros2.vr_devices import VrTrackedDevice, VrTrackingReference


class TriadOpenVr:
    def __init__(self):
        with open('vive_config.yaml', 'r') as file:
            self.vive_config = yaml.safe_load(file)
        [world_offset_pos, world_offset_quat] = self.vive_config['world_frame_pose_offset']
        world_offset_ori = Rotation.from_quat(world_offset_quat)
        vive_world_offset_pos = -world_offset_ori.apply(world_offset_pos, inverse=True)
        vive_world_offset_ori = world_offset_ori.inv()
        self.vive_world_frame = Frame("vive_world", WorldFrame(), vive_world_offset_pos, vive_world_offset_ori)

        print("Initializing OpenVR ...")
        self.vr = openvr.init(openvr.VRApplication_Other)

        # Initializing object to hold indexes for various tracked objects
        self.object_names = {"Tracking Reference": [], "HMD": [], "Controller": [], "Tracker": []}
        self.devices = {}
        poses = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                        openvr.k_unMaxTrackedDeviceCount)

        # Iterate through the pose list to find the active devices and determine their type
        print("Searching for active devices ...")
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if poses[i].bPoseIsValid:
                device_class = self.vr.getTrackedDeviceClass(i)
                if device_class == openvr.TrackedDeviceClass_Controller:
                    device_name = "controller_" + str(len(self.object_names["Controller"]) + 1)
                    self.object_names["Controller"].append(device_name)
                    self.devices[device_name] = VrTrackedDevice(self.vr, i, "Controller", self.vive_world_frame)
                elif device_class == openvr.TrackedDeviceClass_HMD:
                    device_name = "hmd_" + str(len(self.object_names["HMD"]) + 1)
                    self.object_names["HMD"].append(device_name)
                    self.devices[device_name] = VrTrackedDevice(self.vr, i, "HMD", self.vive_world_frame)
                elif device_class == openvr.TrackedDeviceClass_GenericTracker:
                    device_name = "tracker_" + str(len(self.object_names["Tracker"]) + 1)
                    self.object_names["Tracker"].append(device_name)
                    self.devices[device_name] = VrTrackedDevice(self.vr, i, "Tracker", self.vive_world_frame)
                elif device_class == openvr.TrackedDeviceClass_TrackingReference:
                    device_name = "tracking_reference_" + str(len(self.object_names["Tracking Reference"]) + 1)
                    self.object_names["Tracking Reference"].append(device_name)
                    self.devices[device_name] = VrTrackingReference(self.vr, i, "Tracking Reference", self.vive_world_frame)

    def rename_device(self, old_device_name, new_device_name):
        self.devices[new_device_name] = self.devices.pop(old_device_name)
        for i in range(len(self.object_names[self.devices[new_device_name].device_class])):
            if self.object_names[self.devices[new_device_name].device_class][i] == old_device_name:
                self.object_names[self.devices[new_device_name].device_class][i] = new_device_name

    def get_device_count(self):
        return len(self.devices)

    def print_discovered_objects(self):
        for device_type in self.object_names:
            plural = device_type
            if len(self.object_names[device_type]) != 1:
                plural += "s"
            print("Found " + str(len(self.object_names[device_type])) + " " + plural)
            for device in self.object_names[device_type]:
                device_serial = self.devices[device].serial
                if device_type == "Tracking Reference":
                    device_print_line = "  " + device + " (" + device_serial + ", Mode " + self.devices[
                        device].get_mode() + ", " + self.devices[device].model + ")"
                else:
                    device_print_line = "  " + device + " (" + device_serial + ", " + self.devices[
                        device].model + ")"
                if device_serial in self.vive_config['device_alias']:
                    device_print_line += ", alias: " + self.vive_config['device_alias'][device_serial]
                print(device_print_line)
