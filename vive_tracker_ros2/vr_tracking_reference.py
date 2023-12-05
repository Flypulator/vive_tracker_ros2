import openvr

from vive_tracker_ros2.vr_tracked_device import VrTrackedDevice


class VrTrackingReference(VrTrackedDevice):
    def get_mode(self):
        return self.vr.getStringTrackedDeviceProperty(self.index, openvr.Prop_ModeLabel_String).decode('utf-8').upper()

    def sample(self, num_samples, sample_rate):
        print("Warning: Tracking References do not move, sample isn't much use...")
