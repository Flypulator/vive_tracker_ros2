import fileinput
import re

import src.triad_openvr
from src.config_file_util import get_config_file_path


def set_world_origin(device_name):
    """
    script to set the world origin to the current position of the tracker

    Args:
        device_name: name, serial or alias of tracker to use as reference for world origin
    """
    v = src.triad_openvr.TriadOpenVr()
    v.print_discovered_objects()

    if device_name in v.devices:
        device = v.devices[device_name]
    elif device_name in [v.devices[it_device].alias for it_device in v.devices]:
        device = [v.devices[it_device] for it_device in v.devices if v.devices[it_device].alias == device_name][0]
    elif device_name in [v.devices[it_device].serial for it_device in v.devices]:
        device = [v.devices[it_device] for it_device in v.devices if v.devices[it_device].serial == device_name][0]
    else:
        raise RuntimeError("Device not found!")

    ref_position = device.get_position(reference_frame=v.vive_world_frame)
    ref_orientation = device.get_orientation(reference_frame=v.vive_world_frame)

    world_offset_pos = (-ref_orientation.apply(ref_position, inverse=True)).tolist()
    world_offset_orientation = ref_orientation.inv()

    search_exp = "world_frame_pose_offset:.*"
    replace_exp = f"world_frame_pose_offset: [{world_offset_pos},{world_offset_orientation.as_quat().tolist()}]"
    config_path = get_config_file_path()
    for line in fileinput.input(config_path, inplace=True):
        searched = re.search(search_exp, line)
        newline = (replace_exp if searched is not None else line).rstrip()
        print(newline)
    pass

    print(f"World origin successfully moved to device {device_name}. New world offset:")
    print(f"xyz: {world_offset_pos}")
    print(f"quat xyzw: {world_offset_orientation.as_quat()}")


if __name__ == '__main__':
    # set name of the tracker that will be used as a reference pose for the world frame
    tracker_name = "calib_tracker"
    set_world_origin(tracker_name)
