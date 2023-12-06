import fileinput
import re

import vive_tracker_ros2.triad_openvr


def set_world_origin(device_name):
    """
    script to set the world origin to the current position of the tracker

    Args:
        device_name: name, serial or alias of tracker to use as reference for world origin
    """
    v = vive_tracker_ros2.triad_openvr.TriadOpenVr()
    v.print_discovered_objects()

    if device_name in v.devices:
        device = v.devices[device_name]
    elif device_name in [v.devices[it_device].alias for it_device in v.devices]:
        device = [v.devices[it_device] for it_device in v.devices if v.devices[it_device].alias == device_name][0]
    elif device_name in [v.devices[it_device].serial for it_device in v.devices]:
        device = [v.devices[it_device] for it_device in v.devices if v.devices[it_device].serial == device_name][0]

    ref_position = device.get_position()
    ref_orientation = device.get_orientation()

    world_offset_pos = (-ref_orientation.apply(ref_position, inverse=True)).tolist()
    world_offset_orientation = ref_orientation.inv()

    search_exp = "world_frame_pose_offset:.*"
    replace_exp = f"world_frame_pose_offset: [{world_offset_pos},{world_offset_orientation.as_quat().tolist()}]"
    for line in fileinput.input('vive_config.yaml', inplace=True):
        searched = re.search(search_exp, line)
        newline = (replace_exp if searched is not None else line).rstrip()
        print(newline)
    pass

    print(f"World origin successfully moved to device {device_name}. New world offset:")
    print(f"xyz: {world_offset_pos}")
    print(f"quat xyzw: {world_offset_orientation.as_quat()}")


if __name__ == '__main__':
    set_world_origin("drone_tracker")
