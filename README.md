This ROS2 Node publishes pose data from HTC Vive Tracker on Ubuntu 22.04. 

### Note: This software depends on SteamVR on Ubuntu. Be advised that SteamVR is not fully supported on Ubuntu and may not install properly with your version of Ubuntu/graphics drivers/hardware. I recommend you try to get SteamVR installed on your computer before considering this node for your project. 

# Prerequisites

Up-to-date graphics drivers

SteamVR requires >4GB disk space

Have python3 and pip installed on your system


# Installation Instructions

1. Install Steam from http://store.steampowered.com/ or via apt:
      1. `sudo apt install steam`
      2. Open steam with `steam` command, or through the Ubuntu menu. Make a Steam account & Log in.
      3. Enable the Steam beta through the Steam Menu -> Settings -> Account -> Beta Participation. [See the video here to see how to enable the beta.](https://www.youtube.com/watch?v=7AFUcj3HpvE)
      4. (Recommended) Save your credentials while logging in, and once you do log in open the `Steam` Menu item in the top left corner and select `Go Offline`. This prevents Steam from updating every time you use the Vive Tracker. 

2. Install SteamVR. 
   1. Click Library
   2. Click VR
   3. On the left you should now see SteamVR. Add to library.
   4. Before installing, left click on SteamVR and click on `Properties`
   5. In Tab `Betas` under `Beta Participation` select `linux_v1.14`

3. Make a Symbolic Link from libudev.so.0 to libudev.so.1 for SteamVR to use. 

`sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1 /lib/x86_64-linux-gnu/libudev.so.0`

4. Install pyopenvr

`sudo pip install -U pip openvr`

5. Disable the headset requirement and enable a null (simulated) headset:

`gedit ~/.steam/steam/steamapps/common/SteamVR/resources/settings/default.vrsettings`

   1. Change the third line from `"requireHmd" : true,` to `"requireHmd" : false,`

   2. Add `"activateMultipleDrivers"` : true, and add the line `"forcedDriver": "null"` beneath it.
   
   3. Open `default.vrsettings`

`gedit ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings`

   1. Set `enable` (line 3) to `true` in null driver to enable it.

  [Source](https://www.reddit.com/r/Vive/comments/6uo053/how_to_use_steamvr_tracked_devices_without_a_hmd/) 
  
6. Install this project in your colcon workspace.

```
cd ~/colcon_ws/src/
git clone https://github.com/moon-wreckers/vive_tracker.git
cd ~/colcon_ws
colcon build --select-packages vive_tracker_ros2
```

7. 

# Usage
1. Start SteamVR from the Steam Library (If you encounter `VRClientDLLNotFound`, make sure all of the dependencies are installed properly, especially VulkanSDK, and delete and recreate the symbolic link described above).

2. Turn on the tracker with its button, and make sure that its wireless USB dongle is plugged in to your computer. If the tracker shows up in the SteamVR overlay skip to step 4.

3. Sync the tracker. Hold the button on the tracker until the light blinks. On the SteamVR overlay click the "SteamVR" dropdown menu. Click Devices->Pair Controller. The Tracker should then pair with the computer, and a green outline of the tracker should appear on the SteamVR overlay. If this doesn't work try unplugging the wireless USB dongle, plugging it back in, and restarting SteamVR. Restarting your computer wouldn't hurt either.

4. Ensure the Lighthouse base stations are turned on, facing each other, have green lights showing on them Place the tracker in view of the Base Stations. The SteamVR overlay should now show two green square Base Stations and a solid green Tracker hexagon. The tracker is now working. 

     1. If you're only using 1 Base Station, make sure it's set to mode A.
     
     2. If you're using 2 Base Stations without a sync cable, ensure they're set to modes B and C.
     
     3. If you're using 2 Base Stations with a sync cable, ensure they're set to modes A and B.

5. Run this ROS2 node. 

```
source ~/ros2_ws/install/setup.bash
ros2 run vive_tracker_ros2
``` 
6. (Optional) Start RViz in another terminal with `rviz2`

7. (Optional) In the lower left corner of RViz click on `Add`, and scroll down the Add menu to add a `TF`. If all went well you should now be able to see the tracker moving in RViz. 

8. If for some reason it isn't working, check to ensure that the Tracker is turned on, SteamVR is still running, the tracker icon is green, and the vive_tracker ros2 node is still running.


# Command Line

Here's a handy command, run this in the command line to start SteamVR with the command `steamvr`

`alias steamvr='LD_LIBRARY_PATH=~/.steam/bin32/ ~/.steam/bin32/steam-runtime/run.sh ~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh' >> ~/.bashrc && source ~/.bashrc`

This will start the server in another process, so you're free to keyboard interrupt (Ctrl+C) the terminal once the server starts. 

To kill the SteamVR process:

`sudo pkill -9 vr*`




# Links

This project is based on https://github.com/moon-wreckers/vive_tracker.git, which uses ROS1. Many thanks to Daniel Arnett et.al.!
