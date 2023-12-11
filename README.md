This ROS2 Node publishes pose data from HTC Vive Tracker on Ubuntu 22.04. 

### Note: This software depends on SteamVR on Ubuntu. Be advised that SteamVR is not fully supported on Ubuntu and may not install properly with your version of Ubuntu/graphics drivers/hardware. I recommend you try to get SteamVR installed on your computer before considering this node for your project. 

# Prerequisites

Up-to-date graphics drivers

SteamVR requires >4GB disk space

Have python3 and ros2 installed on your system


# Installation Instructions

1. Install Steam from http://store.steampowered.com/ or via apt:
      1. `sudo apt install steam`
      2. Open steam with `steam` command, or through the Ubuntu menu. Make a Steam account & Log in.
      3. Enable the Steam beta through the Steam Menu -> Settings -> Account -> Beta Participation. [See the video here to see how to enable the beta.](https://www.youtube.com/watch?v=7AFUcj3HpvE)
      4. (Recommended) Save your credentials while logging in, and once you do log in open the `Steam` Menu item in the top left corner and select `Go Offline`. This prevents Steam from updating every time you use the Vive Tracker. 

2. Install SteamVR. 
   1. Click Library -> VR
   2. On the left you should now see SteamVR. Add to library.
   3. Before installing, left click on SteamVR and click on `Properties`
   4. In Tab `Betas` under `Beta Participation` select `linux_v1.14`

3. Install udev rules to be able to use USB dongles

   1. Download and follow instructions at https://gitlab.com/fabiscafe/game-devices-udev
   2. Download https://github.com/ValveSoftware/steam-devices and do the same with these files

4. Make a Symbolic Link from libudev.so.0 to libudev.so.1 for SteamVR to use. 

   `sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1 /lib/x86_64-linux-gnu/libudev.so.0`

5. Disable the headset requirement and enable a null (simulated) headset:

   `gedit ~/.steam/steam/steamapps/common/SteamVR/resources/settings/default.vrsettings`

   1. Change the third line from `"requireHmd" : true,` to `"requireHmd" : false,`

   2. Add `"activateMultipleDrivers"` : true, and add the line `"forcedDriver": "null"` beneath it.
   
   3. Open `default.vrsettings`

   `gedit ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings`

   1. Set `enable` (line 3) to `true` in null driver to enable it.

   [Source](https://www.reddit.com/r/Vive/comments/6uo053/how_to_use_steamvr_tracked_devices_without_a_hmd/) 

6. Download this project to your computer in a directory of your choice (in the following `~/ros2_steamvr` is used).
   ```
   cd ~/ros2_steamvr
   git clone https://github.com/moon-wreckers/vive_tracker.git
   ```

7. Create python venv and install dependencies
   1. install environment manager *poetry* with *pipx* (if *pipx* does not work you can also use pip, but installing a python package with normal pip and without a virtual enviroment will change your system python which can be dangerous)
      ```
      sudo apt install pipx
      pipx install poetry
      ```
   2. create the poetry environment with all necessary packages
      ```
      cd ~/ros2_steamvr/vive_tracker_ros2
      poetry install
      ```

8. Set configuration of vive_tracker_ros2 Python Node
   1. open `~/ros2_steamvr/vive_tracker_ros2/vive_config.yaml`
   2. set property `ros2_packages_path` to the python package location of your ROS2 installation

9. (Optional) set bash alias
   1. open ~/.bashrc
   2. add the following line and save:
      ```
      alias run_vive_tracker_ros2="cd ~/ros2_steamvr/vive_tracker_ros2 && poetry run python run.py"
      ```

# Usage
1. Start SteamVR from the Steam Library (If you encounter `VRClientDLLNotFound`, make sure all the dependencies are installed properly, especially VulkanSDK, and delete and recreate the symbolic link described above).

2. Turn on the tracker with its button, and make sure that its wireless USB dongle is plugged in to your computer. If the tracker shows up in the SteamVR overlay skip to step 4.

3. Sync the tracker. Hold the button on the tracker until the light blinks. On the SteamVR overlay click the "SteamVR" dropdown menu. Click Devices->Pair Controller. The Tracker should then pair with the computer, and a green outline of the tracker should appear on the SteamVR overlay. If this doesn't work try unplugging the wireless USB dongle, plugging it back in, and restarting SteamVR. Restarting your computer wouldn't hurt either.

4. Ensure the Lighthouse base stations are turned on, facing each other, have green lights showing on them Place the tracker in view of the Base Stations. The SteamVR overlay should now show two green square Base Stations and a solid green Tracker hexagon. The tracker is now working. 

     1. If you're only using 1 Base Station, make sure it's set to mode A.
     
     2. If you're using 2 Base Stations without a sync cable, ensure they're set to modes B and C.
     
     3. If you're using 2 Base Stations with a sync cable, ensure they're set to modes A and B.

5. Run tracking ROS2 node.

   With bash alias:
   ```
   run_vive_tracker_ros2
   ```
   or directly:
   ```
   cd ~/ros2_steamvr/vive_tracker_ros2
   poetry run python run.py
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
