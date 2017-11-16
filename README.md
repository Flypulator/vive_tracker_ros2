This ROS Indigo Node publishes pose data from HTC Vive Tracker Ubuntu 14.04.


# Prerequesites

Up to date graphics drivers

x86 architecture

SteamVR requires >4GB disk space

# Installation Instructions
1. Install Steam, as well as Python and OpenVR dependencies

`sudo apt-get install steam libsdl2-dev libvulkan-dev libudev-dev libssl-dev zlib1g-dev python-pip`
   
   1. Make Steam account & Log in.

2. Install SteamVR 

   1. Click Library

   1. Click VR

   1. On the left you should now see SteamVR. Select it and click the the blue Install button.

1. Make a Symbolic Link from libudev.so.0 to libudev.so.1 for SteamVR to use

`sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1 /lib/x86_64-linux-gnu/libudev.so.0`

5. Install pyopenvr

`sudo pip install -U pip openvr`

7. (Optional) Disable the headset requirement with your preferred text editor

`gedit ~/.steam/steam/steamapps/common/SteamVR/resources/settings/default.vrsettings`

   1. Change the third line from `"requireHmd" : true,` to `"requireHmd" : false,`

   1. Save and exit the settings document.
   
8. Install this project in your catkin workspace.

```
cd ~/catkin_ws/src/
git clone https://github.com/moon-wreckers/vive_tracker.git
cd ~/catkin_ws
catkin_make
```

9. 

# Usage
1. Start SteamVR from the Steam Library

2. Turn on the tracker with its button, and make sure that its wireless USB dongle is plugged in to your computer. If the tracker shows up in the SteamVR overlay skip to step 4.

3. Sync the tracker. Hold the button on the tracker until the light blinks. On the SteamVR overlay click the "SteamVR" dropdown menu. Click Devices->Pair Controller. The Tracker should then pair with the computer, and a green outline of the tracker should appear on the SteamVR overlay. If this doesn't work try unplugging the wireless USB dongle, plugging it back in, and restarting SteamVR. 

4. Ensure the Lighthouse base stations are turned on, facing each other, and have green lights showing on them. Place the tracker in view of the Base Stations. The SteamVR overlay should now show two green square Base Stations and a solid green Tracker hexagon. The tracker is now working.

5. In one terminal start ros with `roscore`, in another terminal run this ROS node. 

```
source ~/catkin_ws/devel/setup.bash
roslaunch vive_tracker vive_tracker.launch
``` 
   1. Now open another terminal and run `rostopic echo /vive_tracker` to view the x y z roll pitch yaw output from the tracker.

6. (Optional) Start RViz in another terminal with `rviz`

7. (Optional) In the lower left corner of RViz click on `Add`, and scroll down the Add menu to add a `TF`. If all went well you should now be able to see the tracker moving in RViz. 

8. If for some reason it isn't working, check to ensure that the Tracker is turned on, SteamVR is still running, the tracker icon is green, and the vive_tracker ros node is still running.


# Command Line

To start SteamVR from the commandline you can run (as one command):

`LD_LIBRARY_PATH=~/.steam/bin32/ ~/.steam/bin32/steam-runtime/run.sh ~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh`

This will start the server in another process, so you're free to keyboard interrupt (Ctrl+C) the terminal once the server starts. 

To kill the SteamVR process:

`sudo killall -9 vrmonitor`




# Links

This is based off of Triad Semiconductor's awesome tutorial found here:

http://help.triadsemi.com/steamvr-tracking/steamvr-tracking-without-an-hmd/

Also thanks to Christopher Bruns for his work on pyopenvr.

https://github.com/cmbruns/pyopenvr
