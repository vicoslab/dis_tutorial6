# Tutorial 6: The Real Turtlebot 4

#### Development of Intelligent Systems, 2025

![robots](figs/robots.png)
*The Company of iRobot*

### Robot Status

The Create3 base comes with a built-in LED ring around the main power button, which indicates the robot state. The other two buttons around it are unassigned by default and can be used for custom actions.

| State Description | Robot LED State |
|------|-------|
| The Create3 is powered off, the Pi 4 power supply has been cut. In this state the robot can only be activated by placing it back on the dock. | ![off](figs/off.png) |
| The battery is charging, the robot is on the dock. | ![charging](figs/charging.png) |
| Normal operating state | ![normal](figs/normal.png) |
| Emergency stop (ESTOP) has been triggered, either due to hitting a bumper/IR/cliff switch or because there's no communication with the Pi 4. It can be reset by pressing the power button once, or through ROS | ![estop](figs/estop.png) |
| Battery is low, if you don't recharge the robot soon it will automatically power off. Place the robot back on the dock and power off the Pi 4 so it can recharge faster. | ![lowbattery](figs/lowbattery.png) |

The Turtlebot 4 consists of two DDS connected machines, the Pi 4 and the Create3 which are internally connected via USB-C. Both must be powered on for the robot to work properly.

![control](figs/control.png)
*Image source: [Clearpath Robotics](https://turtlebot.github.io/turtlebot4-user-manual/mechanical/turtlebot4.html#removing-the-pcba)*

## STEP 1 - Turning on/restarting the robot 

> The Create3 cannot be turned off while on the charging station, and the only way to turn it on when it's powered off is to place it on the dock. If the power is cut to the Pi 4 by turning off the Create3 before executing safe shutdown, it might corrupt the SD card. Always power off the Pi 4 first, before cutting power from the Create3.

If the robot is powered off (lidar not spinning, LED ring is off):
- place the robot on the dock so both computers boot in sync
- wait for the beep (it may take a minute or two)

If the robot is already on the dock (lidar not spinning, LED ring is on):
- remove the robot from the dock
- hold the Create3 power button for 10 seconds so the base powers down
- follow instructions for powering on

If the robot is running but not responsive (lidar is spinning, LED ring is on):
- remove the robot from the dock
- hold the Pi Shutdown button for a few seconds (the lidar should stop spinning)
- hold the Create3 power button for 10 seconds so the base powers down
- follow instructions for powering on

![poweroff](figs/poweroff.png)

If the buttons on the display do not respond or the display is off, it means that the Pi 4 is powered off, unless the lidar is spinning, in which case the ROS 2 nodes that handle the screen may not be running or the boot process hasn't finished yet.

![oled](figs/oled.png)
*OLED display: The first line shows the IP address. Buttons 3 and 4 select the action, button 1 confirms it, button 2 scrolls back to top. The top bar shows battery level and IP address*


⚠️ To keep robots charged and ready for use, follow these steps when you finish work:
- place the robot on the charging dock (green LED on the dock should turn on)
- if the robot was completely powered off before, wait until the chime sound
- hold the Pi Shutdown button for a few seconds (the lidar should stop spinning)

If the Pi remains powered on, the robot will charge extremely slowly, so it's essential to turn it off for faster charging.

## STEP 2 - Connecting to the robot from your workstation

As the robot turns on, it will connect to its dedicated colour-coded router and network. Robots are split over multiple routers on different channels for more concurrent bandwidth.

If you are using your own laptop, first connect to the same network as the robot (e.g. for the robot `Kili`, connect to `KiliWifi` or `KiliWifi_5GHz`).

Lab workstations are already connected to one of the robot routers via Ethernet, and should be used with the corresponding robot.

![pc](https://github.com/user-attachments/assets/67fe1698-475b-4ff2-ae51-f647a1782ed5)
*The router to which the Gloin Turtlebot connects to, and the workstation that can be used with it*

Next, you need to adjust your environment variables in the `~/.bashrc` file. The `ROS_DOMAIN_ID` should match the one written on the robot, and `RMW_IMPLEMENTATION` needs to be set to `rmw_cyclonedds_cpp`.

![domain](figs/domain.png)
*The sticker with the `ROS_DOMAIN_ID`, different for each robot*

The Cyclone middleware requires [an xml config](cyclonedds.xml), specified as the `CYCLONEDDS_URI` envrionment variable. For more info about the cyclone config, [see here](https://iroboteducation.github.io/create3_docs/setup/xml-config/), you might need to specify the network interface explicitly if using multiple ones e.g. `<NetworkInterface name="wlan0" />`. Lab workstations already have cyclone configured, if using your own laptop, you need to download and link the provided config.

Make sure your `~/.bashrc` looks as follows:
```
export CYCLONEDDS_URI='/home/<your_user_name>/cyclonedds.xml'
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<your_robot_domain_id>
```
 
Then, `source ~/.bashrc`, or re-open any terminals so the new `.bashrc` is properly sourced. 

Optionally, restart the `ros2 daemon` just to be sure:
```
ros2 daemon stop; ros2 daemon start
```

This should be it!

----------------

#### Check 1

To check if the robot is properly connected you can inspect topics with:

    ros2 topic list

You should see a lot of topics. If not, recheck:
- that the robot is charged and powered on
- your network settings
- your DDS config (`echo $RMW_IMPLEMENTATION`)
- your domain config (`echo $ROS_DOMAIN_ID`)

#### Check 2

See if you are receiving the `/odom` topic:

    ros2 topic echo /odom

You should see a stream of data. If not, the Create3 base has not yet booted, wait for the chime sound.

#### Check 3

See if you can move the robot with keyboard teleoperation:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

You should be able to move the robot.

If you have passed all the checks, you can move on with building a map and navigating the robot.

## Building a map

The procedure for building a map is a little simpler than when using the simulation, we only need to start the SLAM and the keyboard teleoperation. 

Please follow the [official map making tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html).

## Navigating a map

Once you have save a map, we can have the robot navigate around the map. 

Check the [official navigation tutorial](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html).

## Transform frames

![tf_rviz](figs/tf_rviz.png)

The transform system in ROS 2 is a standardized way to define and handle coordinate frames and transforms between them. It consists of two topics: `/tf` and `/tf_static`, as well as a collection of libraries for python and C++.

Every robot is defined by a collection of coordinate frames, with `base_link` being considered the parent frame for the current robot. Other common frames include:

- `base_footprint` - typically below base link, where the robot meets the ground
- `odom` - the transformation between the location where the robot started and where it is now, based on wheel encoder data
- `map` - the correction for odometry drift, usually calculated based on external landmarks
- `world` or `earth` - localizes one or multiple robots on the world WGS84 system based on GNSS or other global data
- sensor frames like `camera`, `imu`, `laser`, `sonar`, etc. which reflect the position and rotation of sensors mounted on the real robot, so their data can be accurately transformed into other frames

When multiple robots are in the same TF graph, the conventional way to separate them is using namespacing, i.e. prepending a robot name to the frames. that way `/robot1/base_link` can be distinct from `/robot2/base_link` while using the same conventions or even be built from the same URDF file.

You can use the following command to collect and view currently available frames:

    ros2 run tf2_tools view_frames

If you're running the turtlebot simulator, the generated PDF should look something similar as the excerpt below, a full graph of all connected frames:

![tf_frames](figs/tf_frames.png)

Check the [official documentation page](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html) for more info.
