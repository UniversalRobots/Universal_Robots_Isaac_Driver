# Universal_Robots_Isaac_Driver
Universal Robots have become a dominant supplier of lightweight, robotic manipulators for industry, as well as for scientific research and education. It is the core value of Universal Robots, to empower people to achieve any goal within automation.

<center><img src="ur_robot_driver/doc/initial_setup_images/e-Series.png" alt="Universal Robot e-Series family" style="width: 45%;"/></center>

The goal of this driver is to provide a stable and sustainable interface between UR robots and NVIDIA Isaac-SDK that strongly benefit all parties.

NVIDIA Isaac-SDK is the main software toolkit for NVIDIA Robotics. Included in the Isaac SDK is the ability to easily write modular applications and deploy them on UR Robots.

The driver is currently not completed, or intended for industrial use without modification. Contribution is welcome on the driver.

<div style="background-color:red;color:black;padding:5pt;">DISCLAIMER: This robot interface is under development and is not yet as robust as wanted. Documentation is missing and things will change.</div>


## Acknowledgement
 - This driver is forked from the [Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Drive),
which is forked from the [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver).

 - NVIDIA for making and maintain the [Isaac-SDK](https://www.nvidia.com/isaac) framework and libraries. As well as contributing to making this driver become reality.

## Recommended Prerequisites
 - Can be used on a CB3 (with software version >= 3.7) and e-Series (software >= 5.1) UR robot or simulator. Simulator can be found on Universal Robots website here: https://www.universal-robots.com/download/?option=41508#section41493

 - Can be used with NVIDIA Jetson hardware platform see here: https://www.universal-robots.com/plus/urplus-components/plug-ins-interfaces/jetson-agx-xavier-developer-kit/


Setup
-------------
1. Download Isaac SDK from https://developer.nvidia.com/isaac-sdk, then
   follow the documentation to setup and install dependencies.
2. Clone this repository.
3. In Isaac SDK folder, create a symlink "packages/universal_robots" to
   this repository:
```
isaac/packages/universal_robots -> /home/username/universal_robots
```

4. In Isaac SDK folder, run
```
bazel run packages/universal_robots/apps:simple_joint_control
```
   This should open a jupyter notebook in browser. Follow instructions
   there to manually control joints or digital io channels on the arm.
   Remember to update the ip for the arm.

5. A second example repetitively pick and place box between two preset
   waypoints. This app assumes a vacuum pump is connected through the
   digital io interfaces. Make sure to update the waypoints based on the
   actual setup, and make sure the path between the waypoints are
   obstacle-free, as the current planner in SDK do not have obstacle
   avoidance capability. To run the app:

```
bazel run packages/universal_robots/apps:shuffle_box
```
6. Both samples can be run on Jetson. Follow instructions in
   https://docs.nvidia.com/isaac/isaac/doc/getting_started.html#deploying-and-running-on-jetson
   to deploy the sample apps.
