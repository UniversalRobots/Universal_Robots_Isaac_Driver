# UR controller

This package contains a controller, that is special to the UR family. This controller
can handle the the actual speedscaling of the robot, by slowing down trajectory execution.

## Isaac Api

This folder contains a controller, not being available in Isaac SDK. The controller
is created to offer a controller that can handle the actual  speed scaling of a
UR robot. This controller is meant to be used instead of the [MultiJointcontroller](https://docs.nvidia.com/isaac/isaac/doc/doc/component_api.html#isaac-controller-multijointcontroller)
provided by nvidia.

This folder contains 1 codelet that can be used in applications:

- **ScaledMultiJointController** This codelets interpolates a trajectory received
from the planner to determine the current command to send. This codelet can output
either a joint position or speed command. It uses the actual speed scaling reported
by UniversalRobots to reduce progress in the trajectory. The component is further
documented in a [standalone document](../ur_robot_driver/doc/component_api.md#ScaledMultiJointController)

The codelet uses functionality from other classes in this folder, these classes
can be seen below:

- **Pid** class used to calculate the control output. This class is used in the
ScaledMultiJointController to calculate joint speeds.

- **JointTrajectory** class is used to store a trajectory of waypoints for one joint.
The waypoints consist of time, position, velocity and acceleration. This Class uses
JointSegment to interpolate between two waypoints.

- **JointSegment** class calculates a segment between two waypoints. The segment
is a function describing joint position as a function of time. The function can
be linear, cubic or quintic.

### Subgraph

This controller comes with a [subgraph](apps/scaled_multi_joint_ur_control.subgraph.json)
that integrates the [MultiJointLqrPlanner](https://docs.nvidia.com/isaac/isaac/doc/doc/component_api.html#isaac-planner-multijointplanner),
[ScaledMultiJointController](../ur_robot_driver/doc/component_api.md#ScaledMultiJointController)
and [KinematicTree](https://docs.nvidia.com/isaac/isaac/doc/doc/component_api.html#isaac-map-kinematictree)
components. The subgraph provides the following interface edges:

- **joint_state(input)** "subgraph/interface/joint_state"

- **joint_target(input)** "subgraph/interface/joint_target"

- **joint_command(output)** "subgraph/interface/command"

- **trajectory_executed_succesfully(output)** "subgraph/interface/trajectory_executed_succesfully"

## Controller description

This controller works similar to [MultiJointcontroller](https://docs.nvidia.com/isaac/isaac/doc/doc/component_api.html#isaac-controller-multijointcontroller)
provided by nvidia. It receives a plan that contains a trajectory for each joint.
It interpolates the received plan and publish the command as either joint speeds
or joint position.

However, it is extended to handle the robot's execution speed specifically.
Because MultiJointController would interpolate the trajectory with the configured
time constraints (ie: always assume maximum velocity and acceleration configured
in the application), this could lead to significant path deviation due to multiple
reasons:

- The speed slider on the robot might not be at 100%, so motion commands sent from
ROS would effectively get scaled down resulting in a slower execution.

- The robot could scale down motions based on configured safety limits resulting
in a slower motion than expected and therefore not reaching the desired target in
a control cycle.

- Motions might not be executed at all, e.g. because the robot is E-stopped or in
a protective stop

- Motion commands sent to the robot might not be interpreted, e.g. because there
is no `external_control` program node running on the robot controller.

- The program interpreting motion commands could be paused.

The following plot illustrates the problem:

![Trajectory execution with default multiJointController](doc/traj_without_speed_scaling.png
"Trajectory execution with default multiJointController")

This graph shows a trajectory with one joint being moved to a target point. The
joint speed is limited to 40% on the teach pendant, speed scaling activates and
limits the joint speed. As a result, the target trajectory (blue) doesn't get executed
by the robot, instead the red trajectory is executed. The green line is the error
between the blue and red line in each control cycle. We can also see that the robot
ends up oscillating around the target joint position.

All of the cases mentioned above are addressed by the ScaledMultiJointController
versions. Trajectory execution can be transparently scaled down using the speed
slider on the teach pendant without leading to additional path deviations. Pausing
the program or hitting the E-stop effectively leads to speed_scaling being 0 meaning
the trajectory will not be continued until the program is continued. This way, trajectory
executions can be explicitly paused and continued.

With the ScaledMultiJointController the example motion shown in the previous diagram
becomes:

![Trajectory execution with scaled_joint_trajectory_controller](doc/traj_with_speed_scaling.png
"Trajectory execution with scaled_joint_trajectory_controller")

The deviation between trajectory interpolation on the Isaac side and actual robot
execution stays minimal and the robot reaches the setpoint instead of oscillating
around it.

Under the hood this is implemented by proceeding the trajectory not by a full time
step but only by the fraction determined by the current speed scaling. If speed
scaling is currently at 50% then interpolation of the current control cycle will
start half a time step after the beginning of the previous control cycle.
