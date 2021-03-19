# Component api

This section describes all the codelets part of this driver.

For each component, the incoming and outgoing message channels and the corresponding
message types are listed. Additionally, all parameters with their names and types
and corresponding default values are explained.

## UniversalRobots

The `UniversalRobots` codelet handles the interface between an Isaac application
and the main driver. It is publishing actual robot and IO state and receiving IO
and arm commands.

### Incoming messages

- **arm_command** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Receives command for arm, parsed based on control mode.

- **io_command** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Receives IO command, to set any of the robot's IOs.

- **stop_control** [[BooleanProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#booleanproto)]:
Stop control of the robot through Isaac. The boolean flag needs to be true, to stop
control of the robot through Isaac. This will make the "External Control" program
node on the UR-Program return.

- **resend_control_script** [[BooleanProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#booleanproto)]:
Receives a signal to send the URScript program to the robot for execution, when
the robot is in headless mode. The boolean flag needs to be true, to send the script.
Use this after the program has been interrupted, e.g. by a protective- or EM-stop.

- **set_speed_slider** [[SpeedSliderProto](../../ur_msg/README.md#SpeedSliderProto)]:
Set the speed slider fraction used by the robot's execution. Values should be between
0 and 1. Only set this smaller than 1 if you are using the ur_controller or you
know what you're doing. Using this with other controllers might lead to unexpected
behaviors.

### Outgoing messages

- **arm_state** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Current state of the arm, includes joint angles, joint speeds and current speed fraction.

- **io_state** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Current IO, includes digital inputs and outputs.

- **robot_program_running** [[BooleanProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#booleanproto)]:
Whenever the runtime state of the "External Control" program node in the UR-program
changes a message gets published. So this is equivalent to the information whether
the robot accepts commands from Isaac side.

- **trajectory** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Publish a trajectory to the controller to make sure the robots stays in the same
position. The trajectory is published everytime "External Control" program node
starts running. This is used to make sure that the newest trajectory, is based upon
the robots current position when Isaac starts controlling the robot.

### Parameters

- **kinematic_tree** [std::string] [default=]: Name of the node containing the map:KinematicTree
component. This is used to obtain the names of the joints for parsing/creating
composite message.

- **controller** [std::string] [default=]: Name of the node containing the controller
component. This node is started and stopped based upon the drivers control status
of the robot.

- **dashboard_client** [std::string] [default=]: Name of the node containing the
DashboardClientIsaac component This is used to start the node and set the ip address.
Nothing will happen if this parameter is kept empty.

- **control_mode** [ArmControlMode] [default = kJointPosition]: Set control mode
for arm. Use "joint position" or "joint speed" to specify wheter the control mode
should be joint positions or joint speeds.

- **misc_names** [std::vector\<std::string>] [default = std::vector\<std::string>({
"timestamp", "speed_fraction" })]: Name of speed_fraction and timestamp used for
creating composite message.

- **tool_digital_out_names** [std::vector\<std::string>] [default = std::vector\<std::string>({
"tool_digital_out_0", "tool_digital_out_1" })]: Entity name for digital tool output.

- **tool_digital_in_names** [std::vector\<std::string>] [default = std::vector\<std::string>({
"tool_digital_in_0", "tool_digital_in_1" })]: Entity name for digital tool input.

#### Driver settings

These are parameters parsed forward to the UrDriver object, which is created, during
startup of this codelet.

- **robot_ip** [std::string] [default=]: Ip address of the robot.

- **control_script_program** [std::string] [default = "packages/universal_robots/ur_robot_driver/resources/ros_control.urscript"]:
URScript file that is sent to the robot.

- **rtde_input_recipe** [std::string] [default = "packages/universal_robots/ur_robot_driver/resources/rtde_input_recipe.txt"]:
Filename where the input recipe is stored in.

- **rtde_output_recipe** [std::string] [default = ""packages/universal_robots/ur_robot_driver/resources/rtde_output_recipe.txt"]:
Filename where the output recipe is stored in.

- **headless_mode** [bool] [default = false]: Parameter to control if the driver
should be started in headless mode.

- **servoj_gain** [double] [default = 2000]: Specify gain for servoing to position
in joint space. A higher gain can sharpen the trajectory.

- **servoj_lookahead_time** [double] [default = 0.03]: Specify lookahead time for
servoing to position in joint space. A longer lookahead time can smooth the trajectory.

## DashboardClientIsaac

The `dashboardClientIsaac` class is the interface between the dashboardserver and
an Isaac application. Almost all the dashboard commands can be sent through this
codelet to the dashboardserver and this codelet, will return the anwser to the application.

If you use this codelet together with the UniversalRobots codelet, the node containing
this codelet can be started through the UniversalRobots codelet, this is done so
you only have to set the robots ip once. See this [subgraph](../apps/ur_eseries_robot.subgraph.json)
for inspiration.

### Incoming messages

- **dashboard_command** [[DashboardCommandProto](../../ur_msg/README.md#DashboardCommandProto)]:
Receiving the dashboard command, which is forwarded to the dashboard server.

### Outgoing messages

- **anwser** [[DashboardCommandStatusProto](../../ur_msg/README.md#DashboardCommandStatusProto)]:
Channel to publish the anwser from the dashbordserver and whether the command was
a succes.

### Parameters

- **robot_ip** [std::string] [default=]: The ip address of the robot.

## ScaledMultiJointController

The ScaledMultiJointController class receives a trajectory and calculates the actual
joint command to publish each iteration. It slows down trajectory execution according
to the actual speedscaling of the robot.

### Incoming messages
- **current_arm_state** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Receives current state of the robot arm, this also includes the actual speedscaling.

- **plan** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Receives a plan that contains a trajectory for each joint. If linear interpolation
is configured the trajectory must contain joint position. If cubic interpolation
is configured the trajectory must contain joint position and joint speeds. If quintic
interpolation is configured the trajectory must contain joint positions, joint speeds
and joint accelerations.

### Outgoing messages

- **joint_command** [[CompositeProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#compositeproto)]:
Publishes the actual joint command as joint position or joint speeds based upon control_mode.

- **trajectory_executed_succesfully** [[BooleanProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#booleanproto)]:
Publishes the status after a trajectory has been executed. The flag will be true
if the joints are within the tolerance after trajectory execution and false otherwise.

### Parameters

- **kinematic_tree** [std::string] [default=]: Name of the node containing the map:KinematicTree
component. This is used to obtain the names of the joints for parsing/creating
composite message.

- **interpolation_scheme** [InterpolationScheme] [default=quintic_interpolation]:
Set the interpolation scheme used to calculate joint commands and parse joint trajectory.

- **control_mode** [ArmControlMode] [default=kJointPosition]: Set the control mode.
Use "joint position" or "joint speed" to specify wheter the control mode should be
joint positions or joint speeds.

- **goal_time** [double] [default = 0.0]: Time after the last waypoint in the trajectory
is published and until the joints should be within the tolerance.

- **tolerance** [double] [default = 0.0]: Tolerance for arrival. The trajectory
is completed when the error for all joints are below this tolerance after the last
waypoint in the trajectory is published. If the tolerance is 0.0, no check will
be made.

- **gains** [nlohmann::json] [default=(nlohmann::json{{"gains", {}}})]: PID gains
for each joint, when control mode is kJointSpeed.

## ControllerStopper

ControllerStopper class is a small helper component that stops and restarts the
controller based on a boolean message. When the message goes to false, the controller
is stopped. If message returns to true the stopped controller is restarted. This
component is automatically started inside the driver.

### Incoming messages

- **robot_program_running** [[BooleanProto](https://docs.nvidia.com/isaac/isaac/doc/doc/message_api.html#booleanproto)]:
Receives the robots running state.

### Parameters

- **controller** [std::string] [default=]: Name of the node containing the controller
node that is sending targets to the driver. This node is started and stopped based
upon the drivers control status of the robot.
