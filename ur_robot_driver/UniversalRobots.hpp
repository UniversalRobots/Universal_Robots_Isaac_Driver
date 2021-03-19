// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2020”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#pragma once
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "ur_client_library/ur/ur_driver.h"
#include "engine/alice/alice_codelet.hpp"
#include "packages/math/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/composite/gems/serializer.hpp"
#include "packages/composite/gems/parser.hpp"
#include "messages/composite.capnp.h"
#include "messages/basic.capnp.h"
#include "packages/universal_robots/ur_msg/speed_slider.capnp.h"

namespace isaac
{
namespace ur_driver
{
/*!
 * \brief Available control modes for arm.
 */
enum ArmControlMode
{
  kJointPosition,  // Control joints position
  kJointSpeed,     // Control joints speed
  kInvalid = -1    // Invalid control mode, returned from an invalid string in JSON
};

/*!
 * \brief Mapping between each ArmControlMode type and an identifying string.
 */
NLOHMANN_JSON_SERIALIZE_ENUM(ArmControlMode, { { kInvalid, nullptr },
                                               { kJointPosition, "joint position" },
                                               { kJointSpeed, "joint speed" } });

/*!
 * \brief Possible states for robot control
 */
enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

/*!
 * \brief The UniversalRobots class handles the interface between an Isaac application and
 * the main driver. It is publishing actual robot and IO state and receiving IO and arm commands.
 */

class UniversalRobots : public isaac::alice::Codelet
{
public:
  /*!
   * \brief Handles the setup functionality. This includes collecting the kinematic tree and controller node, setting
   * up the main driver object and connecting to the robot.
   */
  void start() override;

  /*!
   * \brief This functions is run cyclic with a fixed frequency. It checks whether a new message is available,
   * in any of the incomming channels. In the end the robot and IO state is published.
   */
  void tick() override;

  /*!
   * \brief Used to stop the running thread and stop the drivers control of the robot.
   */
  void stop() override;

  /*!
   * \brief Receives command for arm, parsed based on control mode
   */
  ISAAC_PROTO_RX(CompositeProto, arm_command);

  /*!
   * \brief Receives IO command, to set any of the robot's IOs.
   */
  ISAAC_PROTO_RX(CompositeProto, io_command);

  /*!
   * \brief Receives a signal to stop control of robot through Isaac. The boolean flag needs to be true,
   * to stop control of the robot through Isaac. This will make the "External Control" program node on the UR-Program
   * return.
   */
  ISAAC_PROTO_RX(BooleanProto, stop_control);

  /*!
   * \brief Receives a signal to send the URScript program to the robot for execution, when the robot is in headless
   * mode. The boolean flag needs to be true, to send the script. Use this after the program has been interrupted, e.g.
   * by a protective- or EM-stop.
   */
  ISAAC_PROTO_RX(BooleanProto, resend_control_script);

  /*!
   * \brief Set the speed slider fraction used by the robot's execution. Values should be between 0 and 1.
   * Only set this smaller than 1 if you are using the ur_controller or you know what you're
   * doing. Using this with other controllers might lead to unexpected behaviors.
   */
  ISAAC_PROTO_RX(SpeedSliderProto, set_speed_slider);

  /*!
   * \brief Current state of the arm, includes joint angles, joint speeds and current speed fraction.
   */
  ISAAC_PROTO_TX(CompositeProto, arm_state);

  /*!
   * \brief Current IO state, includes digital inputs and outputs.
   */
  ISAAC_PROTO_TX(CompositeProto, io_state);

  /*!
   * \brief Whenever the runtime state of the "External Control" program node in the UR-program changes
   * a message gets published. So this is equivalent to the information whether the robot
   * accepts commands from Isaac side.
   */
  ISAAC_PROTO_TX(BooleanProto, robot_program_running);

  /*!
   * \brief Publish a trajectory to the controller to make sure the robots stays in the same position. The trajectory is
   * published everytime "External Control" program node starts running. This used to make sure that the newest
   * trajectory, is based upon the robots current position when Isaac starts controlling the robot.
   */
  ISAAC_PROTO_TX(CompositeProto, trajectory);

  /*!
   * \brief Name of the node containing the map:KinematicTree component. This is used to obtain the names
   * of the joints for parsing/creating composite message.
   */
  ISAAC_PARAM(std::string, kinematic_tree);

  /*!
   * \brief Name of the node containing the controller component.
   * This node is started and stopped based upon the drivers control status of the robot.
   */
  ISAAC_PARAM(std::string, controller);

  /*!
   * \brief Name of the node containing the dashboard component This is used to start the node and set the ip address.
   * Nothing will happen if this parameter is kept empty.
   */
  ISAAC_PARAM(std::string, dashboard_client);

  /*!
   * \brief Set control mode for arm.
   */
  ISAAC_PARAM(ArmControlMode, control_mode, kJointPosition);

  /*!
   * \brief Name of speed_fraction and timestamp used for creating composite message.
   */
  ISAAC_PARAM(std::vector<std::string>, misc_names, std::vector<std::string>({ "timestamp", "speed_fraction" }));

  /*!
   * \brief Entity name for digital tool output.
   */
  ISAAC_PARAM(std::vector<std::string>, tool_digital_out_names,
              std::vector<std::string>({ "tool_digital_out_0", "tool_digital_out_1" }));

  /*!
   * \brief Entity name for digital tool input.
   */
  ISAAC_PARAM(std::vector<std::string>, tool_digital_in_names,
              std::vector<std::string>({ "tool_digital_in_0", "tool_digital_in_1" }));

  /*!
   * \brief Driver settings
   *
   * \param robot_ip Ip address of the robot.
   * \param control_script_program URScript file that is sent to the robot.
   * \param rtde_input_recipe Filename where the input recipe is stored in.
   * \param rtde_output_recipe Filename where the output recipe is stored in.
   * \param headless_mode Parameter to control if the driver should be started in headless mode.
   * \param servoj_gain Specify gain for servoing to position in joint space. A higher gain can sharpen the trajectory.
   * \param servoj_lookahead_time Specify lookahead time for servoing to position in joint space. A longer lookahead
   * time can smooth the trajectory.
   */
  ISAAC_PARAM(std::string, robot_ip);
  ISAAC_PARAM(std::string, control_script_program,
              "packages/universal_robots/ur_robot_driver/resources/ros_control.urscript");
  ISAAC_PARAM(std::string, rtde_input_recipe,
              "packages/universal_robots/ur_robot_driver/resources/rtde_input_recipe.txt");
  ISAAC_PARAM(std::string, rtde_output_recipe,
              "packages/universal_robots/ur_robot_driver/resources/rtde_output_recipe.txt");
  ISAAC_PARAM(bool, headless_mode, false);
  ISAAC_PARAM(double, servoj_gain, 2000);
  ISAAC_PARAM(double, servoj_lookahead_time, 0.03)

private:
  std::unique_ptr<urcl::UrDriver> ur_driver_;

  /*!
   * \brief Callback to handle a change in the current state of the URScript running on the robot.
   * This function is called when the robot connects to the driver or when it disconnects from driver.
   *
   * \param program_running True when robot connects false when robot disconnects.
   */
  void handle_program_state(bool program_running);

  /*!
   * \brief Running thread which reads robot state data over RTDE connection.
   */
  std::thread rtde_main_loop_trd_;
  bool rtde_thread_running_;
  void RTDEMainLoop();

  /*!
   * \brief Struct representing the robot state.
   *
   * \param timestamp Time elapsed since the controller was started.
   * \param q Actual joint positions.
   * \param qd Actual joint speeds.
   * \param actual_dig_in_bits Current state of the digital inputs.
   * \param actual_dig_out_bits Current state of the digital outputs.
   * \param speed_fraction Current speed fraction.
   */
  struct RobotState
  {
    double timestamp;
    urcl::vector6d_t q, qd;
    uint64_t actual_dig_in_bits, actual_dig_out_bits;
    double speed_fraction;
  };

  /*!
   * \brief Reads robot state over RTDE connection.
   *
   * \returns true on succes.
   */
  std::mutex robot_state_mutex_;
  bool updateRobotState();
  RobotState latest_robot_state_;

  /*!
   * \brief Update the speed scaling depending on the robot state.
   *
   * \param data_pkg received RTDE Data package.
   *
   * \returns true on succes.
   */
  PausingState pausing_state_;
  double pausing_ramp_up_increment_;
  double speed_scaling_combined_;
  bool updateSpeedScale(urcl::rtde_interface::DataPackage* data_pkg);

  /*!
   * \brief Validates the kinematic tree object is consistent with the hardware model
   *
   * \param model The kinematicTree model.
   *
   * \returns true on succes.
   */
  bool validateKinematicTree(const kinematic_tree::KinematicTree& model);

  /*!
   * \brief Initialize controller_stopper node, used to stop and start the Isaac controller.
   * When the status goes to false, the controller is stopped. If status returns to true the stopped controller is
   * restarted.
   */
  void initControllerStopper(const std::string controller_node_name);

  /*!
   * \brief Request a schema for parsing IO command and arm command based on control mode.
   *
   * \param mode Arm control mode.
   */
  void initCommandParser(const ArmControlMode mode);
  void initIOCommandParser();

  /*!
   * \brief Parse commands from Composite message
   *
   * \param mode Arm control mode.
   */
  void parseCommand(const ArmControlMode mode);
  void parseIOCommand();

  /*!
   * \brief Generate the composite schema for publishing messages.
   */
  void initStateSchema();
  void initIOSchema();
  void initTrajectorySchema();

  /*!
   * \brief Publish current arm state and current IO state
   *
   * \param state Current robot state.
   */
  void publishRobotState(RobotState state);
  void publishIOState(RobotState state);
  void publishState();

  /*!
   * \brief Publish a trajectory to the Isaac controller to make sure the robots stays in the same position.
   * The trajectory is published everytime "External Control" program node starts running.
   *
   * \param state Current robot state.
   */
  void publishJointTrajectory(RobotState state);

  /*!
   * \brief Publishing program state.
   */
  void publishProgramState();

  /*!
   * \brief Stop robot control.
   */
  void stopControl();

  /*!
   * \brief Set the speed slider.
   */
  void setSpeedSlider();

  /*!
   * \brief Resend the control script.
   */
  void resendControlScript();

  // Cache control mode.
  ArmControlMode control_mode_;

  // Cache list of joint names
  std::vector<std::string> joint_names_;

  // Acquire time for most recently received message
  std::optional<int64_t> last_command_time_;
  std::optional<int64_t> last_io_command_time_;
  std::optional<int64_t> last_stop_control_time_;
  std::optional<int64_t> last_speed_slider_time_;
  std::optional<int64_t> last_resend_control_script_time_;

  // Parsers for command message
  composite::Parser command_parser_;
  composite::Parser io_command_parser_;

  // Cache schema for state message
  composite::Schema state_schema_;
  composite::Schema io_schema_;
  composite::Schema target_state_schema_;

  // Cache serialize for creating joint trajectory
  composite::Serializer trajectory_serializer_;

  // Robots runtime state and bool to reflect if control script is running on the robot.
  uint32_t runtime_state_;
  std::mutex runtime_mutex_;
  bool robot_program_running_;

  // Bool to reflect when to publish program state and trajectory reset.
  bool publish_program_state_;
  bool trajectory_reset_necessary_;

  // Node containing the controller
  alice::Node* controller_node_;

  // Node containing the dashboard client used for starting and stopping the client along with the driver
  alice::Node* dashboard_client_;
  bool dashboard_client_started_;
};

}  // namespace ur_driver
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ur_driver::UniversalRobots);
