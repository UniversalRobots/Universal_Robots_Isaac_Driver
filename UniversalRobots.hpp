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

#include "ur_robot_driver/ur/ur_driver.h"
#include "engine/alice/alice_codelet.hpp"
#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/composite/gems/parser.hpp"
#include "messages/composite.capnp.h"

namespace isaac
{
namespace universal_robots
{
// Available control modes for arm.
enum ArmControlMode
{
  kJointPosition,  // Control joints position
  kJointSpeed,     // Control joints speed
  kInvalid = -1    // Invalid control mode, returned from an invalid string in JSON
};

// Mapping between each ArmControlMode type and an identifying string.
NLOHMANN_JSON_SERIALIZE_ENUM(ArmControlMode, { { kInvalid, nullptr },
                                               { kJointPosition, "joint position" },
                                               { kJointSpeed, "joint "
                                                              "speed" } });

enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

class UniversalRobots : public isaac::alice::Codelet
{
public:
  void start() override;
  void tick() override;
  void stop() override;

  // Command for arm, parsed based on control mode
  ISAAC_PROTO_RX(CompositeProto, arm_command);

  // Command for arm, parsed based on control mode
  ISAAC_PROTO_RX(CompositeProto, io_command);

  // Current state, includes joint angles and speed
  ISAAC_PROTO_TX(CompositeProto, arm_state);

  // Current io, includes digital inputs and outputs
  ISAAC_PROTO_TX(CompositeProto, io_state);

  // Set control mode for arm.
  ISAAC_PARAM(ArmControlMode, control_mode, kJointPosition);
  // Name of the node containing the map:KinematicTree component. This is used to obtain the names
  // of the joints for parsing/creating composite message.
  ISAAC_PARAM(std::string, kinematic_tree);
  // Name of the joints for parsing/creating composite message.
  ISAAC_PARAM(std::vector<std::string>, misc_names, std::vector<std::string>({ "timestamp", "speed_fraction" }));
  // Entity name for digital output from the arm to devices on the IO interface.
  ISAAC_PARAM(std::vector<std::string>, tool_digital_out_names,
              std::vector<std::string>({ "tool_digital_out_0", "tool_digital_out_1" }));
  // Entity name for digital input to the arm from devices on the IO interface.
  ISAAC_PARAM(std::vector<std::string>, tool_digital_in_names,
              std::vector<std::string>({ "tool_digital_in_0", "tool_digital_in_1" }));

  // Driver settings
  ISAAC_PARAM(std::string, robot_ip, "127.0.0.1");
  ISAAC_PARAM(std::string, control_script_program,
              "packages/universal_robots/ur_robot_driver/resources/"
              "ros_control.urscript");
  ISAAC_PARAM(std::string, rtde_input_recipe,
              "packages/universal_robots/ur_robot_driver/resources/"
              "rtde_input_recipe.txt");
  ISAAC_PARAM(std::string, rtde_output_recipe,
              "packages/universal_robots/ur_robot_driver/resources/"
              "rtde_output_recipe.txt");

private:
  ur_driver::UrDriver* ur_driver;
  void handle_program_state(bool program_running);
  std::thread RTDEMainLoopTrd;
  bool rtde_thread_running;
  void RTDEMainLoop();

  struct RobotState
  {
    double timestamp;
    ur_driver::vector6d_t q, qd;
    uint64_t actual_dig_in_bits, actual_dig_out_bits;
    double speed_fraction;
  };

  std::mutex robotStateMutex;
  bool updateRobotState();
  RobotState latestRobotState;

  PausingState pausing_state_;
  double pausing_ramp_up_increment_;
  double speed_scaling_combined_;
  bool updateSpeedScale(ur_driver::rtde_interface::DataPackage* data_pkg);

  // Validates the kinematic tree object is consistent with the hardware model
  bool validateKinematicTree(const kinematic_tree::KinematicTree& model);

  // Request a schema for parsing command based on control mode
  void initCommandParser(const ArmControlMode mode);
  void initIOCommandParser();

  // Parse command from Composite message
  void parseCommand(const ArmControlMode mode);
  void parseIOCommand();

  // Generate the composite schema for publishing state
  void initStateSchema();
  void initIOSchema();

  // Arm state
  void publishRobotState(RobotState state);
  void publishIOState(RobotState state);
  void publishState();

  // Cache control mode. If control mode changed, need to reset parser schema.
  ArmControlMode control_mode_;
  // Cache list of joint names used in the kinematic tree
  std::vector<std::string> joint_names_;

  // Acquire time for most recently received position command
  std::optional<int64_t> last_command_time_, last_io_command_time_;

  // Parsers for command message
  composite::Parser command_parser_;
  composite::Parser io_command_parser_;

  // Cache schema for state message
  composite::Schema state_schema_;
  composite::Schema io_schema_;
};

}  // namespace universal_robots
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::universal_robots::UniversalRobots);
