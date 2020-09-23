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

#include "ur_robot_driver/rtde/get_urcontrol_version.h"
#include "ur_robot_driver/rtde/data_package.h"
#include "UniversalRobots.hpp"

#include "engine/gems/kinematic_tree/kinematic_tree.hpp"
#include "packages/map/KinematicTree.hpp"

namespace isaac
{
namespace universal_robots
{
namespace
{
constexpr int kNumJoints = 6;  // number of joints
}  // namespace

void UniversalRobots::handle_program_state(bool program_running)
{
}

void UniversalRobots::start()
{
  // Try to get the map::KinematicTree component
  const auto maybe_kinematic_tree_node = try_get_kinematic_tree();
  if (!maybe_kinematic_tree_node)
  {
    reportFailure("KinematicTree node is not specified.");
    return;
  }
  const auto maybe_kinematic_tree_component =
      node()->app()->getNodeComponentOrNull<map::KinematicTree>(*maybe_kinematic_tree_node);
  if (!maybe_kinematic_tree_component)
  {
    reportFailure("Node %s does not contain KinematicTree component.", maybe_kinematic_tree_node->c_str());
    return;
  }
  // Get the kinematic_tree object and validate against hardware
  const kinematic_tree::KinematicTree& model = maybe_kinematic_tree_component->model();
  if (!validateKinematicTree(model))
  {
    reportFailure("Kinematic tree model does not match hardware.");
    return;
  }

  LOG_INFO("Connecting to robot at %s", get_robot_ip().c_str());

  // Set up driver
  static const bool HEADLESS_MODE = true;
  static const std::string CALIB_CHECKSUM = "";
  ur_driver = new ur_driver::UrDriver(get_robot_ip(), get_control_script_program(), get_rtde_output_recipe(),
                                      get_rtde_input_recipe(),
                                      std::bind(&UniversalRobots::handle_program_state, this, std::placeholders::_1),
                                      HEADLESS_MODE, NULL, CALIB_CHECKSUM, 50001, 50002, 200, 0.03,
                                      // true //non blocking mode
                                      false  // blocking mode
  );

  // Init internal state
  initStateSchema();
  initIOSchema();
  initIOCommandParser();
  control_mode_ = kInvalid;
  pausing_ramp_up_increment_ = 0.01;
  pausing_state_ = PausingState::RUNNING;

  // Connect to RTDE
  ur_driver->startRTDECommunication();
  LOG_INFO("Waiting to receive the first RTDE data");
  while (!updateRobotState())
    ;  // TODO add count limit
  LOG_INFO("First RTDE data received");

  RTDEMainLoopTrd = std::thread(&UniversalRobots::RTDEMainLoop, this);

  // Setup tick
  tickPeriodically();
}

void UniversalRobots::tick()
{
  if (rx_arm_command().available())
  {
    const int64_t time = rx_arm_command().acqtime();
    bool isFirstMessage = !last_command_time_;
    bool unseenMessage = time > *last_command_time_;
    if (isFirstMessage || unseenMessage)
    {
      last_command_time_ = time;
      parseCommand(get_control_mode());
    }
  }

  if (rx_io_command().available())
  {
    const int64_t time = rx_io_command().acqtime();
    bool isFirstMessage = !last_io_command_time_;
    bool unseenMessage = time > *last_io_command_time_;
    if (isFirstMessage || unseenMessage)
    {
      last_io_command_time_ = time;
      parseIOCommand();
    }
  }

  publishState();
}

void UniversalRobots::stop()
{
  ur_driver->stopControl();
  rtde_thread_running = false;
  delete ur_driver;
}

bool UniversalRobots::validateKinematicTree(const kinematic_tree::KinematicTree& model)
{
  // Number of active links match number of joints
  const auto& links = model.getActiveLinks();
  if (links.size() != kNumJoints)
  {
    return false;
  }
  // Each active link only has one degree of freedom
  if (model.getMotorStateDimensions() != kNumJoints)
  {
    return false;
  }
  // Kinematic tree is valid, get the list of joint names for parsing composite message
  joint_names_.clear();
  for (const auto* link : links)
  {
    joint_names_.push_back(link->name);
  }
  return true;
}

void UniversalRobots::initCommandParser(const ArmControlMode mode)
{
  switch (mode)
  {
    case kJointPosition:
      command_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kPosition));
      break;
    case kJointSpeed:
      command_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kSpeed));
      break;
    default:
      command_parser_.requestSchema({});
      return;
  }
}

void UniversalRobots::initIOCommandParser()
{
  const auto io_names = get_tool_digital_out_names();
  io_command_parser_.requestSchema(composite::Schema(
      io_names, composite::Measure::kNone));  // Set to rotation so it's controllable by joint controller
}

ur_driver::vector6d_t toVector6D(const VectorXd q_in)
{
  ur_driver::vector6d_t q;

  for (unsigned int i = 0; i < kNumJoints; ++i)
    q[i] = q_in(i);

  return q;
}

void UniversalRobots::parseCommand(const ArmControlMode mode)
{
  // Check control mode
  if (mode == kInvalid)
    return;

  // Reset command parser if control mode changes
  if (control_mode_ != mode)
  {
    initCommandParser(mode);
    control_mode_ = mode;
  }

  bool parse_success = false;
  if (mode == kJointPosition || mode == kJointSpeed)
  {
    VectorXd command(kNumJoints);
    parse_success = command_parser_.parse(rx_arm_command().getProto(), rx_arm_command().buffers(), command);

    if (parse_success)
    {
      ur_driver::vector6d_t q_command = toVector6D(command);

      // Show targets from command
      for (unsigned int i = 0; i < q_command.size(); ++i)
      {
        show(joint_names_[i], q_command[i]);
      }

      if (mode == kJointPosition)
        ur_driver->writeJointCommand(q_command, ur_driver::comm::ControlMode::MODE_SERVOJ);
      else if (mode == kJointSpeed)
        ur_driver->writeJointCommand(q_command, ur_driver::comm::ControlMode::MODE_SPEEDJ);
    }
    else
    {
      LOG_WARNING("Failed to parse joint command.");
    }
  }
}

void UniversalRobots::parseIOCommand()
{
  VectorXd io_command(get_tool_digital_out_names().size());
  bool parse_success = io_command_parser_.parse(rx_io_command().getProto(), rx_io_command().buffers(), io_command);

  if (parse_success)
  {
    ur_driver::rtde_interface::RTDEWriter& rtdeWriter = ur_driver->getRTDEWriter();
    rtdeWriter.sendToolDigitalOutput(0, io_command(0) != 0.0 ? true : false);
    rtdeWriter.sendToolDigitalOutput(1, io_command(1) != 0.0 ? true : false);
  }
  else
  {
    LOG_WARNING("Failed to parse io command.");
  }
}

void UniversalRobots::initStateSchema()
{
  std::vector<composite::Quantity> quantities;
  // Add joint positions
  for (int i = 0; i < kNumJoints; i++)
  {
    quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
  }

  // Add joint velocities
  for (int i = 0; i < kNumJoints; i++)
  {
    quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kSpeed));
  }

  // Add timestamp and speed fraction
  const auto misc_names = get_misc_names();
  quantities.push_back(composite::Quantity::Scalar(misc_names[0], composite::Measure::kTime));
  quantities.push_back(composite::Quantity::Scalar(misc_names[1], composite::Measure::kNone));  // Measure should be
                                                                                                // fraction

  state_schema_ = composite::Schema(std::move(quantities));
}

void UniversalRobots::initIOSchema()
{
  std::vector<composite::Quantity> quantities;

  // Add digital outputs
  const auto io_out_names = get_tool_digital_out_names();
  ASSERT(io_out_names.size() == 2, "Wrong number of tool digital outs in config");
  for (unsigned int i = 0; i < io_out_names.size(); i++)
  {
    quantities.push_back(composite::Quantity::Scalar(io_out_names[i], composite::Measure::kNone));
  }

  // Add digital inputs
  const auto io_in_names = get_tool_digital_in_names();
  ASSERT(io_in_names.size() == 2, "Wrong number of tool digital inputs in config");
  for (unsigned int i = 0; i < io_in_names.size(); i++)
  {
    quantities.push_back(composite::Quantity::Scalar(io_in_names[i], composite::Measure::kNone));
  }

  io_schema_ = composite::Schema(std::move(quantities));
}

// The speed scaling is normally just the target speed fraction set by the user in
// the UI multiplied with the speed scale used by the robot to comply with limits.
// But in some cases (depending on robot state) the logic is a little more convoluted
bool UniversalRobots::updateSpeedScale(ur_driver::rtde_interface::DataPackage* data_pkg)
{
  bool rtde_read_succeed(false);
  double target_speed_fraction(0.0);
  double speed_scaling(0.0);
  uint32_t runtime_state(0);

  rtde_read_succeed = data_pkg->getData<double>("target_speed_fraction", target_speed_fraction);
  rtde_read_succeed = data_pkg->getData<double>("speed_scaling", speed_scaling);
  rtde_read_succeed = data_pkg->getData<uint32_t>("runtime_state", runtime_state);

  if (rtde_read_succeed)
  {
    // pausing state follows runtime state when pausing
    if (runtime_state == static_cast<uint32_t>(ur_driver::rtde_interface::RUNTIME_STATE::PAUSED))
    {
      pausing_state_ = PausingState::PAUSED;
    }
    // When the robot resumed program execution and pausing state was PAUSED, we enter RAMPUP
    else if (runtime_state == static_cast<uint32_t>(ur_driver::rtde_interface::RUNTIME_STATE::PLAYING) &&
             pausing_state_ == PausingState::PAUSED)
    {
      speed_scaling_combined_ = 0.0;
      pausing_state_ = PausingState::RAMPUP;
    }

    if (pausing_state_ == PausingState::RAMPUP)
    {
      double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
      speed_scaling_combined_ = std::min(speed_scaling_ramp, speed_scaling * target_speed_fraction);

      if (speed_scaling_ramp > speed_scaling * target_speed_fraction)
      {
        pausing_state_ = PausingState::RUNNING;
      }
    }
    else if (runtime_state == static_cast<uint32_t>(ur_driver::rtde_interface::RUNTIME_STATE::RESUMING))
    {
      // We have to keep speed scaling during RESUMING to prevent controllers from continuing to interpolate
      speed_scaling_combined_ = 0.0;
    }
    else
    {
      // Normal case
      speed_scaling_combined_ = speed_scaling * target_speed_fraction;
    }
  }

  return rtde_read_succeed;
}

bool UniversalRobots::updateRobotState()
{
  bool rtde_read_succeed(false);
  std::unique_ptr<ur_driver::rtde_interface::DataPackage> data_pkg;

  if (data_pkg = ur_driver->getDataPackage())
  {
    RobotState newState;
    rtde_read_succeed = data_pkg->getData<double>("timestamp", newState.timestamp);
    rtde_read_succeed = data_pkg->getData("target_q", newState.q) && rtde_read_succeed;
    rtde_read_succeed = data_pkg->getData("target_qd", newState.qd) && rtde_read_succeed;
    rtde_read_succeed =
        data_pkg->getData<uint64_t>("actual_digital_input_bits", newState.actual_dig_in_bits) && rtde_read_succeed;
    rtde_read_succeed =
        data_pkg->getData<uint64_t>("actual_digital_output_bits", newState.actual_dig_out_bits) && rtde_read_succeed;

    rtde_read_succeed = updateSpeedScale(data_pkg.get()) && rtde_read_succeed;

    newState.speed_fraction = speed_scaling_combined_;

    if (rtde_read_succeed)
    {
      std::lock_guard<std::mutex> guard(robotStateMutex);
      latestRobotState = newState;
    }
  }
  return rtde_read_succeed;
}

void UniversalRobots::RTDEMainLoop()
{
  rtde_thread_running = true;
  while (rtde_thread_running)
  {
    updateRobotState();
  }
}

void UniversalRobots::publishRobotState(RobotState robotStateSnapshot)
{
  auto proto_builder = tx_arm_state().initProto();
  composite::WriteSchema(state_schema_, proto_builder);

  // allocate tensor1d to store state data
  Tensor1d state_data(2 * kNumJoints + 2);

  int offset = 0;

  state_data(offset + 0) = robotStateSnapshot.q[0];
  state_data(offset + 1) = robotStateSnapshot.q[1];
  state_data(offset + 2) = robotStateSnapshot.q[2];
  state_data(offset + 3) = robotStateSnapshot.q[3];
  state_data(offset + 4) = robotStateSnapshot.q[4];
  state_data(offset + 5) = robotStateSnapshot.q[5];
  offset += kNumJoints;

  state_data(offset + 0) = robotStateSnapshot.qd[0];
  state_data(offset + 1) = robotStateSnapshot.qd[1];
  state_data(offset + 2) = robotStateSnapshot.qd[2];
  state_data(offset + 3) = robotStateSnapshot.qd[3];
  state_data(offset + 4) = robotStateSnapshot.qd[4];
  state_data(offset + 5) = robotStateSnapshot.qd[5];
  offset += kNumJoints;

  state_data(offset + 0) = robotStateSnapshot.timestamp;
  state_data(offset + 1) = robotStateSnapshot.speed_fraction;

  const auto misc_names = get_misc_names();
  show(misc_names[0], robotStateSnapshot.timestamp);
  show(misc_names[1], robotStateSnapshot.speed_fraction);

  ToProto(std::move(state_data), proto_builder.initValues(), tx_arm_state().buffers());
  tx_arm_state().publish();
}

void UniversalRobots::publishIOState(RobotState robotStateSnapshot)
{
  // Publish io to Isaac
  auto proto_builder = tx_io_state().initProto();
  composite::WriteSchema(io_schema_, proto_builder);

  const auto io_out_names = get_tool_digital_out_names();
  const auto io_in_names = get_tool_digital_in_names();

  // allocate tensor1d to store io data
  Tensor1d io_data(io_out_names.size() + io_in_names.size());

  io_data(0) = ((robotStateSnapshot.actual_dig_out_bits & 0x10000) != 0) ? 1.0 : 0.0;
  io_data(1) = ((robotStateSnapshot.actual_dig_out_bits & 0x20000) != 0) ? 1.0 : 0.0;
  io_data(2) = ((robotStateSnapshot.actual_dig_in_bits & 0x10000) != 0) ? 1.0 : 0.0;
  io_data(3) = ((robotStateSnapshot.actual_dig_in_bits & 0x20000) != 0) ? 1.0 : 0.0;

  // Outputs
  show(io_out_names[0], io_data(0));
  show(io_out_names[1], io_data(1));

  // Inputs
  show(io_in_names[0], io_data(2));
  show(io_in_names[1], io_data(3));

  ToProto(std::move(io_data), proto_builder.initValues(), tx_io_state().buffers());

  tx_io_state().publish();
}

// This call is too time consuming to be called in the RTDEMainLoop thread
void UniversalRobots::publishState()
{
  RobotState robotStateSnapshot;
  {
    std::lock_guard<std::mutex> guard(robotStateMutex);
    robotStateSnapshot = latestRobotState;
  }

  publishRobotState(robotStateSnapshot);
  publishIOState(robotStateSnapshot);
}

}  // namespace universal_robots
}  // namespace isaac
