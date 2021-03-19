// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 Universal Robots A/S
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
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#include "ScaledMultiJointController.hpp"
#include "packages/map/KinematicTree.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac
{
namespace ur_controller
{
void ScaledMultiJointController::start()
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

  // Get the kinematic_tree object
  const kinematic_tree::KinematicTree& model = maybe_kinematic_tree_component->model();
  const auto& links = model.getActiveLinks();
  joint_names_.clear();
  for (const auto* link : links)
  {
    joint_names_.push_back(link->name);
  }

  // Update states according to number of joints
  number_of_joints_ = joint_names_.size();
  controllerState_ = State(number_of_joints_);
  robotState_ = State(number_of_joints_);
  errorState_ = State(number_of_joints_);
  cur_traj_.resize(number_of_joints_);
  successful_joints_.resize(number_of_joints_);
  plan_parser_.resize(number_of_joints_);
  pid_.resize(number_of_joints_);
  ff_term_.resize(number_of_joints_);

  control_mode_ = get_control_mode();
  interpolation_scheme_ = get_interpolation_scheme();
  succesful_arm_parse_ = false;
  succesful_plan_parse_ = false;

  // Init parsers and schemas.
  initPlanParser();
  initStateParser();
  initCommandSchema();

  // Setup tick
  tickPeriodically();
}

void ScaledMultiJointController::tick()
{
  if (rx_plan().available())
  {
    const int64_t time = rx_plan().acqtime();
    bool isFirstMessage = !plan_time_;
    bool unseenMessage = time > *plan_time_;
    if (isFirstMessage || unseenMessage)
    {
      plan_time_ = time;
      succesful_plan_parse_ = parsePlan();

      // Update time data
      period_ = 0;
      controller_uptime_ = getTickTime();
      tick_time_ = getTickTime();
    }
  }
  if (rx_current_arm_state().available())
  {
    const int64_t time = rx_current_arm_state().acqtime();
    bool isFirstMessage = !arm_time_;
    bool unseenMessage = time > *arm_time_;
    if (isFirstMessage || unseenMessage)
    {
      arm_time_ = time;
      succesful_arm_parse_ = parseJointStates();
      // initialize joints
      if (isFirstMessage && succesful_arm_parse_)
      {
        for (unsigned int i = 0; i < number_of_joints_; ++i)
        {
          controllerState_.position[i] = robotState_.position[i];
          controllerState_.velocity[i] = robotState_.velocity[i];
          controllerState_.acceleration[i] = 0.0;
        }
      }
    }
  }
  if (succesful_plan_parse_ && succesful_arm_parse_)
  {
    calculateCommand();
    publishCommand();
  }
}

void ScaledMultiJointController::initPlanParser()
{
  switch (interpolation_scheme_)
  {
    case linear_interpolation:
      for (unsigned int i = 0; i < number_of_joints_; ++i)
      {
        std::vector<composite::Quantity> quantities;
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
        plan_parser_[i].requestSchema(std::move(quantities));
      }
      return;
    case cubic_interpolation:
      for (unsigned int i = 0; i < number_of_joints_; ++i)
      {
        std::vector<composite::Quantity> quantities;
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kSpeed));
        plan_parser_[i].requestSchema(std::move(quantities));
      }
      return;
    case quintic_interpolation:
      for (unsigned int i = 0; i < number_of_joints_; ++i)
      {
        std::vector<composite::Quantity> quantities;
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kSpeed));
        quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kAcceleration));
        plan_parser_[i].requestSchema(std::move(quantities));
      }
      return;
    default:
      LOG_WARNING("interpolation shceme not recognized");
      return;
  }
}

void ScaledMultiJointController::initStateParser()
{
  position_state_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kPosition));
  velocity_state_parser_.requestSchema(composite::Schema(joint_names_, composite::Measure::kSpeed));

  std::vector<composite::Quantity> misc_names;
  misc_names.push_back(composite::Quantity::Scalar("speed_fraction", composite::Measure::kNone));
  speed_fraction_parser_.requestSchema(composite::Schema(std::move(misc_names)));
}

bool ScaledMultiJointController::parsePlan()
{
  // Reset plan parser if interpolation scheme changes.
  if (interpolation_scheme_ != get_interpolation_scheme())
  {
    interpolation_scheme_ = get_interpolation_scheme();
    initPlanParser();
  }
  std::vector<Timeseries<VectorXd, double> > parsed_traj(6);
  for (unsigned int i = 0; i < number_of_joints_; ++i)
  {
    if (!plan_parser_[i].parse(rx_plan().getProto(), rx_plan().buffers(), "time", parsed_traj[i]))
    {
      LOG_WARNING("failed to parse plan for joint %s", joint_names_[i].c_str());
      return false;
    }
  }
  // Initilaize trajectory
  initTrajectory(parsed_traj);
  return true;
}

bool ScaledMultiJointController::parseJointStates()
{
  VectorXd current_position(number_of_joints_);
  if (!position_state_parser_.parse(rx_current_arm_state().getProto(), rx_current_arm_state().buffers(),
                                    current_position))
  {
    LOG_WARNING("Failed to parse joint position from driver");
    return false;
  }
  VectorXd current_velocity(number_of_joints_);
  if (!velocity_state_parser_.parse(rx_current_arm_state().getProto(), rx_current_arm_state().buffers(),
                                    current_velocity))
  {
    LOG_WARNING("Failed to parse joint velocity from driver");
    return false;
  }
  VectorXd speed_fraction(1);
  if (!speed_fraction_parser_.parse(rx_current_arm_state().getProto(), rx_current_arm_state().buffers(),
                                    speed_fraction))
  {
    LOG_WARNING("Failed to parse speed scaling from driver");
    return false;
  }
  robotState_.position = current_position;
  robotState_.velocity = current_velocity;
  speed_fraction_ = speed_fraction[0];
  return true;
}

void ScaledMultiJointController::initCommandSchema()
{
  std::vector<composite::Quantity> quantities;
  if (control_mode_ == kJointPosition)
  {
    for (unsigned int i = 0; i < number_of_joints_; i++)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kPosition));
    }
  }
  else if (control_mode_ == kJointSpeed)
  {
    for (unsigned int i = 0; i < number_of_joints_; i++)
    {
      quantities.push_back(composite::Quantity::Scalar(joint_names_[i], composite::Measure::kSpeed));
    }
  }
  else
  {
    LOG_WARNING("invalid control mode");
    return;
  }
  command_schema_ = composite::Schema(std::move(quantities));
}

void ScaledMultiJointController::publishCommand()
{
  // Reset command schema if control mode changes
  ArmControlMode mode = get_control_mode();
  if (control_mode_ != mode)
  {
    control_mode_ = mode;
    initCommandSchema();
  }

  if (control_mode_ == kInvalid)
  {
    LOG_WARNING("invalid control mode");
    return;
  }
  auto proto_builder = tx_joint_command().initProto();
  composite::WriteSchema(command_schema_, proto_builder);

  // Allocate tensor1d to store state data
  Tensor1d state_data(number_of_joints_);

  if (control_mode_ == kJointPosition)
  {
    for (unsigned int i = 0; i < number_of_joints_; i++)
    {
      state_data(i) = controllerState_.position[i];
    }
  }
  else if (control_mode_ == kJointSpeed)
  {
    for (unsigned int i = 0; i < number_of_joints_; i++)
    {
      state_data(i) = controllerState_.velocity[i];
    }
  }
  ToProto(std::move(state_data), proto_builder.initValues(), tx_joint_command().buffers());
  tx_joint_command().publish();
}

void ScaledMultiJointController::publishTrajectoryStatus(bool trajectory_status)
{
  auto proto = tx_trajectory_executed_succesfully().initProto();
  proto.setFlag(trajectory_status);
  tx_trajectory_executed_succesfully().publish();
}

void ScaledMultiJointController::calculateCommand()
{
  // Update time according to current speed scale
  period_ = (getTickTime() - tick_time_) * speed_fraction_;
  controller_uptime_ = controller_uptime_ + period_;
  tick_time_ = getTickTime();

  for (unsigned int i = 0; i < number_of_joints_; ++i)
  {
    // Interpolate current trajectory
    cur_traj_[i].interpolate(controller_uptime_, controllerState_.position[i], controllerState_.velocity[i],
                             controllerState_.acceleration[i]);

    errorState_.position[i] = DeltaAngle(controllerState_.position[i], robotState_.position[i]);
    errorState_.velocity[i] = controllerState_.velocity[i] - robotState_.velocity[i];
    errorState_.acceleration[i] = 0.0;

    // Check tolerance, if there is an active goal
    if (active_goal_)
    {
      const double end_time = cur_traj_[i].endTime();
      // Done executing last segment in the trajectory
      if (controller_uptime_ > end_time)
      {
        const bool inside_goal_tolerance = insideTolerance(errorState_.position[i]);
        if (inside_goal_tolerance)
        {
          successful_joints_[i] = 1;
        }
        else if (controller_uptime_ < end_time + get_goal_time())
        {
          // Still have time to reach target
        }
        else
        {
          LOG_ERROR("Tolerance failed for joint: %s", joint_names_[i].c_str());
          active_goal_ = false;
          publishTrajectoryStatus(false);
        }
      }
    }
    if (control_mode_ == kJointSpeed && active_pid_)
    {
      double target = pid_[i].calculate(errorState_.position[i], errorState_.velocity[i], period_);
      controllerState_.velocity[i] = controllerState_.velocity[i] * ff_term_[i] + target;
    }
    // Show data
    show(joint_names_[i] + " position error", errorState_.position[i]);
    show(joint_names_[i] + " velocity error", errorState_.velocity[i]);
    show(joint_names_[i] + " position", controllerState_.position[i]);
    show(joint_names_[i] + " velocity", controllerState_.velocity[i]);
    show(joint_names_[i] + " acceleration", controllerState_.acceleration[i]);
  }
  if (active_goal_)
  {
    unsigned int count =
        std::accumulate(successful_joints_.begin(), successful_joints_.end(), static_cast<std::size_t>(0));
    if (count == number_of_joints_)
    {
      active_goal_ = false;
      std::fill(successful_joints_.begin(), successful_joints_.end(), false);
      publishTrajectoryStatus(true);
    }
  }
}

bool ScaledMultiJointController::insideTolerance(const double& error_pos)
{
  // The check is only made if the tolerance is above 0
  const double tolerance = get_tolerance();
  return !(tolerance > 0.0 && std::abs(error_pos) > tolerance);
}

void ScaledMultiJointController::initTrajectory(std::vector<Timeseries<VectorXd, double> > trajectory)
{
  // Get PID gains
  nlohmann::json gains = get_gains();

  active_pid_ = true;
  for (unsigned int i = 0; i < number_of_joints_; ++i)
  {
    // Initialize trajectory
    cur_traj_[i].initTrajectory(trajectory[i]);

    const auto pid_joint = serialization::TryGetFromMap<nlohmann::json>(gains, joint_names_[i]);
    if (pid_joint)
    {
      ff_term_[i] = serialization::GetFromMapOrDefault(*pid_joint, "velocity_ff", 1.0);
      if (!pid_[i].init(*pid_joint))
      {
        active_pid_ = false;
      }
    }
    else
    {
      active_pid_ = false;
    }
  }
  active_goal_ = true;
  std::fill(successful_joints_.begin(), successful_joints_.end(), false);
}

}  // namespace ur_controller
}  // namespace isaac