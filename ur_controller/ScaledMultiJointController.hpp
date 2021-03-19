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

#pragma once
#include <vector>
#include <string>
#include <functional>

#include "engine/alice/alice_codelet.hpp"
#include "messages/composite.capnp.h"
#include "messages/basic.capnp.h"
#include "packages/composite/gems/parser.hpp"
#include "engine/gems/algorithm/timeseries.hpp"
#include "JointTrajectory.hpp"
#include "Pid.hpp"

namespace isaac
{
namespace ur_controller
{
/*!
 * \brief Possible control modes for arm.
 */
enum ArmControlMode
{
  kJointPosition,  // Control joints position
  kJointSpeed,     // Control joints speed
  kInvalid = -1    // Invalid control mode, returned from an invalid string in JSON
};

/*!
 * \brief Possible interpolation schemes.
 */
enum InterpolationScheme
{
  linear_interpolation,       // line ax+b
  cubic_interpolation,        // cubic polynomium - ax^3+bx^2+cx+d
  quintic_interpolation,      // quintic polynomium - ax^5+bx^4+cx^3+dx^2+ex+f
  invalid_interpolation = -1  // Invalid interpolation mode, returned from an invalid string in JSON
};

/*!
 * \brief Mapping between each ArmControlMode type and an identifying string.
 */
NLOHMANN_JSON_SERIALIZE_ENUM(ArmControlMode, { { kInvalid, nullptr },
                                               { kJointPosition, "joint position" },
                                               { kJointSpeed, "joint speed" } });

/*!
 * \brief Mapping between each Interpolation scheme and an identifying string.
 */
NLOHMANN_JSON_SERIALIZE_ENUM(InterpolationScheme, { { invalid_interpolation, nullptr },
                                                    { linear_interpolation, "linear interpolation" },
                                                    { cubic_interpolation, "cubic interpolation" },
                                                    { quintic_interpolation, "quintic interpolation" } });

/*!
 * \brief This ScaledMultiJointController class receives a trajectory and calculates the actual joint command
 * to publish each iteration. It slows down trajectory execution according to the actual speedscaling of the robot.
 */

class ScaledMultiJointController : public isaac::alice::Codelet
{
public:
  /*!
   * \brief Handles the setup functionality of this class. This includes collecting the kinematic tree node,
   * setting up memory according to number of joints and setting up message parsing parameters.
   */
  void start() override;

  /*!
   * \brief This functions is run cyclic with a fixed frequency. It checks whether a new trajectory message is available
   * and whether a new current arm state message is availabe and it calculates and publishes the actual joint command.
   */
  void tick() override;

  /*!
   * \brief Receives current state of the robot arm, this also includes the actual speedscaling.
   */
  ISAAC_PROTO_RX(CompositeProto, current_arm_state);

  /*!
   * \brief Receives a plan that contains a trajectory for each joint. If linear interpolation is configured
   * the trajectory must contain joint position. If cubic interpolation is configured the trajectory must contain joint
   * position and joint speeds. If quintic interpolation is configured the trajectory must contain joint positions,
   * joint speeds and joint accelerations.
   */
  ISAAC_PROTO_RX(CompositeProto, plan);

  /*!
   * \brief Publishes the actual joint command as joint position or joint speeds based upon control_mode.
   */
  ISAAC_PROTO_TX(CompositeProto, joint_command);

  /*!
   * \brief Publishes the status after a trajectory has been executed. The flag will be true if the joints are within
   * the tolerance after trajectory execution and false otherwise.
   */
  ISAAC_PROTO_TX(BooleanProto, trajectory_executed_succesfully);

  /*!
   * \brief Name of the node containing the map:KinematicTree component. This is used to obtain the names
   * of the joints for parsing/creating composite message.
   */
  ISAAC_PARAM(std::string, kinematic_tree);

  /*!
   * \brief Set the interpolation scheme used to calculate joint commands.
   */
  ISAAC_PARAM(InterpolationScheme, interpolation_scheme, quintic_interpolation);

  /*!
   * \brief Set the control mode.
   */
  ISAAC_PARAM(ArmControlMode, control_mode, kJointPosition);

  /*!
   * \brief Time after the last waypoint in the trajectory is published and until the joints should be within the
   * tolerance.
   */
  ISAAC_PARAM(double, goal_time, 0.0);

  /*!
   * \brief Tolerance for arrival. The trajectory is completed when the error for all joints is below this tolerance
   * after the last waypoint in the trajectory is published. If the tolerance is 0.0, no check will be made.
   */
  ISAAC_PARAM(double, tolerance, 0.0);

  /*!
   * \brief PID gains for each joint, used when control mode is kJointSpeed.
   */
  ISAAC_PARAM(nlohmann::json, gains, (nlohmann::json{ { "gains", {} } }));

private:
  /*!
   * \brief A state storing position, velocity and acceleration for each joint in the robot.
   *
   * \param size number of joints in the robot.
   */
  struct State
  {
    State()
    {
    }

    State(const unsigned int size) : position(size), velocity(size), acceleration(size)
    {
    }
    VectorXd position;
    VectorXd velocity;
    VectorXd acceleration;
  };

  /*!
   * \brief Request a schema for parsing arm state and trajectory, trajectory is parsed
   * based on interpolation scheme.
   */
  void initPlanParser();
  void initStateParser();

  /*!
   * \brief Parse received trajectory and received joint states.
   *
   * \returns true on succesful parse
   */
  bool parsePlan();
  bool parseJointStates();

  /*!
   * \brief Generate composite schema for publishing current command
   */
  void initCommandSchema();

  /*!
   * \brief Publish current command and trajectory status.
   *
   * \param trajectory_status the trajectory status after execution.
   */
  void publishCommand();
  void publishTrajectoryStatus(bool trajectory_status);

  /*!
   * \brief Calculate the current joint command.
   */
  void calculateCommand();

  /*!
   * \brief Test that we are inside the tolerance.
   *
   * \param error_pos Error between robot position and controller position.
   *
   * \returns true if inside tolerance.
   */
  bool insideTolerance(const double& error_pos);

  /*!
   * \brief Initialize a new trajectory for each joint in the robot.
   *
   * \param trajectory Recevied trajectory.
   */
  void initTrajectory(std::vector<Timeseries<VectorXd, double>> trajectory);

  // Current control period, the period is scaled according to the actual speedscaling.
  double period_;

  // Controller uptime, updated based on period.
  double controller_uptime_;

  // Current time of the tick
  double tick_time_;

  // Current speed fraction
  double speed_fraction_;

  // Current trajectory object for each joint
  std::vector<JointTrajectory> cur_traj_;

  // Controller, robot and error state.
  State controllerState_, robotState_, errorState_;

  // Parsers for arm state.
  composite::Parser position_state_parser_;
  composite::Parser velocity_state_parser_;
  composite::Parser speed_fraction_parser_;

  // Parser for trajectory for each joint.
  std::vector<composite::Parser> plan_parser_;

  // Bools feflecting if parsing message was succesful
  bool succesful_plan_parse_;
  bool succesful_arm_parse_;

  // Cache list of joint names used in the kinematic tree
  std::vector<std::string> joint_names_;

  // Number of joints
  unsigned int number_of_joints_;

  // Acquire time for most recently received plan and arm state
  std::optional<int64_t> plan_time_;
  std::optional<int64_t> arm_time_;

  // Cache composite schema for current joint command
  composite::Schema command_schema_;

  // Cache current control_mode and current interpolation scheme
  ArmControlMode control_mode_;
  InterpolationScheme interpolation_scheme_;

  // Used to store information about, when target position is reached
  std::vector<bool> successful_joints_;
  bool active_goal_;

  // Pid controller
  std::vector<Pid<double>> pid_;
  std::vector<double> ff_term_;
  bool active_pid_;
};

}  // namespace ur_controller
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ur_controller::ScaledMultiJointController)