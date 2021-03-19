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

#include "packages/universal_robots/ur_controller/JointTrajectory.hpp"
#include "gtest/gtest.h"

namespace isaac
{
namespace ur_controller
{
double EPS = 1e-9;

TEST(JointTrajectory, EmptyTrajectoryTest)
{
  Timeseries<VectorXd, double> trajectory;
  JointTrajectory joint_traj;

  EXPECT_DEATH(joint_traj.initTrajectory(trajectory), "The trajectory shouldn't be empty");
}

TEST(JointTrajectory, EndTimeTest)
{
  // Test that end time equals last entry in the plan
  Timeseries<VectorXd, double> trajectory;
  VectorXd state(3);
  state << 1, 2, 3;
  trajectory.tryPush(1.0, state);
  trajectory.tryPush(2.0, state);
  trajectory.tryPush(3.0, state);
  trajectory.tryPush(4.0, state);

  JointTrajectory joint_traj;

  joint_traj.initTrajectory(trajectory);
  EXPECT_EQ(joint_traj.endTime(), 4.0);
}

TEST(JointTrajectory, LinearInterpolationTest)
{
  // Linear function y = 1 + x
  auto pos = [](double x) { return (1 + x); };

  Timeseries<VectorXd, double> trajectory;

  double start_time = 1.0;
  double end_time = 2.0;

  VectorXd start_state(1);
  VectorXd end_state(1);
  start_state << 1.0;
  end_state << 2.0;

  trajectory.tryPush(start_time, start_state);
  trajectory.tryPush(end_time, end_state);

  JointTrajectory joint_traj;

  joint_traj.initTrajectory(trajectory);

  double position, velocity, acceleration;

  const double duration = end_time - start_time;
  const double vel = (end_state[0] - start_state[0]) / duration;

  // Interpolate before trajectory start
  joint_traj.interpolate(start_time - 0.5, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(start_state[0], position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate at trajectory start
  joint_traj.interpolate(start_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(0.0), position);
  EXPECT_FLOAT_EQ(vel, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate inside trajectory
  double time = duration * ((double)rand() / (RAND_MAX));
  joint_traj.interpolate(start_time + time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(time), position);
  EXPECT_FLOAT_EQ(vel, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate at trajectory end
  joint_traj.interpolate(end_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(duration), position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate after trajectory end
  joint_traj.interpolate(end_time + 0.5, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(duration), position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);
}

TEST(JointTrajectory, CubicInterpolationTest)
{
  // Cubic function y = -2.511 + 0.1x - 3.789x^2 + 2.689x^3 for the only segment in the trajectory
  auto pos = [](double x) { return -2.511 + 0.1 * x - 3.789 * x * x + 2.689 * x * x * x; };
  auto vel = [](double x) { return 0.1 - 2.0 * 3.789 * x + 3.0 * 2.689 * x * x; };
  auto acc = [](double x) { return 2.0 * -3.789 + 6.0 * 2.689 * x; };

  Timeseries<VectorXd, double> trajectory;

  double start_time = 1.0;
  double end_time = 2.0;

  VectorXd start_state(2);
  VectorXd end_state(2);
  start_state << -2.511, 0.1;
  end_state << -3.511, 0.589;

  trajectory.tryPush(start_time, start_state);
  trajectory.tryPush(end_time, end_state);

  JointTrajectory joint_traj;

  joint_traj.initTrajectory(trajectory);

  double position, velocity, acceleration;
  const double duration = end_time - start_time;

  // Interpolate before trajectory start
  joint_traj.interpolate(start_time - 0.5, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(start_state[0], position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate at trajectory start
  joint_traj.interpolate(start_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(0.0), position);
  EXPECT_FLOAT_EQ(vel(0.0), velocity);
  EXPECT_FLOAT_EQ(acc(0.0), acceleration);

  // Interpolate inside trajectory
  double time = duration * ((double)rand() / (RAND_MAX));
  joint_traj.interpolate(start_time + time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(time), position);
  EXPECT_FLOAT_EQ(vel(time), velocity);
  EXPECT_FLOAT_EQ(acc(time), acceleration);

  // Interpolate at trajectory end
  joint_traj.interpolate(end_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(duration), position);
  EXPECT_FLOAT_EQ(vel(duration), velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate after trajectory end
  joint_traj.interpolate(end_time + 0.5, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(duration), position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);
}

TEST(JointTrajectory, QuinticInterpolationTest)
{
  // Quintic function  y = -2.511 + 0.1x + 0.05x^2 - 12.6775x^3 + 19.216x^4 - 7.6885x^5
  auto pos = [](double x) {
    return -2.511 + 0.1 * x + 0.05 * x * x - 12.6775 * x * x * x + 19.216 * x * x * x * x - 7.6885 * x * x * x * x * x;
  };
  auto vel = [](double x) {
    return 0.1 + 2.0 * 0.05 * x + 3.0 * -12.6775 * x * x + 4.0 * 19.216 * x * x * x + 5.0 * -7.6885 * x * x * x * x;
  };
  auto acc = [](double x) {
    return 2.0 * 0.05 + 6.0 * -12.6775 * x + 12.0 * 19.216 * x * x + 20.0 * -7.6885 * x * x * x;
  };

  Timeseries<VectorXd, double> trajectory;

  double start_time = 1.0;
  double end_time = 2.0;

  VectorXd start_state(3);
  VectorXd end_state(3);
  start_state << -2.511, 0.1, 0.1;
  end_state << -3.511, 0.589, 0.857;

  trajectory.tryPush(start_time, start_state);
  trajectory.tryPush(end_time, end_state);

  JointTrajectory joint_traj;

  joint_traj.initTrajectory(trajectory);

  double position, velocity, acceleration;
  const double duration = end_time - start_time;

  // Interpolate before trajectory start
  joint_traj.interpolate(start_time - 0.5, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(start_state[0], position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate at trajectory start
  joint_traj.interpolate(start_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(0.0), position);
  EXPECT_FLOAT_EQ(vel(0.0), velocity);
  EXPECT_FLOAT_EQ(acc(0.0), acceleration);

  // Interpolate inside trajectory
  double time = duration * ((double)rand() / (RAND_MAX));
  joint_traj.interpolate(start_time + time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(time), position);
  EXPECT_FLOAT_EQ(vel(time), velocity);
  EXPECT_FLOAT_EQ(acc(time), acceleration);

  // Interpolate at trajectory end
  joint_traj.interpolate(end_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(duration), position);
  EXPECT_FLOAT_EQ(vel(duration), velocity);
  EXPECT_FLOAT_EQ(acc(duration), acceleration);

  // Interpolate after trajectory end
  joint_traj.interpolate(end_time + 0.5, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(duration), position);
  EXPECT_FLOAT_EQ(0.0, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);
}

}  // namespace ur_controller
}  // namespace isaac
