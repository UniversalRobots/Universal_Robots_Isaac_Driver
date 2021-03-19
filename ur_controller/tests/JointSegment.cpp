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

#include "packages/universal_robots/ur_controller/JointSegment.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac
{
namespace ur_controller
{
double EPS = 1e-9;

TEST(JointSegment, InvalidJointSegmentConstructionTest)
{
  Timeseries<VectorXd, double>::Entry start_entry, end_entry;

  // Entry stamp mismatch
  start_entry.stamp = 2.0;
  end_entry.stamp = 1.0;
  EXPECT_DEATH(JointSegment(start_entry, end_entry), "start_waypoint stamp shouldn't be larger than end_waypoint "
                                                     "stamp");

  // Empty entry state
  start_entry.stamp = 1.0;
  end_entry.stamp = 2.0;
  EXPECT_DEATH(JointSegment(start_entry, end_entry), "empty state");

  // Entry state size mismatch
  VectorXd start_state(3);
  VectorXd end_state(2);
  start_state[0] = 1;
  start_state[1] = 1;
  start_state[2] = 1;
  end_state[0] = 1;
  end_state[1] = 1;
  start_entry.state = start_state;
  end_entry.state = end_state;
  EXPECT_DEATH(JointSegment(start_entry, end_entry), "start_waypoint and end_waypoint states size mismatch");
}

TEST(JointSegment, InterpolationOutsideSegment)
{
  Timeseries<VectorXd, double>::Entry start_entry, end_entry;

  double start_time = 1.0;
  double end_time = 2.0;

  VectorXd start_pos(1);
  VectorXd end_pos(1);
  start_pos << 1.0;
  end_pos << 2.0;

  start_entry.stamp = start_time;
  end_entry.stamp = end_time;
  start_entry.state = start_pos;
  end_entry.state = end_pos;

  JointSegment segment(start_entry, end_entry);
  double position, velocity, acceleration;

  // Interpolate before start time
  EXPECT_DEATH(segment.interpolate(0.5, position, velocity, acceleration), "Time stamp is not within the boundaries.");

  // Interpolate after end_time
  EXPECT_DEATH(segment.interpolate(2.5, position, velocity, acceleration), "Time stamp is not within the boundaries.");
}

TEST(JointSegment, LinearSegmentConstructionTest)
{
  double start_time = 1.0;
  double end_time = 2.0;

  // Test linear segment
  Timeseries<VectorXd, double>::Entry linear_start_entry, linear_end_entry;
  Vector6d linear_coefs, expected_linear_coefs;

  linear_start_entry.stamp = start_time;
  linear_end_entry.stamp = end_time;

  VectorXd linear_start_pos(1);
  VectorXd linear_end_pos(1);
  linear_start_pos << 1.0;
  linear_end_pos << 2.0;
  linear_start_entry.state = linear_start_pos;
  linear_end_entry.state = linear_end_pos;

  // Constructing linear segment of y=coef[0]+coef[1]x
  JointSegment linear_segment(linear_start_entry, linear_end_entry);

  // Expected linear segment y = 1 + 1x
  // coef[2] = coef[3] = coef[4] = coef[5] = 0
  expected_linear_coefs[0] = 1.0;
  expected_linear_coefs[1] = 1.0;
  expected_linear_coefs[2] = 0.0;
  expected_linear_coefs[3] = 0.0;
  expected_linear_coefs[4] = 0.0;
  expected_linear_coefs[5] = 0.0;

  // Test that coefficients match
  linear_coefs = linear_segment.getCoefficients();
  ISAAC_ASSERT_VEC_NEAR(linear_coefs, expected_linear_coefs, EPS);
}

TEST(JointSegment, CubicSegmentConstructionTest)
{
  // Test cubic segment
  double start_time = 1.0;
  double end_time = 2.0;
  Timeseries<VectorXd, double>::Entry cubic_start_entry, cubic_end_entry;
  Vector6d cubic_coefs, expected_cubic_coefs;

  cubic_start_entry.stamp = start_time;
  cubic_end_entry.stamp = end_time;

  VectorXd cubic_start_state(2);
  VectorXd cubic_end_state(2);
  cubic_start_state << -2.511, 0.1;
  cubic_end_state << -3.511, 0.589;
  cubic_start_entry.state = cubic_start_state;
  cubic_end_entry.state = cubic_end_state;

  // Constructing cubic segment of y=coef[0]+coef[1]x+coef[2]*x^2+coef[3]x^3
  JointSegment cubic_segment(cubic_start_entry, cubic_end_entry);

  // Expected cubic segment y = -2.511 + 0.1x + -3.789x^2 + 2.689x^3
  // coef[4] = coef[5] = 0
  expected_cubic_coefs[0] = -2.511;
  expected_cubic_coefs[1] = 0.1;
  expected_cubic_coefs[2] = -3.789;
  expected_cubic_coefs[3] = 2.689;
  expected_cubic_coefs[4] = 0.0;
  expected_cubic_coefs[5] = 0.0;

  // Test that coefficients match
  cubic_coefs = cubic_segment.getCoefficients();
  ISAAC_ASSERT_VEC_NEAR(cubic_coefs, expected_cubic_coefs, EPS);
}

TEST(JointSegment, QuinticSegmentConstructionTest)
{
  // Test quintic segment
  double start_time = 1.0;
  double end_time = 2.0;
  Timeseries<VectorXd, double>::Entry quintic_start_entry, quintic_end_entry;
  Vector6d quintic_coefs, expected_quintic_coefs;

  quintic_start_entry.stamp = start_time;
  quintic_end_entry.stamp = end_time;

  VectorXd quintic_start_state(3);
  VectorXd quintic_end_state(3);
  quintic_start_state << -2.511, 0.1, 0.1;
  quintic_end_state << -3.511, 0.589, 0.857;
  quintic_start_entry.state = quintic_start_state;
  quintic_end_entry.state = quintic_end_state;

  // Constructing quintic segment of y = coef[0] + coef[1]x + coef[2]*x^2 + coef[3]x^3 + coef[4]x^4 + coef[5]x^5
  JointSegment quintic_segment(quintic_start_entry, quintic_end_entry);

  // expected quintic segment y = -2.511 + 0.1x + 0.05x^2 - 12.6775x^3 + 19.216x^4 - 7.6885x^5
  expected_quintic_coefs[0] = -2.511;
  expected_quintic_coefs[1] = 0.1;
  expected_quintic_coefs[2] = 0.05;
  expected_quintic_coefs[3] = -12.6775;
  expected_quintic_coefs[4] = 19.216;
  expected_quintic_coefs[5] = -7.6885;

  // Test that coefficients match
  quintic_coefs = quintic_segment.getCoefficients();
  ISAAC_ASSERT_VEC_NEAR(quintic_coefs, expected_quintic_coefs, EPS);
}

TEST(JointSegment, LinearInterpolationTest)
{
  // Linear function y = 1 + x
  auto pos = [](double x) { return (1 + x); };
  Timeseries<VectorXd, double>::Entry start_entry, end_entry;
  double start_time = 1.0;
  double end_time = 2.0;

  start_entry.stamp = start_time;
  end_entry.stamp = end_time;

  VectorXd start_state(1);
  VectorXd end_state(1);
  start_state << 1.0;
  end_state << 2.0;
  start_entry.state = start_state;
  end_entry.state = end_state;

  JointSegment linearSegment(start_entry, end_entry);

  const double duration = end_time - start_time;
  const double expected_vel = (end_state[0] - start_state[0]) / duration;
  double position, velocity, acceleration;

  // Interpolate at segment start
  linearSegment.interpolate(start_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(start_state[0], position);
  EXPECT_FLOAT_EQ(expected_vel, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate inside segment
  double time = duration * ((double)rand() / (RAND_MAX));
  linearSegment.interpolate(start_time + time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(time), position);
  EXPECT_FLOAT_EQ(expected_vel, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);

  // Interpolate at the end segment
  linearSegment.interpolate(end_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(end_state[0], position);
  EXPECT_FLOAT_EQ(expected_vel, velocity);
  EXPECT_FLOAT_EQ(0.0, acceleration);
}

TEST(JointSegment, CubicInterpolationTest)
{
  // Cubic function y = -2.511 + 0.1x - 3.789x^2 + 2.689x^3
  auto pos = [](double x) { return -2.511 + 0.1 * x - 3.789 * x * x + 2.689 * x * x * x; };
  auto vel = [](double x) { return 0.1 - 2.0 * 3.789 * x + 3.0 * 2.689 * x * x; };
  auto acc = [](double x) { return 2.0 * -3.789 + 6.0 * 2.689 * x; };

  Timeseries<VectorXd, double>::Entry start_entry, end_entry;
  double start_time = 1.0;
  double end_time = 2.0;

  start_entry.stamp = start_time;
  end_entry.stamp = end_time;

  VectorXd start_state(2);
  VectorXd end_state(2);
  start_state << -2.511, 0.1;
  end_state << -3.511, 0.589;
  start_entry.state = start_state;
  end_entry.state = end_state;

  JointSegment cubicSegment(start_entry, end_entry);

  double duration = end_time - start_time;
  double position, velocity, acceleration;

  // interpolate at segment start
  cubicSegment.interpolate(start_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(start_state[0], position);
  EXPECT_FLOAT_EQ(vel(0.0), velocity);
  EXPECT_FLOAT_EQ(acc(0.0), acceleration);

  // interpolate inside segment
  double time = duration * ((double)rand() / (RAND_MAX));
  cubicSegment.interpolate(start_time + time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(time), position);
  EXPECT_FLOAT_EQ(vel(time), velocity);
  EXPECT_FLOAT_EQ(acc(time), acceleration);

  // interpolate at end segment
  cubicSegment.interpolate(end_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(end_state[0], position);
  EXPECT_FLOAT_EQ(end_state[1], velocity);
  EXPECT_FLOAT_EQ(acc(duration), acceleration);
}

TEST(JointSegment, QuinticInterpolationTest)
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

  Timeseries<VectorXd, double>::Entry start_entry, end_entry;
  double start_time = 1.0;
  double end_time = 2.0;

  start_entry.stamp = start_time;
  end_entry.stamp = end_time;

  VectorXd start_state(3);
  VectorXd end_state(3);
  start_state << -2.511, 0.1, 0.1;
  end_state << -3.511, 0.589, 0.857;
  start_entry.state = start_state;
  end_entry.state = end_state;

  JointSegment quinticSegment(start_entry, end_entry);

  double duration = end_time - start_time;
  double position, velocity, acceleration;

  // interpolate at segment start
  quinticSegment.interpolate(start_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(start_state[0], position);
  EXPECT_FLOAT_EQ(start_state[1], velocity);
  EXPECT_FLOAT_EQ(start_state[2], acceleration);

  // interpolate inside segment
  double time = duration * ((double)rand() / (RAND_MAX));
  quinticSegment.interpolate(start_time + time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(pos(time), position);
  EXPECT_FLOAT_EQ(vel(time), velocity);
  EXPECT_FLOAT_EQ(acc(time), acceleration);

  // interpolate at end segment
  quinticSegment.interpolate(end_time, position, velocity, acceleration);
  EXPECT_FLOAT_EQ(end_state[0], position);
  EXPECT_FLOAT_EQ(end_state[1], velocity);
  EXPECT_FLOAT_EQ(end_state[2], acceleration);
}

}  // namespace ur_controller
}  // namespace isaac