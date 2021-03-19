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

#include "packages/universal_robots/ur_controller/Pid.hpp"
#include "gtest/gtest.h"

namespace isaac
{
namespace ur_controller
{
TEST(Pid, BadDurationTest)
{
  Pid<double> pid;
  pid.initPid(1.0, 0.0, 0.0, 1.0, -1.0);

  // When duration time is zero or less than zero we expect the output to be zero.
  EXPECT_EQ(pid.calculate(1.0, 0.0), 0.0);
  EXPECT_EQ(pid.calculate(1.0, -1.0), 0.0);
  EXPECT_EQ(pid.calculate(1.0, 1.0, -1.0), 0.0);
  EXPECT_EQ(pid.calculate(1.0, 1.0, 0.0), 0.0);
}

TEST(Pid, ResetErrorsTest)
{
  Pid<double> pid;
  pid.initPid(1.0, 0.1, 0.1, 1.0, -1.0);
  pid.calculate(1, 0.1, 0.1);

  double last_error, i_error;
  pid.reset();
  pid.getErrors(last_error, i_error);

  EXPECT_EQ(last_error, 0);
  EXPECT_EQ(i_error, 0);
}

TEST(Pid, InitTest)
{
  Pid<double> pid;
  // Empty JSON object
  nlohmann::json json = { { "gains", {} } };

  // Init should return false, when there is no propertional gain
  EXPECT_FALSE(pid.init(json));

  double p_gain = 5.0;
  double i_gain = 0.1;
  double d_gain = 0.01;
  double i_clamp = 1;

  // JSON
  json = { { "p", p_gain }, { "i", i_gain }, { "d", d_gain }, { "i_clamp", i_clamp } };

  // Init should now return true
  EXPECT_TRUE(pid.init(json));

  double p_return, i_return, d_return, i_max_return, i_min_return;
  pid.getGains(p_return, i_return, d_return, i_max_return, i_min_return);

  EXPECT_EQ(p_gain, p_return);
  EXPECT_EQ(i_gain, i_return);
  EXPECT_EQ(d_gain, d_return);
  EXPECT_EQ(std::abs(i_clamp), i_max_return);
  EXPECT_EQ(-std::abs(i_clamp), i_min_return);
}

TEST(Pid, PGainTest)
{
  Pid<double> pid;
  pid.initPid(1.0, 0.0, 0.0, 0.0, 0.0);

  double cmd = pid.calculate(-0.6, 1);
  EXPECT_FLOAT_EQ(cmd, -0.6);

  cmd = pid.calculate(-0.2, 1);
  EXPECT_FLOAT_EQ(cmd, -0.2);

  cmd = pid.calculate(0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.2);

  cmd = pid.calculate(0.8, 1);
  EXPECT_FLOAT_EQ(cmd, 0.8);

  cmd = pid.calculate(0.0, 1);
  EXPECT_FLOAT_EQ(cmd, 0.0);
}

TEST(Pid, IGainTest)
{
  Pid<double> pid;
  pid.initPid(0.0, 1.0, 0.0, 10.0, -10.0);

  double cmd = pid.calculate(-0.6, 1);
  EXPECT_FLOAT_EQ(cmd, -0.6);

  cmd = pid.calculate(-0.2, 1);
  EXPECT_FLOAT_EQ(cmd, -0.8);

  cmd = pid.calculate(0.0, 1);
  EXPECT_FLOAT_EQ(cmd, -0.8);

  pid.reset();

  cmd = pid.calculate(0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.2);

  cmd = pid.calculate(0.8, 1);
  EXPECT_FLOAT_EQ(cmd, 1.0);

  cmd = pid.calculate(0.0, 1);
  EXPECT_FLOAT_EQ(cmd, 1.0);
}

TEST(Pid, DGainTest)
{
  Pid<double> pid;
  pid.initPid(0.0, 0.0, 1.0, 0.0, 0.0);

  double cmd = pid.calculate(-0.6, 1);
  EXPECT_FLOAT_EQ(cmd, -0.6);

  cmd = pid.calculate(-0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.4);

  cmd = pid.calculate(-0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.0);

  pid.reset();

  cmd = pid.calculate(0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.2);

  cmd = pid.calculate(0.8, 1);
  EXPECT_FLOAT_EQ(cmd, 0.6);

  cmd = pid.calculate(0.0, 1);
  EXPECT_FLOAT_EQ(cmd, -0.8);
}

TEST(Pid, CompleteGainTest)
{
  Pid<double> pid;
  pid.initPid(1.0, 1.0, 1.0, 10.0, -10.0);

  double cmd = pid.calculate(0.5, 1);
  EXPECT_FLOAT_EQ(cmd, 1.5);

  cmd = pid.calculate(0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.6);

  cmd = pid.calculate(0.0, 1);
  EXPECT_FLOAT_EQ(cmd, 0.5);

  cmd = pid.calculate(-0.2, 1);
  EXPECT_FLOAT_EQ(cmd, 0.1);

  cmd = pid.calculate(-0.5, 1);
  EXPECT_FLOAT_EQ(cmd, -0.8);
}

}  // namespace ur_controller
}  // namespace isaac
