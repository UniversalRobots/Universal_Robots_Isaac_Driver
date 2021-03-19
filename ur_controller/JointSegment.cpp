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

#include "JointSegment.hpp"
#include "engine/core/math/types.hpp"

namespace isaac
{
namespace ur_controller
{
namespace
{
constexpr int linearInterpolation = 1;
constexpr int cubicInterpolation = 2;
constexpr int quinticInterpolation = 3;
}  // namespace

JointSegment::JointSegment() : duration_(0.0), start_time_(0.0)
{
  coefficients_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

JointSegment::JointSegment(const Timeseries<VectorXd, double>::Entry& start_waypoint,
                           const Timeseries<VectorXd, double>::Entry& end_waypoint)
{
  coefficients_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  initSegment(start_waypoint, end_waypoint);
}

void JointSegment::initSegment(const Timeseries<VectorXd, double>::Entry& start_waypoint,
                               const Timeseries<VectorXd, double>::Entry& end_waypoint)
{
  ASSERT(start_waypoint.stamp <= end_waypoint.stamp, "start_waypoint stamp shouldn't be larger than end_waypoint "
                                                     "stamp");
  ASSERT(start_waypoint.state.rows() >= linearInterpolation && end_waypoint.state.rows() >= linearInterpolation, "empty"
                                                                                                                 " stat"
                                                                                                                 "e");
  ASSERT(start_waypoint.state.rows() == end_waypoint.state.rows(), "start_waypoint and end_waypoint states size "
                                                                   "mismatch");

  // Set all coefficients to zero
  coefficients_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  duration_ = end_waypoint.stamp - start_waypoint.stamp;
  start_time_ = start_waypoint.stamp;

  if (start_waypoint.state.rows() == linearInterpolation)
  {
    linearCoefficients(start_waypoint.state, end_waypoint.state, duration_);
  }
  else if (start_waypoint.state.rows() == cubicInterpolation)
  {
    cubicCoefficients(start_waypoint.state, end_waypoint.state, duration_);
  }
  else if (start_waypoint.state.rows() == quinticInterpolation)
  {
    quinticCoefficients(start_waypoint.state, end_waypoint.state, duration_);
  }
  else
  {
    PANIC("To many elements in the state vector");
  }
}

Vector6d JointSegment::getCoefficients()
{
  return coefficients_;
}

const double JointSegment::getEndTime()
{
  return start_time_ + duration_;
}

const double JointSegment::getStartTime()
{
  return start_time_;
}

void JointSegment::interpolate(const double& stamp, double& pos, double& vel, double& acc)
{
  // We can only interpolate within the waypoints.
  ASSERT(start_time_ <= stamp && stamp <= start_time_ + duration_, "Time stamp is not within the boundaries.");

  const double x = stamp - start_time_;

  pos = ((((coefficients_[5] * x + coefficients_[4]) * x + coefficients_[3]) * x + coefficients_[2]) * x +
         coefficients_[1]) * x + coefficients_[0];
  vel = (((5.0 * coefficients_[5] * x + 4.0 * coefficients_[4]) * x + 3.0 * coefficients_[3]) * x +
         2.0 * coefficients_[2]) * x + coefficients_[1];
  acc = ((20.0 * coefficients_[5] * x + 12.0 * coefficients_[4]) * x + 6.0 * coefficients_[3]) * x +
        2.0 * coefficients_[2];
}

void JointSegment::linearCoefficients(const VectorXd& start, const VectorXd& end, const double& x)
{
  if (x == 0.0)
  {
    coefficients_[0] = start[0];
  }
  else
  {
    coefficients_[0] = start[0];
    coefficients_[1] = (end[0] - start[0]) / x;
  }
}

void JointSegment::cubicCoefficients(const VectorXd& start, const VectorXd& end, const double& x)
{
  if (x == 0.0)
  {
    coefficients_[0] = start[0];
    coefficients_[1] = start[1];
  }
  else
  {
    Vector4d powers;
    powers << 1.0, x, x * x, x * x * x;
    coefficients_[0] = start[0];
    coefficients_[1] = start[1];
    coefficients_[2] = (3.0 * (-start[0] + end[0]) + (-2.0 * start[1] - end[1]) * powers[1]) / powers[2];
    coefficients_[3] = (2.0 * (start[0] - end[0]) + (start[1] + end[1]) * powers[1]) / powers[3];
  }
}

void JointSegment::quinticCoefficients(const VectorXd& start, const VectorXd& end, const double& x)
{
  if (x == 0.0)
  {
    coefficients_[0] = start[0];
    coefficients_[1] = start[1];
    coefficients_[2] = 0.5 * start[2];
  }
  else
  {
    Vector6d powers;
    powers << 1.0, x, x * x, x * x * x, x * x * x * x, x * x * x * x * x;
    coefficients_[0] = start[0];
    coefficients_[1] = start[1];
    coefficients_[2] = 0.5 * start[2];
    coefficients_[3] = (20.0 * (-start[0] + end[0]) + (-12.0 * start[1] - 8.0 * end[1]) * powers[1] +
                       (-3.0 * start[2] + end[2]) * powers[2]) / (2.0 * powers[3]);
    coefficients_[4] = (30.0 * (start[0] - end[0]) + (16.0 * start[1] + 14.0 * end[1]) * powers[1] +
                       (3.0 * start[2] - 2.0 * end[2]) * powers[2]) / (2.0 * powers[4]);
    coefficients_[5] = (12.0 * (-start[0] + end[0]) + 6.0 * (-start[1] - end[1]) * powers[1] +
                       (-start[2] + end[2]) * powers[2]) / (2.0 * powers[5]);
  }
}

}  // namespace ur_controller
}  // namespace isaac