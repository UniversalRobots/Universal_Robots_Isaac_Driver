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

#include "JointTrajectory.hpp"

namespace isaac
{
namespace ur_controller
{
JointTrajectory::JointTrajectory() = default;

void JointTrajectory::initTrajectory(const Timeseries<VectorXd, double>& trajectory)
{
  ASSERT(trajectory.empty() == false, "The trajectory shouldn't be empty");
  joint_trajectory_ = trajectory;

  Timeseries<VectorXd, double>::Entry waypoint;
  Timeseries<VectorXd, double>::Entry next_waypoint;
  // Create a vector of segments
  segments_.clear();
  for (unsigned int j = 0; j < trajectory.size() - 1; ++j)
  {
    waypoint = trajectory.at(j);
    next_waypoint = trajectory.at(j + 1);
    JointSegment segment(waypoint, next_waypoint);
    segments_.push_back(segment);
  }
  // Last segment is the final position of the trajectory, but with a duration of 0.
  waypoint = joint_trajectory_.youngest();
  next_waypoint = joint_trajectory_.youngest();
  JointSegment segment(waypoint, next_waypoint);
  segments_.push_back(segment);
}

void JointTrajectory::interpolate(const double& stamp, double& pos, double& vel, double& acc)
{
  // If we are before trajectory start time or after trajectory end time return position
  if (stamp < joint_trajectory_.oldest().stamp)
  {
    pos = joint_trajectory_.oldest().state[0];
    vel = 0.0;
    acc = 0.0;
    return;
  }
  else if (stamp > joint_trajectory_.youngest().stamp)
  {
    pos = joint_trajectory_.youngest().state[0];
    vel = 0.0;
    acc = 0.0;
    return;
  }

  // Find current segmet and interpolate
  ssize_t index = joint_trajectory_.lower_index(stamp);
  segments_[index].interpolate(stamp, pos, vel, acc);
}

const double JointTrajectory::endTime()
{
  return joint_trajectory_.youngest().stamp;
}

}  // namespace ur_controller
}  // namespace isaac