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

#include "engine/core/math/types.hpp"
#include "engine/gems/algorithm/timeseries.hpp"

namespace isaac
{
namespace ur_controller
{
/*!
 * \brief Calculates a segment between two waypoints. The segment is a function describing
 * joint position as a function of time. The function can be linear, cubic or quintic.
 */

class JointSegment
{
public:
  /*!
   * \brief Construct a new Joint Segment object
   */
  JointSegment();

  /*!
   * \brief Construct a new Joint Segment object with a start_waypoint and end_waypoint.
   * This constructor calls initSegment.
   *
   * Timeseries<VectorXd, double>::Entry is a struct describing a waypoint.
   * Struct Entry {
   *   double Stamp
   *   VectorXd state
   * };
   * An Entry consist of a timestamp and a state which is the current state at that timestamp.
   * State is vector with 1, 2 or 3 entries depending on wheter the vector consist of pos, vel and acc.
   * This function calls InitSegment.
   *
   * \param start_waypoint the first waypoint
   * \param end_waypoint the second waypoint
   */
  JointSegment(const Timeseries<VectorXd, double>::Entry& start_waypoint,
               const Timeseries<VectorXd, double>::Entry& end_waypoint);

  /*!
   * \brief Calculates the function between the two waypoints.
   *
   * If State has length 1 linear function will be calculated.
   * y = coffiecients_[1]*x + coffiecients_[0]
   * coffiecients_[2] = coffiecients_[3] = coffiecients_[4] = coffiecients_[5] = 0.
   *
   * If State has length 2 cubic function will be calculated.
   * y = coffiecients_[3]*x^3 + coffiecients_[2]*x^2 + coffiecients_[1]*x + coffiecients_[0]
   * coffiecients_[4] = coffiecients_[5] = 0.
   *
   * If State has length 3 quintic function will be calculated.
   * y = coffiecients_[5]*x^5 + coffiecients_[4]*x^4 + coffiecients_[3]*x^3 + coffiecients_[2]*x^2 + coffiecients_[1]*x
   * + coffiecients_[0]
   *
   * \param start_waypoint the first waypoint
   * \param end_waypoint the second waypoint
   */
  void initSegment(const Timeseries<VectorXd, double>::Entry& start_waypoint,
                   const Timeseries<VectorXd, double>::Entry& end_waypoint);

  /*!
   * \returns Segment coefficients
   */
  Vector6d getCoefficients();

  /*!
   * \returns End time of the segment
   */
  const double getEndTime();

  /*!
   * \returns Start time of the segment
   */
  const double getStartTime();

  /*!
   * \brief interpolate at a given time stamp.
   *
   * \param stamp Time stamp to do the interpolation, make sure it is within the boundaries.
   * \param pos Variable to store calculated position
   * \param vel Variable to store calculated velocity
   * \param acc Variable to store calculated acceleration
   */
  void interpolate(const double& stamp, double& pos, double& vel, double& acc);

private:
  /*!
   * \brief Calculates linear coefficients for the segment.
   *
   * \param start Start state
   * \param end End state
   * \param x duration of the segment
   */
  void linearCoefficients(const VectorXd& start, const VectorXd& end, const double& x);

  /*!
   * \brief Calculates cubic coefficients for the segment.
   *
   * \param start Start state
   * \param end End state
   * \param x duration of the segment
   */
  void cubicCoefficients(const VectorXd& start, const VectorXd& end, const double& x);

  /*!
   * \brief Calculates quintic coefficients for the segment.
   *
   * \param start Start state
   * \param end End state
   * \param x duration of the segment
   */
  void quinticCoefficients(const VectorXd& start, const VectorXd& end, const double& x);

  // Coefficients
  Vector6d coefficients_;

  // Segment start time, end time and duration for a segment
  double duration_, start_time_;
};

}  // namespace ur_controller
}  // namespace isaac
