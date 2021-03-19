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

#include <limits>
#include "engine/core/math/utils.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac
{
namespace ur_controller
{
/*!
 * \brief Pid class used to calculate the control output. This class is used in the
 * ScaledMultiJointController to calculate joint speeds.
 */

template <typename Scalar>
class Pid
{
public:
  /*!
   * \brief Construct a new Pid object
   */
  Pid() : last_error_(static_cast<Scalar>(0.0)), i_error_(static_cast<Scalar>(0.0))
  {
    setGains(static_cast<Scalar>(0.0), static_cast<Scalar>(0.0), static_cast<Scalar>(0.0),
             std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::min());
  }

  /*!
   * \brief Initialize the PID controller with gains.
   *
   * \param p P gain
   * \param i I gain
   * \param d D gain
   * \param i_max Maximum integral error
   * \param i_min Minimum integral error
   */
  void initPid(const Scalar& p, const Scalar& i, const Scalar& d, const Scalar& i_max, const Scalar& i_min)
  {
    setGains(p, i, d, i_max, i_min);
    reset();
  }

  /*!
   * \brief Initialize the PID controller with gains.
   *
   * \param json Json object storing the gains.
   *
   * \returns true on succes
   */
  bool init(const nlohmann::json& json)
  {
    const auto p = serialization::TryGetFromMap<Scalar>(json, "p");
    if (p == std::nullopt)
    {
      return false;
    }
    Scalar i = serialization::GetFromMapOrDefault<Scalar>(json, "i", 0.0);
    Scalar d = serialization::GetFromMapOrDefault<Scalar>(json, "d", 0.0);
    Scalar i_clamp = serialization::GetFromMapOrDefault<Scalar>(json, "i_clamp", std::numeric_limits<Scalar>::max());
    Scalar i_max = std::abs(i_clamp);
    Scalar i_min = -std::abs(i_clamp);

    setGains(*p, i, d, i_max, i_min);
    reset();

    return true;
  }

  /*!
   * \brief Get the gains
   *
   * \param p P gain
   * \param i I gain
   * \param d D gain
   * \param i_max Integral max error
   * \param i_min Integral minimum error
   */
  void getGains(Scalar& p, Scalar& i, Scalar& d, Scalar& i_max, Scalar& i_min)
  {
    p = p_gain_;
    i = i_gain_;
    d = d_gain_;
    i_max = i_max_;
    i_min = i_min_;
  }

  /*!
   * \brief Set the gains
   *
   * \param p P gain
   * \param i I gain
   * \param d D gain
   * \param i_max Integral max error
   * \param i_min Integral minimum error
   */
  void setGains(const Scalar& p, const Scalar& i, const Scalar& d, const Scalar& i_max, const Scalar& i_min)
  {
    p_gain_ = p;
    i_gain_ = i;
    d_gain_ = d;
    i_max_ = i_max;
    i_min_ = i_min;
  }

  /*!
   * \brief Get the errors
   *
   * \param last_error Lastest error
   * \param i_error Integral error
   */
  void getErrors(Scalar& last_error, Scalar& i_error)
  {
    i_error = i_error_;
    last_error = last_error_;
  }

  /*!
   * \brief Reset errors
   */
  void reset()
  {
    last_error_ = 0.0;
    i_error_ = 0.0;
  }

  /*!
   * \brief Calculates control output.
   * This functions calculates the derivative error, before calculating control output.
   *
   * \param error Error between target and actual value.
   * \param dt Time since last control output was calculated.
   *
   * \returns control output
   */
  Scalar calculate(Scalar error, Scalar dt)
  {
    if (dt <= 0.0)
    {
      return 0.0;
    }

    Scalar error_dot = (error - last_error_) / dt;
    last_error_ = error;
    return calculate(error, error_dot, dt);
  }

  /*!
   * \brief Calculates control output.
   *
   * \param error Error between target and actual value.
   * \param error_dot Derivative error.
   * \param dt Time since last control output was calculated.
   *
   * \returns control output
   */
  Scalar calculate(Scalar error, Scalar error_dot, Scalar dt)
  {
    if (dt <= 0.0)
    {
      return 0.0;
    }

    Scalar p_term, i_term, d_term, cmd;

    // Calculate p term
    p_term = p_gain_ * error;

    // Calculate I term
    i_error_ += error * dt;
    i_term = i_error_ * i_gain_;

    // Clamp I term
    i_term = Clamp(i_term, i_min_, i_max_);

    // Calculate derivative term
    d_term = d_gain_ * error_dot;

    cmd = p_term + i_term + d_term;

    last_error_ = error;

    return cmd;
  }

private:
  Scalar last_error_;
  Scalar i_error_;

  // Gains
  Scalar p_gain_;
  Scalar i_gain_;
  Scalar d_gain_;
  Scalar i_max_;
  Scalar i_min_;
};

}  // namespace ur_controller
}  // namespace isaac