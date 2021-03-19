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

#include "engine/alice/alice_codelet.hpp"
#include "messages/basic.capnp.h"

namespace isaac
{
namespace controller_stopper
{
/*!
 * \brief ControllerStopper class is a small helper component that stops and restarts the controller based on a boolean
 * message. When the message goes to false, the controller is stopped. If message returns to true the stopped controller
 * is restarted.
 */
class ControllerStopper : public isaac::alice::Codelet
{
public:
  /*!
   * \brief Handles the setup functionality of this class. This includes collecting the controller node.
   */
  void start() override;
  /*!
   * \brief This functions is run every time the robot_program_running message is published.
   * When the message goes to false, the controller is stopped. If message returns to true the stopped controller is
   * restarted.
   */
  void tick() override;

  /*!
   * \brief Receives the robots running state.
   */
  ISAAC_PROTO_RX(BooleanProto, robot_program_running);

  /*!
   * \brief Name of the node containing the controller node that is sending targets to the driver.
   * This node is started and stopped based upon the drivers control status of the robot.
   */
  ISAAC_PARAM(std::string, controller);

private:
  // Node containing the controller.
  alice::Node* controller_node_;

  // Boolean to represent if the controller is currently stopped
  bool controller_node_stopped_;
};

}  // namespace controller_stopper
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::controller_stopper::ControllerStopper)