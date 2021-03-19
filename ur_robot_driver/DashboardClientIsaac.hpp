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

#include "engine/alice/alice_codelet.hpp"
#include "ur_client_library/ur/dashboard_client.h"
#include "packages/universal_robots/ur_msg/dashboardCommand.hpp"
#include "packages/universal_robots/ur_msg/dashboard_command.capnp.h"

namespace isaac
{
namespace ur_driver
{
/*!
 * \brief The dashboardClientIsaac class is the interface between the dashboardserver and an Isaac application.
 * Almost all the dashboard commands can be sent through this codelet to the dashboardserver and this codelet,
 * will return the anwser to the application.
 */

class DashboardClientIsaac : public isaac::alice::Codelet
{
public:
  /*!
   * \brief Handles the setup functionality of this class. This includes setting up dashboardclient object.
   */
  void start() override;
  /*!
   * \brief This functions is run when a new message is received. It sends the dashboard command to the dashboard server
   * receives the anwser and publishing the anwser to the application.
   */
  void tick() override;

  /*!
   * \brief Disconnects from the dashboard server
   */
  void stop() override;

  /*!
   * \brief Receiving the dashboard command, which is forwarded to the dashboard server.
   */
  ISAAC_PROTO_RX(DashboardCommandProto, dashboard_command);

  /*!
   * \brief Channel to publish the anwser from the dashbordserver and whether the command was a success.
   */
  ISAAC_PROTO_TX(DashboardCommandStatusProto, dashboard_anwser);

  /*!
   * \brief The ip address of the robot.
   */
  ISAAC_PARAM(std::string, robot_ip);

private:
  /*!
   * \brief Struct representing the anwser from the dasboardserver.
   *
   * \param anwser String storing the anwser from the dashboardserver.
   * \param success Representing if the command was a success.
   */
  struct Response
  {
    std::string anwser;
    bool success;
  };

  /*!
   * \brief Handles the dashboard command, by changing the command with the correct string,
   * that is then send to the dashboardserver.
   *
   * \param command Dashboard command
   * \param argument argument to the dashboard command.
   */
  Response handleCommand(const dashboard_command::DashboardCommand& command, const std::string& argument);

  /*!
   * \brief Sends the command to dashboard server and receives the anwser.
   * It matches the anwser with the expected anwser, to see if the command was a success.
   *
   * \param command The string command to send.
   * \param expected The expected anwser as a string.
   */
  Response executeCommand(const std::string& command, const std::string& expected);

  /*!
   * \brief Used to retrieve polyscope version, in order to make sure commands that is not available in current version
   * arent't executed.
   */
  void getVersion();

  // Dashboardclient object.
  std::unique_ptr<urcl::DashboardClient> dc_client_;

  int major_version_;
  int minor_version_;

  bool initialized_;
};

}  // namespace ur_driver
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ur_driver::DashboardClientIsaac)