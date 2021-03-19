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

#include <string>

#include "packages/universal_robots/ur_msg/dashboard_command.capnp.h"

namespace isaac
{
namespace dashboard_command
{
/*!
 * \brief Enum holding all dashboard commands available.
 * For documentation about the dashboardserver see
 *  - https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/
 *  - https://www.universal-robots.com/articles/ur/dashboard-server-cb-series-port-29999/
 */
enum class DashboardCommand
{
  load,                  // Load program <program.urp>
  play,                  // Play program
  stop,                  // Stop program
  pause,                 // Pause program
  quit,                  // Disconnect client
  shutdown,              // Shut down robot
  running,               // Programm running?
  robotmode,             // Current robot mode
  getLoadedProgram,      // Returns loaded program
  popup,                 // Show popup <message>
  closePopup,            // Close popup
  addToLog,              // Add message to log <log-message>
  isProgramSaved,        // Is current program saved?
  programState,          // Returns current program state
  polyscopeVersion,      // Polyscope version number
  setOperationalMode,    // Set operational mode <mode>
  getOperationalMode,    // Returns curretn operationalmode
  clearOperationalMode,  // Dashboard server no longer controls operational mode
  powerOn,               // Power on robot
  powerOff,              // Power down robot
  brakeRelease,          // Brake release
  safetystatus,          // Current safetystatus
  unlockProtectiveStop,  // Unlocks protective stop
  closeSafetyPopup,      // Close safety popup
  loadInstallation,      // Load installation <default.installation>
  restartSafety,         // Restart safety
  isInRemoteControl,     // Returns remote control status
  getSerialNumber,       // Returns serial number
  getRobotModel,         // Returns robot model
  safetymode,            // Current safetymode
  setUserRole,           // Set User role <role>
  getUserRole,           // Get current user role
};

/*!
 * \brief Parses command in proto message to a dashboardCommand.
 *
 * \param command Proto command.
 */
DashboardCommand FromProto(const ::DashboardCommandProto::DashboardCommand command);

/*!
 * \brief Creates type of command for Proto message
 *
 * \param command DashboardCommand.
 */
::DashboardCommandProto::DashboardCommand ToProto(const DashboardCommand command);

}  // namespace dashboard_command
}  // namespace isaac