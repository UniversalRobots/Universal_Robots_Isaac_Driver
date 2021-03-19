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

#include "dashboardCommand.hpp"
#include "engine/core/assert.hpp"

namespace isaac
{
namespace dashboard_command
{
DashboardCommand FromProto(const ::DashboardCommandProto::DashboardCommand command)
{
  switch (command)
  {
    case ::DashboardCommandProto::DashboardCommand::LOAD:
      return DashboardCommand::load;
    case ::DashboardCommandProto::DashboardCommand::PLAY:
      return DashboardCommand::play;
    case ::DashboardCommandProto::DashboardCommand::STOP:
      return DashboardCommand::stop;
    case ::DashboardCommandProto::DashboardCommand::PAUSE:
      return DashboardCommand::pause;
    case ::DashboardCommandProto::DashboardCommand::QUIT:
      return DashboardCommand::quit;
    case ::DashboardCommandProto::DashboardCommand::SHUTDOWN:
      return DashboardCommand::shutdown;
    case ::DashboardCommandProto::DashboardCommand::RUNNING:
      return DashboardCommand::running;
    case ::DashboardCommandProto::DashboardCommand::ROBOTMODE:
      return DashboardCommand::robotmode;
    case ::DashboardCommandProto::DashboardCommand::GET_LOADED_PROGRAM:
      return DashboardCommand::getLoadedProgram;
    case ::DashboardCommandProto::DashboardCommand::POPUP:
      return DashboardCommand::popup;
    case ::DashboardCommandProto::DashboardCommand::CLOSE_POPUP:
      return DashboardCommand::closePopup;
    case ::DashboardCommandProto::DashboardCommand::ADD_TO_LOG:
      return DashboardCommand::addToLog;
    case ::DashboardCommandProto::DashboardCommand::IS_PROGRAM_SAVED:
      return DashboardCommand::isProgramSaved;
    case ::DashboardCommandProto::DashboardCommand::PROGRAM_STATE:
      return DashboardCommand::programState;
    case ::DashboardCommandProto::DashboardCommand::POLYSCOPE_VERSION:
      return DashboardCommand::polyscopeVersion;
    case ::DashboardCommandProto::DashboardCommand::SET_OPERATIONAL_MODE:
      return DashboardCommand::setOperationalMode;
    case ::DashboardCommandProto::DashboardCommand::GET_OPERATIONAL_MODE:
      return DashboardCommand::getOperationalMode;
    case ::DashboardCommandProto::DashboardCommand::CLEAR_OPERATIONAL_MODE:
      return DashboardCommand::clearOperationalMode;
    case ::DashboardCommandProto::DashboardCommand::POWER_ON:
      return DashboardCommand::powerOn;
    case ::DashboardCommandProto::DashboardCommand::POWER_OFF:
      return DashboardCommand::powerOff;
    case ::DashboardCommandProto::DashboardCommand::BRAKE_RELEASE:
      return DashboardCommand::brakeRelease;
    case ::DashboardCommandProto::DashboardCommand::SAFETYSTATUS:
      return DashboardCommand::safetystatus;
    case ::DashboardCommandProto::DashboardCommand::UNLOCK_PROTECTIVE_STOP:
      return DashboardCommand::unlockProtectiveStop;
    case ::DashboardCommandProto::DashboardCommand::CLOSE_SAFETY_POPUP:
      return DashboardCommand::closeSafetyPopup;
    case ::DashboardCommandProto::DashboardCommand::LOAD_INSTALLATION:
      return DashboardCommand::loadInstallation;
    case ::DashboardCommandProto::DashboardCommand::RESTART_SAFETY:
      return DashboardCommand::restartSafety;
    case ::DashboardCommandProto::DashboardCommand::IS_IN_REMOTE_CONTROL:
      return DashboardCommand::isInRemoteControl;
    case ::DashboardCommandProto::DashboardCommand::GET_SERIAL_NUMBER:
      return DashboardCommand::getSerialNumber;
    case ::DashboardCommandProto::DashboardCommand::GET_ROBOT_MODEL:
      return DashboardCommand::getRobotModel;
    case ::DashboardCommandProto::DashboardCommand::SAFETYMODE:
      return DashboardCommand::safetymode;
    case ::DashboardCommandProto::DashboardCommand::SET_USER_ROLE:
      return DashboardCommand::setUserRole;
    case ::DashboardCommandProto::DashboardCommand::GET_USER_ROLE:
      return DashboardCommand::getUserRole;
    default:
      PANIC("Unknown command %x", command);
  }
}

::DashboardCommandProto::DashboardCommand ToProto(const DashboardCommand command)
{
  switch (command)
  {
    case DashboardCommand::load:
      return ::DashboardCommandProto::DashboardCommand::LOAD;
    case DashboardCommand::stop:
      return ::DashboardCommandProto::DashboardCommand::STOP;
    case DashboardCommand::play:
      return ::DashboardCommandProto::DashboardCommand::PLAY;
    case DashboardCommand::pause:
      return ::DashboardCommandProto::DashboardCommand::PAUSE;
    case DashboardCommand::quit:
      return ::DashboardCommandProto::DashboardCommand::QUIT;
    case DashboardCommand::shutdown:
      return ::DashboardCommandProto::DashboardCommand::SHUTDOWN;
    case DashboardCommand::running:
      return ::DashboardCommandProto::DashboardCommand::RUNNING;
    case DashboardCommand::robotmode:
      return ::DashboardCommandProto::DashboardCommand::ROBOTMODE;
    case DashboardCommand::getLoadedProgram:
      return ::DashboardCommandProto::DashboardCommand::GET_LOADED_PROGRAM;
    case DashboardCommand::popup:
      return ::DashboardCommandProto::DashboardCommand::POPUP;
    case DashboardCommand::closePopup:
      return ::DashboardCommandProto::DashboardCommand::CLOSE_POPUP;
    case DashboardCommand::addToLog:
      return ::DashboardCommandProto::DashboardCommand::ADD_TO_LOG;
    case DashboardCommand::isProgramSaved:
      return ::DashboardCommandProto::DashboardCommand::IS_PROGRAM_SAVED;
    case DashboardCommand::programState:
      return ::DashboardCommandProto::DashboardCommand::PROGRAM_STATE;
    case DashboardCommand::polyscopeVersion:
      return ::DashboardCommandProto::DashboardCommand::POLYSCOPE_VERSION;
    case DashboardCommand::setOperationalMode:
      return ::DashboardCommandProto::DashboardCommand::SET_OPERATIONAL_MODE;
    case DashboardCommand::getOperationalMode:
      return ::DashboardCommandProto::DashboardCommand::GET_OPERATIONAL_MODE;
    case DashboardCommand::clearOperationalMode:
      return ::DashboardCommandProto::DashboardCommand::CLEAR_OPERATIONAL_MODE;
    case DashboardCommand::powerOn:
      return ::DashboardCommandProto::DashboardCommand::POWER_ON;
    case DashboardCommand::powerOff:
      return ::DashboardCommandProto::DashboardCommand::POWER_OFF;
    case DashboardCommand::brakeRelease:
      return ::DashboardCommandProto::DashboardCommand::BRAKE_RELEASE;
    case DashboardCommand::safetystatus:
      return ::DashboardCommandProto::DashboardCommand::SAFETYSTATUS;
    case DashboardCommand::unlockProtectiveStop:
      return ::DashboardCommandProto::DashboardCommand::UNLOCK_PROTECTIVE_STOP;
    case DashboardCommand::closeSafetyPopup:
      return ::DashboardCommandProto::DashboardCommand::CLOSE_SAFETY_POPUP;
    case DashboardCommand::loadInstallation:
      return ::DashboardCommandProto::DashboardCommand::LOAD_INSTALLATION;
    case DashboardCommand::restartSafety:
      return ::DashboardCommandProto::DashboardCommand::RESTART_SAFETY;
    case DashboardCommand::isInRemoteControl:
      return ::DashboardCommandProto::DashboardCommand::IS_IN_REMOTE_CONTROL;
    case DashboardCommand::getSerialNumber:
      return ::DashboardCommandProto::DashboardCommand::GET_SERIAL_NUMBER;
    case DashboardCommand::getRobotModel:
      return ::DashboardCommandProto::DashboardCommand::GET_ROBOT_MODEL;
    case DashboardCommand::safetymode:
      return ::DashboardCommandProto::DashboardCommand::SAFETYMODE;
    case DashboardCommand::setUserRole:
      return ::DashboardCommandProto::DashboardCommand::SET_USER_ROLE;
    case DashboardCommand::getUserRole:
      return ::DashboardCommandProto::DashboardCommand::GET_USER_ROLE;
    default:
      PANIC("Unknown command %x", command);
  }
}

}  // namespace dashboard_command
}  // namespace isaac