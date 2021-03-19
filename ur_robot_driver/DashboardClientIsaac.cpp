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

#include <regex>

#include "DashboardClientIsaac.hpp"
#include "UrclLogHandler.hpp"

namespace isaac
{
namespace ur_driver
{
void DashboardClientIsaac::start()
{
  registerUrclLogHandler();

  const auto robot_ip = try_get_robot_ip();
  if (!robot_ip)
  {
    reportFailure("No IP address was specified unable to start driver");
    return;
  }

  dc_client_.reset(new urcl::DashboardClient(get_robot_ip()));
  dc_client_->connect();

  getVersion();

  // Setup tick
  tickOnMessage(rx_dashboard_command());
}

void DashboardClientIsaac::tick()
{
  // Receive and execute dashboard command
  auto proto = rx_dashboard_command().getProto();
  dashboard_command::DashboardCommand command = dashboard_command::FromProto(proto.getDashboardRequest());
  Response resp = handleCommand(command, proto.getArgument());

  // Publish anwser
  auto proto_sender = tx_dashboard_anwser().initProto();
  proto_sender.setAnwser(resp.anwser);
  proto_sender.setSuccess(resp.success);
  tx_dashboard_anwser().publish();
}

void DashboardClientIsaac::stop()
{
  dc_client_->disconnect();
  dc_client_.reset();
}

DashboardClientIsaac::Response DashboardClientIsaac::handleCommand(const dashboard_command::DashboardCommand& command,
                                                                   const std::string& argument)
{
  switch (command)
  {
    case dashboard_command::DashboardCommand::load:
      return executeCommand("load " + argument + "\n", "Loading program: (.*)");
    case dashboard_command::DashboardCommand::stop:
      return executeCommand("stop\n", "Stopped");
    case dashboard_command::DashboardCommand::play:
      return executeCommand("play\n", "Starting program");
    case dashboard_command::DashboardCommand::pause:
      return executeCommand("pause\n", "Pausing program");
    case dashboard_command::DashboardCommand::quit:
      return executeCommand("quit\n", "Disconnected");
    case dashboard_command::DashboardCommand::shutdown:
      return executeCommand("shutdown\n", "Shutting down");
    case dashboard_command::DashboardCommand::running:
      return executeCommand("running\n", "Program running: (.*)");
    case dashboard_command::DashboardCommand::robotmode:
      return executeCommand("robotmode\n", "Robotmode: (.*)");
    case dashboard_command::DashboardCommand::getLoadedProgram:
      return executeCommand("get loaded program\n", "Loaded program: (.*)");
    case dashboard_command::DashboardCommand::popup:
      return executeCommand("popup " + argument + "\n", "showing popup");
    case dashboard_command::DashboardCommand::closePopup:
      return executeCommand("close popup\n", "closing popup");
    case dashboard_command::DashboardCommand::addToLog:
      return executeCommand("addToLog " + argument + "\n", "(Added log message|No log message to add)");
    case dashboard_command::DashboardCommand::isProgramSaved:
      return executeCommand("isProgramSaved\n", "(true|false) (.*)");
    case dashboard_command::DashboardCommand::programState:
      return executeCommand("programState\n", "(STOPPED|PLAYING|PAUSED) (.*)");
    case dashboard_command::DashboardCommand::polyscopeVersion:
      return executeCommand("PolyscopeVersion\n", "URSoftware (.*)");
    case dashboard_command::DashboardCommand::setOperationalMode:
      if (major_version_ >= 5)
      {
        return executeCommand("set operational mode " + argument + "\n", "Operational mode ('manual'|'automatic') is "
                                                                         "set");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::getOperationalMode:
      if (major_version_ == 5 && minor_version_ >= 6)
      {
        return executeCommand("get operational mode\n", "(MANUAL|AUTOMATIC|NONE)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::clearOperationalMode:
      if (major_version_ == 5)
      {
        return executeCommand("clear operational mode\n", "No longer controlling the operational mode. (.*)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::powerOn:
      return executeCommand("power on\n", "Powering on");
    case dashboard_command::DashboardCommand::powerOff:
      return executeCommand("power off\n", "Powering off");
    case dashboard_command::DashboardCommand::brakeRelease:
      return executeCommand("brake release\n", "Brake releasing");
    case dashboard_command::DashboardCommand::safetystatus:
      if (major_version_ == 5 && minor_version_ >= 4)
      {
        return executeCommand("safetystatus\n", "Safetystatus: (.*)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::unlockProtectiveStop:
      return executeCommand("unlock protective stop\n", "Protective stop releasing");
    case dashboard_command::DashboardCommand::closeSafetyPopup:
      return executeCommand("close safety popup\n", "closing safety popup");
    case dashboard_command::DashboardCommand::loadInstallation:
      return executeCommand("load installation " + argument + "\n", "Loading installation: (.*)");
    case dashboard_command::DashboardCommand::restartSafety:
      return executeCommand("restart safety\n", "Restarting safety");
    case dashboard_command::DashboardCommand::isInRemoteControl:
      if (major_version_ == 5 && minor_version_ >= 6)
      {
        return executeCommand("is in remote control\n", "(true|false)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::getSerialNumber:
      if ((major_version_ == 5 && minor_version_ >= 6) || (major_version_ == 3 && minor_version_ >= 12))
      {
        return executeCommand("get serial number\n", "\\d+");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::getRobotModel:
      if ((major_version_ == 5 && minor_version_ >= 6) || (major_version_ == 3 && minor_version_ >= 12))
      {
        return executeCommand("get robot model\n", "(UR3|UR5|UR10|UR16)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::safetymode:
      return executeCommand("safetymode\n", "Safetymode: (.*)");
    case dashboard_command::DashboardCommand::setUserRole:
      if (major_version_ == 3)
      {
        return executeCommand("setUserRole " + argument + "\n", "Setting user role: (.*)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    case dashboard_command::DashboardCommand::getUserRole:
      if (major_version_ == 3)
      {
        return executeCommand("getUserRole\n", "(PROGRAMMER|OPERATOR|NONE|LOCKED|RESTRICTED)");
      }
      else
      {
        Response resp;
        resp.anwser = "Command not supported for current software version: major " + std::to_string(major_version_) +
                      " minor " + std::to_string(minor_version_);
        resp.success = false;
        return resp;
      }
    default:
      PANIC("Unknown command %x", command);
  }
}

DashboardClientIsaac::Response DashboardClientIsaac::executeCommand(const std::string& command,
                                                                    const std::string& expected)
{
  Response resp;
  if (dc_client_->getState() == urcl::comm::SocketState::Connected)
  {
    resp.anwser = dc_client_->sendAndReceive(command);
    resp.success = std::regex_match(resp.anwser, std::regex(expected));
  }
  else
  {
    LOG_INFO("reconnecting");
    if (dc_client_->connect())
    {
      resp.anwser = dc_client_->sendAndReceive(command);
      resp.success = std::regex_match(resp.anwser, std::regex(expected));
    }
    else
    {
      resp.anwser = "failed to connect to dashboard server, unable to send command";
      resp.success = false;
    }
  }
  return resp;
}

void DashboardClientIsaac::getVersion()
{
  std::smatch match;
  std::string anwser = dc_client_->sendAndReceive("PolyscopeVersion\n");
  std::regex_search(anwser, match, std::regex("\\d+\\.\\d+\\.\\d+\\.\\d+"));
  if (match.empty())
  {
    reportFailure("Unable to get polyscope version");
  }

  const std::regex base_regex("\\d+");
  std::smatch base_match;
  if (std::regex_search(anwser, base_match, base_regex))
  {
    major_version_ = std::atoi(base_match[0].str().c_str());
    anwser = base_match.suffix();
  }
  else
  {
    reportFailure("Unable to get major version");
  }

  if (std::regex_search(anwser, base_match, base_regex))
  {
    minor_version_ = std::atoi(base_match[0].str().c_str());
    anwser = base_match.suffix();
  }
  else
  {
    reportFailure("Unable to get minor version");
  }
}

}  // namespace ur_driver
}  // namespace isaac