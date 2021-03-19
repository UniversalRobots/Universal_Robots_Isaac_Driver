# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2021 Universal Robots A/S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
# Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
# NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
# MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
# CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
# OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
# USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
# Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
# appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
# published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
# in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
# please contact legal@universal-robots.com.
# -- END LICENSE BLOCK ------------------------------------------------

@0xfe6f9cc1919289ad;

# A message to represent all dashboard commands available
# For documentation about the dashboard server see
#   - https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/
#   - https://www.universal-robots.com/articles/ur/dashboard-server-cb-series-port-29999/
struct DashboardCommandProto
{
  # Enum holding all dashboard commands available.
  enum DashboardCommand
  {
    load                      @0;   # Load program </programs/program.urp>
    play                      @1;   # Play program
    stop                      @2;   # Stop program
    pause                     @3;   # Pause program
    quit                      @4;   # Disconnect client
    shutdown                  @5;   # Shut down robot
    running                   @6;   # Programm running?
    robotmode                 @7;   # Current robot mode
    getLoadedProgram          @8;   # Returns loaded program
    popup                     @9;   # Show popup <message>
    closePopup                @10;  # Close popup
    addToLog                  @11;  # Add message to log <log-message>
    isProgramSaved            @12;  # Is current program saved?
    programState              @13;  # Returns current program state
    polyscopeVersion          @14;  # Polyscope version number
    setOperationalMode        @15;  # Set operational mode <mode>
    getOperationalMode        @16;  # Returns curretn operationalmode
    clearOperationalMode      @17;  # Dashboard server no longer controls operational mode
    powerOn                   @18;  # Power on robot
    powerOff                  @19;  # Power down robot
    brakeRelease              @20;  # Brake release
    safetystatus              @21;  # Current safety status
    unlockProtectiveStop      @22;  # Unlocks protective stop
    closeSafetyPopup          @23;  # Close safety popup
    loadInstallation          @24;  # Load installation </programs/default.installation>
    restartSafety             @25;  # Restart safety
    isInRemoteControl         @26;  # Returns remote control status
    getSerialNumber           @27;  # Returns serial number
    getRobotModel             @28;  # Returns robot model
    safetymode                @29;  # Current safetymode
    setUserRole               @30;  # Setting user role <role>
    getUserRole               @31;  # Get current user role
  }

  # The request/command to send to the dashboardserver.
  dashboardRequest @0: DashboardCommand;

  # Possible argument to the dashboard command can be empty.
  argument @1: Text = "";
}

# A message to represent the status of a dashboard command.
struct DashboardCommandStatusProto
{
  # The anwser from the dashboard server.
  anwser @0: Text;

  # Returns whether the command was a succes or not.
  success @1: Bool;
}