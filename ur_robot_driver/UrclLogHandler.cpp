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

#include "UrclLogHandler.hpp"
#include "engine/core/logger.hpp"

namespace isaac
{
namespace ur_driver
{

bool g_initialized_ = false;
std::unique_ptr<UrclLogHandler> g_log_handler(new UrclLogHandler);

UrclLogHandler::UrclLogHandler() = default;

void UrclLogHandler::log(const char* file, int line, urcl::LogLevel loglevel, const char* message)
{
  switch (loglevel)
  {
    case urcl::LogLevel::INFO:
      logger::LoggingFunction(file, line, logger::Severity::INFO, message);
      break;
    case urcl::LogLevel::DEBUG:
      logger::LoggingFunction(file, line, logger::Severity::DEBUG, message);
      break;
    case urcl::LogLevel::WARN:
      logger::LoggingFunction(file, line, logger::Severity::WARNING, message);
      break;
    case urcl::LogLevel::ERROR:
      logger::LoggingFunction(file, line, logger::Severity::ERROR, message);
      break;
    case urcl::LogLevel::FATAL:
      logger::LoggingFunction(file, line, logger::Severity::ERROR, message);
      break;
    default:
      break;
  }
}

void registerUrclLogHandler()
{
  if (g_initialized_ == false)
  {
    urcl::setLogLevel(urcl::LogLevel::DEBUG);
    urcl::registerLogHandler(std::move(g_log_handler));
    g_initialized_ = true;
  }
}

void unregisterUrclLogHandler()
{
  if (g_initialized_ == true)
  {
    urcl::unregisterLogHandler();
    g_initialized_ = false;
  }
}

}  // namespace ur_driver
}  // namespace isaac