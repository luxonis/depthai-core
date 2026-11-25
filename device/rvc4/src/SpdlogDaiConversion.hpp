#pragma once

#include <spdlog/spdlog.h>
#include "depthai/log/LogMessage.hpp"
#include "depthai/log/LogLevel.hpp"

namespace dai {
namespace gate {

spdlog::level::level_enum daiLogLevelToSpdlogLevel(dai::LogLevel level);
dai::LogLevel spdlogLevelToDaiLogLevel(spdlog::level::level_enum level);

dai::LogMessage spdlogMsgToDaiLogMessage(const spdlog::details::log_msg& msg);

} // namespace gate
} // namespace dai