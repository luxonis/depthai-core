#include "SpdlogDaiConversion.hpp"

#include <assert.h>

namespace dai {
namespace gate {
spdlog::level::level_enum daiLogLevelToSpdlogLevel(dai::LogLevel level) {
    spdlog::level::level_enum spdlogLevel = spdlog::level::warn;
    switch(level) {
        case dai::LogLevel::TRACE:
            spdlogLevel = spdlog::level::trace;
            break;
        case dai::LogLevel::DEBUG:
            spdlogLevel = spdlog::level::debug;
            break;
        case dai::LogLevel::INFO:
            spdlogLevel = spdlog::level::info;
            break;
        case dai::LogLevel::WARN:
            spdlogLevel = spdlog::level::warn;
            break;
        case dai::LogLevel::ERR:
            spdlogLevel = spdlog::level::err;
            break;
        case dai::LogLevel::CRITICAL:
            spdlogLevel = spdlog::level::critical;
            break;
        case dai::LogLevel::OFF:
            spdlogLevel = spdlog::level::off;
            break;
        default:
            assert(0 && "Add new value to switch!");
    }

    return spdlogLevel;
}

dai::LogLevel spdlogLevelToDaiLogLevel(spdlog::level::level_enum level) {
    dai::LogLevel daiLogLevel = dai::LogLevel::WARN;
    switch(level) {
        case spdlog::level::trace:
            daiLogLevel = dai::LogLevel::TRACE;
            break;
        case spdlog::level::debug:
            daiLogLevel = dai::LogLevel::DEBUG;
            break;
        case spdlog::level::info:
            daiLogLevel = dai::LogLevel::INFO;
            break;
        case spdlog::level::warn:
            daiLogLevel = dai::LogLevel::WARN;
            break;
        case spdlog::level::err:
            daiLogLevel = dai::LogLevel::ERR;
            break;
        case spdlog::level::critical:
            daiLogLevel = dai::LogLevel::CRITICAL;
            break;
        case spdlog::level::off:
            daiLogLevel = dai::LogLevel::OFF;
            break;
        default:
            assert(0 && "Add new value to switch!");
    }

    return daiLogLevel;
}

dai::LogMessage spdlogMsgToDaiLogMessage(const spdlog::details::log_msg& msg) {
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::LogMessage daiMsg;
    daiMsg.nodeIdName = std::string(msg.logger_name.begin(), msg.logger_name.end());
    daiMsg.level = spdlogLevelToDaiLogLevel(msg.level);

    auto d = msg.time.time_since_epoch();
    daiMsg.time.sec = duration_cast<seconds>(d).count();
    daiMsg.time.nsec = duration_cast<nanoseconds>(d).count() % 1000000000;

    daiMsg.colorRangeStart = msg.color_range_start;
    daiMsg.colorRangeEnd = msg.color_range_end;
    daiMsg.payload = std::string(msg.payload.begin(), msg.payload.end());

    return daiMsg;
}

}  // namespace gate
}  // namespace dai
