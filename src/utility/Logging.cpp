#include "Logging.hpp"

#include "Environment.hpp"

namespace dai {

LogLevel spdlogLevelToLogLevel(spdlog::level::level_enum level, LogLevel defaultValue) {
    switch(level) {
        case spdlog::level::trace:
            return LogLevel::TRACE;
        case spdlog::level::debug:
            return LogLevel::DEBUG;
        case spdlog::level::info:
            return LogLevel::INFO;
        case spdlog::level::warn:
            return LogLevel::WARN;
        case spdlog::level::err:
            return LogLevel::ERR;
        case spdlog::level::critical:
            return LogLevel::CRITICAL;
        case spdlog::level::off:
            return LogLevel::OFF;
        case spdlog::level::n_levels:
        default:
            return defaultValue;
    }
}

spdlog::level::level_enum logLevelToSpdlogLevel(LogLevel level, spdlog::level::level_enum defaultValue) {
    switch(level) {
        case LogLevel::TRACE:
            return spdlog::level::trace;
        case LogLevel::DEBUG:
            return spdlog::level::debug;
        case LogLevel::INFO:
            return spdlog::level::info;
        case LogLevel::WARN:
            return spdlog::level::warn;
        case LogLevel::ERR:
            return spdlog::level::err;
        case LogLevel::CRITICAL:
            return spdlog::level::critical;
        case LogLevel::OFF:
            return spdlog::level::off;
        default:
            return defaultValue;
    }
}

Logging::Logging() : logger("depthai", {std::make_shared<spdlog::sinks::stdout_color_sink_mt>()}) {
    // Default global logging level set to WARN; override with ENV variable 'DEPTHAI_LEVEL'
    // Taken from spdlog, to replace with DEPTHAI_LEVEL instead of SPDLOG_LEVEL
    // spdlog::cfg::load_env_levels();
    auto level = spdlog::level::warn;
    // Set the logging level to warn by default, so it doesn't spam the consol with the env checks.
    logger.set_level(level);
    auto envLevel = utility::getEnvAs<std::string>("DEPTHAI_LEVEL", "", logger);
    if(!envLevel.empty()) {
        level = parseLevel(envLevel);
    }

    // Set the logging level to the parsed level
    logger.set_level(level);

    auto debugStr = utility::getEnvAs<std::string>("DEPTHAI_DEBUG", "", logger);
    if(!debugStr.empty()) {
        // Try parsing the string as a number
        try {
            int debug{std::stoi(debugStr)};
            if(debug && (level > spdlog::level::debug)) {
                logger.set_level(spdlog::level::debug);
                logger.info("DEPTHAI_DEBUG enabled, lowered DEPTHAI_LEVEL to 'debug'");
            }
        } catch(const std::invalid_argument& e) {
            logger.warn("DEPTHAI_DEBUG value invalid: {}, should be a number (non-zero to enable)", e.what());
        }
    }
}

spdlog::level::level_enum Logging::parseLevel(std::string lvl) {
    std::transform(lvl.begin(), lvl.end(), lvl.begin(), [](char ch) { return static_cast<char>((ch >= 'A' && ch <= 'Z') ? ch + ('a' - 'A') : ch); });

    if(lvl == "trace") {
        return spdlog::level::trace;
    } else if(lvl == "debug") {
        return spdlog::level::debug;
    } else if(lvl == "info") {
        return spdlog::level::info;
    } else if(lvl == "warn") {
        return spdlog::level::warn;
    } else if(lvl == "error") {
        return spdlog::level::err;
    } else if(lvl == "off") {
        return spdlog::level::off;
    } else {
        throw std::invalid_argument(fmt::format("Cannot parse logging level: {}", lvl));
    }
}

}  // namespace dai
