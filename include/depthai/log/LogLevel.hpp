#pragma once

#include <cstdint>
namespace dai {

enum class LogLevel : std::int32_t { TRACE = 0, DEBUG, INFO, WARN, ERR, CRITICAL, OFF };  // Same values as the spdlog levels

}  // namespace dai
