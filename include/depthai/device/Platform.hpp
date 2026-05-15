#pragma once

// std
#include <string>

namespace dai {

/**
 * @brief Hardware platform type
 */
enum class Platform { RVC2, RVC3, RVC4 };

/**
 * @brief Convert Platform enum to string
 *
 * @param platform Platform enum
 * @return std::string String representation of Platform
 */
std::string platform2string(Platform platform);

/**
 * @brief Convert string to Platform enum
 *
 * @param platform String representation of Platform
 * @return Platform Platform enum
 */
Platform string2platform(const std::string& platform);

}  // namespace dai
