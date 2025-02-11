#pragma once

#include <spdlog/logger.h>

#include <string>

namespace dai {
namespace utility {

std::string getEnv(const std::string& var);
std::string getEnv(const std::string& var, spdlog::logger& logger);
std::vector<std::string> splitList(const std::string& list, const std::string& delimiter);

}  // namespace utility
}  // namespace dai
