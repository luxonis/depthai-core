#pragma once

#include <string>
#include <spdlog/logger.h>

namespace dai
{
namespace utility
{

std::string getEnv(const std::string& var);
std::string getEnv(const std::string& var, spdlog::logger& logger);
std::vector<std::string> splitList(const std::string& list, const std::string& delimiter);


} // namespace utility
} // namespace dai
