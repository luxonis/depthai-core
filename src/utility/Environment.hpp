#pragma once

#include <string>
#include <spdlog/spdlog.h>

namespace spdlog {

inline level::level_enum get_level()
{
    return default_logger_raw()->level();
}

}

namespace dai
{
namespace utility
{

std::string getEnv(const std::string& var);
void spdlogLoadLevels(const std::string &input);

} // namespace utility
} // namespace dai
