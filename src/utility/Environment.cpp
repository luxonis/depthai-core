#include "Environment.hpp"

#include <spdlog/details/os.h>
#include <spdlog/spdlog.h>

#include <mutex>
#include <unordered_map>

namespace dai {
namespace utility {

static std::mutex mtx;
static std::unordered_map<std::string, std::string> map;

std::string getEnv(const std::string& var) {
    std::unique_lock<std::mutex> lock(mtx);

    if(map.count(var) > 0) {
        return map.at(var);
    }
    auto value = spdlog::details::os::getenv(var.c_str());
    map[var] = value;

    // Warn if env variable set
    if(!value.empty()) {
        spdlog::warn("Environment '{}' set to '{}'", var, value);
    }

    return value;
}

}  // namespace utility
}  // namespace dai
