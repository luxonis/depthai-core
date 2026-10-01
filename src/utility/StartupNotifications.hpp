#pragma once

#ifdef DEPTHAI_ENABLE_CURL

    #include <nlohmann/json_fwd.hpp>
    #include <string>
    #include <vector>

    #include "depthai/device/Platform.hpp"

namespace dai {
namespace utility {

nlohmann::json getStartupNotifications(const std::string& url);

std::vector<std::string> collectMessages(
    const nlohmann::json& json, const std::string& depthaiVersion, Platform platform, const std::string& deviceSKU, const std::string& osVersion);

void printStartupNotifications(const std::string& depthaiVersion, Platform platform, const std::string& deviceSKU, const std::string& osVersion);

}  // namespace utility
}  // namespace dai

#endif  // DEPTHAI_ENABLE_CURL
