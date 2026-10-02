#pragma once

#ifdef DEPTHAI_ENABLE_CURL

    #include <XLink/XLinkPublicDefines.h>

    #include <nlohmann/json_fwd.hpp>
    #include <string>
    #include <vector>

    #include "depthai/device/Platform.hpp"

namespace dai {
namespace utility {

nlohmann::json getStartupNotifications(const std::string& url);

std::vector<std::string> collectMessages(const nlohmann::json& json,
                                         const std::string& depthaiVersion,
                                         Platform platform,
                                         XLinkProtocol_t protocol,
                                         const std::string& osVersion,
                                         const std::string& deviceSKU);

void printStartupNotifications(
    const std::string& depthaiVersion, Platform platform, XLinkProtocol_t protocol, const std::string& osVersion, const std::string& deviceSKU);

}  // namespace utility
}  // namespace dai

#endif  // DEPTHAI_ENABLE_CURL
