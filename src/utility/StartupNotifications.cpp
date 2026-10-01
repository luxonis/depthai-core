#include "StartupNotifications.hpp"

#include <cpr/cpr.h>

#include <cstdio>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "depthai/device/Version.hpp"
#include "utility/Environment.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace utility {

nlohmann::json getStartupNotifications(const std::string& url) {
    if(url.empty()) {
        return nlohmann::json::object();
    }

    try {
        const auto response = cpr::Get(cpr::Url{url}, cpr::Timeout{5000}, cpr::VerifySsl{true});
        if(response.error.code == cpr::ErrorCode::OK && response.status_code == 200) {
            auto notifications = nlohmann::json::parse(response.text);
            if(notifications.is_object()) {
                return notifications;
            }
        } else {
            logger::debug("Startup notification request failed: HTTP {}, {}", response.status_code, response.error.message);
        }
    } catch(const std::exception& ex) {
        logger::debug("Startup notification request failed: {}", ex.what());
    }
    return nlohmann::json::object();
}

std::vector<std::string> collectMessages(
    const nlohmann::json& json, const std::string& depthaiVersion, Platform platform, const std::string& deviceSKU, const std::string& osVersion) {
    std::vector<std::string> messages;
    if(!json.is_object()) {
        return messages;
    }

    // General
    if(const auto general = json.find("general"); general != json.end() && general->is_array()) {
        for(const auto& message : *general) {
            if(message.is_string() && !message.get_ref<const std::string&>().empty()) {
                messages.push_back(message.get<std::string>());
            }
        }
    }

    // DepthAI Core Version
    if(const auto version = json.find("version"); version != json.end() && version->is_string() && !depthaiVersion.empty()) {
        try {
            const Version currentVersion(depthaiVersion);
            const Version latestVersion(version->get_ref<const std::string&>());
            if(latestVersion > currentVersion) {
                messages.push_back(fmt::format("A new DepthAI version is available: {} (current: {}).", latestVersion.toString(), depthaiVersion));
            }
        } catch(const std::invalid_argument& ex) {
            logger::debug("Skipping DepthAI startup version comparison: {}", ex.what());
        }
    }

    // Platform specific
    const auto platformEntry = json.find(platform == Platform::RVC4 ? "rvc4" : "rvc2");
    if(platformEntry != json.end() && platformEntry->is_object()) {
        // General platform specific messages
        if(const auto general = platformEntry->find("general"); general != platformEntry->end() && general->is_array()) {
            for(const auto& message : *general) {
                if(message.is_string() && !message.get_ref<const std::string&>().empty()) {
                    messages.push_back(message.get<std::string>());
                }
            }
        }

        // RVC4 OS Version
        if(const auto version = platformEntry->find("osVersion");
           platform == Platform::RVC4 && !osVersion.empty() && version != platformEntry->end() && version->is_string()) {
            try {
                const Version currentVersion(osVersion);
                const Version latestVersion(version->get_ref<const std::string&>());
                if(latestVersion > currentVersion) {
                    messages.push_back(fmt::format("A new RVC4 OS version is available: {} (current: {}).", latestVersion.toString(), osVersion));
                }
            } catch(const std::invalid_argument& ex) {
                logger::debug("Skipping RVC4 OS startup version comparison: {}", ex.what());
            }
        }

        // Device Specific messages
        if(const auto sku = platformEntry->find(deviceSKU); sku != platformEntry->end() && sku->is_array()) {
            for(const auto& message : *sku) {
                if(message.is_string() && !message.get_ref<const std::string&>().empty()) {
                    messages.push_back(message.get<std::string>());
                }
            }
        }
    }

    return messages;
}

void printStartupNotifications(const std::string& depthaiVersion, Platform platform, const std::string& deviceSKU, const std::string& osVersion) {
    const auto url = getEnvAs<std::string>("DEPTHAI_STARTUP_NOTIFICATIONS_URL", "");  // TO DO: Add default value
    if(url.empty()) {
        return;
    }

    nlohmann::json fetchedJson = getStartupNotifications(url);
    std::vector<std::string> collectedMessages = collectMessages(fetchedJson, depthaiVersion, platform, deviceSKU, osVersion);
    for(const auto& message : collectedMessages) {
        fmt::print(stderr, "{}\n", message);
    }
}

}  // namespace utility
}  // namespace dai
