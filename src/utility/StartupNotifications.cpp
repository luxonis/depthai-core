#include "StartupNotifications.hpp"

#include <cpr/cpr.h>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <nlohmann/json.hpp>
#include <unordered_map>

#include "utility/Environment.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace utility {

nlohmann::json getStartupNotifications(const std::string& url, const std::atomic<bool>* cancel) {
    if(url.empty() || (cancel && cancel->load())) {
        return nlohmann::json::object();
    }

    try {
        const auto response = cpr::Get(
            cpr::Url{url},
            cpr::Timeout{5000},
            cpr::VerifySsl{true},
            cpr::ProgressCallback{[cancel](cpr::cpr_off_t, cpr::cpr_off_t, cpr::cpr_off_t, cpr::cpr_off_t, intptr_t) { return !cancel || !cancel->load(); }});
        if(cancel && cancel->load()) {
            return nlohmann::json::object();
        }
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

std::vector<std::string> collectMessages(const nlohmann::json& json,
                                         const std::string& depthaiVersion,
                                         Platform platform,
                                         XLinkProtocol_t protocol,
                                         const std::string& osVersion,
                                         const std::string& deviceSKU) {
    std::vector<std::string> messages;
    const auto entries = json.find("messages");
    if(entries == json.end() || !entries->is_array()) {
        return messages;
    }

    std::string protocolName;
    if(protocol == X_LINK_USB_VSC || protocol == X_LINK_USB_CDC || protocol == X_LINK_USB_EP) {
        protocolName = "usb";
    } else if(protocol == X_LINK_TCP_IP) {
        protocolName = "tcpip";
    }
    const std::unordered_map<std::string, std::string> filters{
        {"depthaiVersions", depthaiVersion},
        {"platforms", platform2string(platform)},
        {"protocols", protocolName},
        {"osVersions", osVersion},
        {"deviceSKUs", deviceSKU},
    };

    // Go over the messages and filter the ones relevant
    for(const auto& entry : *entries) {
        const auto message = entry.find("message");
        if(message == entry.end() || !message->is_string() || message->get_ref<const std::string&>().empty()) {
            continue;
        }

        bool applicable = true;
        for(const auto& [key, value] : filters) {
            const auto filter = entry.find(key);
            if(filter == entry.end() || !filter->is_array()) {
                applicable = false;
                break;
            }

            // Check if all values are valid
            const bool validValues = std::all_of(
                filter->begin(), filter->end(), [](const nlohmann::json& item) { return item.is_string() && !item.get_ref<const std::string&>().empty(); });

            // Empty lists match all devices; a trailing '*' matches any suffix.
            const bool matchesDevice =
                validValues && (filter->empty() || std::any_of(filter->begin(), filter->end(), [&value = value](const nlohmann::json& item) {
                                    const auto& pattern = item.get_ref<const std::string&>();
                                    return pattern.back() == '*' ? value.compare(0, pattern.size() - 1, pattern, 0, pattern.size() - 1) == 0 : value == pattern;
                                }));
            if(!matchesDevice) {
                applicable = false;
                break;
            }
        }
        if(applicable) {
            messages.push_back(message->get<std::string>());
        }
    }
    return messages;
}

void printStartupNotifications(const std::string& depthaiVersion,
                               Platform platform,
                               XLinkProtocol_t protocol,
                               const std::string& osVersion,
                               const std::string& deviceSKU,
                               const std::atomic<bool>* cancel) {
    const auto url = getEnvAs<std::string>("DEPTHAI_STARTUP_NOTIFICATIONS_URL", "https://depthai-releases.luxonis.com/startup_notifications.json");
    if(url.empty()) {
        return;
    }

    nlohmann::json fetchedJson = getStartupNotifications(url, cancel);
    std::vector<std::string> collectedMessages = collectMessages(fetchedJson, depthaiVersion, platform, protocol, osVersion, deviceSKU);
    for(const auto& message : collectedMessages) {
        if(cancel && cancel->load()) {
            return;
        }
        fmt::print(stderr, "{}\n", message);
    }
}

}  // namespace utility
}  // namespace dai
