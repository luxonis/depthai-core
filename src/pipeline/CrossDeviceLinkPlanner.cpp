#include "pipeline/CrossDeviceLinkPlanner.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <stdexcept>

#include "spdlog/fmt/fmt.h"
#include "utility/Logging.hpp"

namespace dai {

std::optional<CrossDeviceLinkMode> parseCrossDeviceLinkMode(const std::string& value) {
    std::string lower = value;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if(lower == "auto") return CrossDeviceLinkMode::AUTO;
    if(lower == "direct") return CrossDeviceLinkMode::DIRECT;
    if(lower == "relay" || lower == "host_relay") return CrossDeviceLinkMode::HOST_RELAY;
    return std::nullopt;
}

const char* crossDeviceLinkModeName(CrossDeviceLinkMode mode) {
    switch(mode) {
        case CrossDeviceLinkMode::AUTO:
            return "AUTO";
        case CrossDeviceLinkMode::DIRECT:
            return "DIRECT";
        case CrossDeviceLinkMode::HOST_RELAY:
            return "HOST_RELAY";
    }
    return "UNKNOWN";
}

CrossDeviceLinkMode resolveCrossDeviceLinkMode(CrossDeviceLinkMode apiMode, bool apiSetByUser, const std::string& envValue) {
    if(envValue.empty()) {
        return apiMode;
    }
    const auto envMode = parseCrossDeviceLinkMode(envValue);
    if(!envMode) {
        Logging::getInstance().logger.warn(
            "{}='{}' is not valid (use auto, direct or relay), using {}", CROSS_DEVICE_LINK_ENV, envValue, crossDeviceLinkModeName(apiMode));
        return apiMode;
    }
    if(apiSetByUser && *envMode != apiMode) {
        Logging::getInstance().logger.warn("{}={} overrides the cross-device link mode {} set with setCrossDeviceLinkMode()",
                                           CROSS_DEVICE_LINK_ENV,
                                           envValue,
                                           crossDeviceLinkModeName(apiMode));
    } else {
        Logging::getInstance().logger.info("{}={}: cross-device link mode {}", CROSS_DEVICE_LINK_ENV, envValue, crossDeviceLinkModeName(*envMode));
    }
    return *envMode;
}

bool isLoopbackPeerAddress(const std::string& host) {
    std::string lower = host;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return lower == "localhost" || lower == "::1" || lower == "[::1]" || lower.rfind("127.", 0) == 0;
}

std::string hostOfDeviceName(const std::string& name) {
    const auto endpoint = peerlink::parseEndpoint(name);
    return endpoint ? endpoint->host : name;
}

CrossDeviceLinkPlan planCrossDeviceLink(CrossDeviceLinkMode mode,
                                        const CrossDeviceLinkDevice& producer,
                                        const CrossDeviceLinkDevice& consumer,
                                        const CrossDeviceLinkProbe& probe) {
    auto relay = [&](std::string reason) {
        if(mode == CrossDeviceLinkMode::DIRECT) {
            throw std::runtime_error(
                fmt::format("Cannot link device {} to device {} directly (cross-device link mode DIRECT): {}", producer.deviceId, consumer.deviceId, reason));
        }
        CrossDeviceLinkPlan plan;
        plan.reason = std::move(reason);
        return plan;
    };

    if(mode == CrossDeviceLinkMode::HOST_RELAY) {
        return relay("cross-device link mode HOST_RELAY");
    }
    const std::array<const CrossDeviceLinkDevice*, 2> devices{&producer, &consumer};
    for(const auto* device : devices) {
        if(device->platform != Platform::RVC4) {
            return relay(fmt::format("device {} is {}, direct links need RVC4 devices", device->deviceId, platform2string(device->platform)));
        }
    }
    for(const auto* device : devices) {
        if(!device->peerLink) {
            return relay(fmt::format("device {} firmware does not support direct links ({})", device->deviceId, device->peerLinkError));
        }
        if(device->peerLink->protocolVersion != peerlink::PEERLINK_PROTOCOL_VERSION) {
            return relay(fmt::format("device {} uses peer-link protocol version {}, this library uses version {}",
                                     device->deviceId,
                                     device->peerLink->protocolVersion,
                                     peerlink::PEERLINK_PROTOCOL_VERSION));
        }
    }

    // The address the host reaches the producer at is the most likely to work, then the ones the producer reports
    std::vector<std::string> hosts;
    auto addHost = [&hosts](const std::string& host) {
        // A loopback address names the host or the consumer, never the producer; a path is a local socket
        if(!host.empty() && !isLoopbackPeerAddress(host) && host.find('/') == std::string::npos && std::find(hosts.begin(), hosts.end(), host) == hosts.end()) {
            hosts.push_back(host);
        }
    };
    addHost(producer.hostAddress);
    for(const auto& address : producer.peerLink->addresses) {
        addHost(address);
    }
    if(hosts.empty()) {
        return relay(fmt::format("device {} reports no network address", producer.deviceId));
    }

    const auto port = producer.peerLink->port;
    std::vector<std::string> reached;
    std::string tried;
    for(const auto& host : hosts) {
        const auto endpoint = peerlink::formatEndpoint(host, port);
        tried += (tried.empty() ? "" : ", ") + endpoint;
        const auto listenerId = probe ? probe(host, port) : std::nullopt;
        // Another device can answer at an address of the producer's network (same subnet on another LAN)
        if(listenerId && *listenerId == producer.peerLink->listenerId) {
            reached.push_back(endpoint);
        }
    }
    if(reached.empty()) {
        return relay(fmt::format("device {} cannot reach device {} at {}", consumer.deviceId, producer.deviceId, tried));
    }
    CrossDeviceLinkPlan plan;
    plan.direct = true;
    plan.producerEndpoints = std::move(reached);
    plan.producerListenerId = producer.peerLink->listenerId;
    return plan;
}

}  // namespace dai
