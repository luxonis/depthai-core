#include "DeviceTelemetry.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <thread>
#include <utility>

#include <nlohmann/json.hpp>

#include "XLink/XLink.h"
#include "depthai/xlink/XLinkConstants.hpp"
#include "utilities/ErrorMacros.hpp"

namespace dai {

namespace {

constexpr int STREAM_OPEN_RETRIES = 5;
constexpr auto WAIT_FOR_STREAM_RETRY = std::chrono::milliseconds(50);

}  // namespace

DeviceTelemetry& DeviceTelemetry::getInstance() {
    static DeviceTelemetry telemetry;
    return telemetry;
}

DeviceTelemetry::~DeviceTelemetry() {
    stop();
}

void DeviceTelemetry::start() {
    std::lock_guard<std::mutex> lock(mtx);
    (void)openStreamLocked(device::XLINK_USB_BUFFER_MAX_SIZE);
}

void DeviceTelemetry::stop() {
    std::lock_guard<std::mutex> lock(mtx);
    closeStreamLocked();
}

void DeviceTelemetry::event(std::string eventName, nlohmann::json properties) {
    if(eventName.empty()) {
        return;
    }
    if(!properties.is_object()) {
        properties = nlohmann::json::object();
    }
    if(eventName == "depthai_rvc4_device_usage") {
        applyAggregateMetrics(properties);
        if(!properties.is_object()) {
            properties = nlohmann::json::object();
        }
    }

    const auto payload = nlohmann::json{
        {"event", eventName},
        {"properties", std::move(properties)},
    };
    const auto serialized = payload.dump();

    std::lock_guard<std::mutex> lock(mtx);
    if(streamId == INVALID_STREAM_ID && !openStreamLocked(std::max<std::size_t>(serialized.size(), 4096))) {
        DAI_WARN("Failed to open telemetry stream for event '{}'", eventName);
        return;
    }

    const auto status = XLinkWriteData(streamId, reinterpret_cast<const uint8_t*>(serialized.data()), serialized.size());
    if(status != X_LINK_SUCCESS) {
        DAI_WARN("Telemetry write failed for event '{}' with status {}", eventName, static_cast<int>(status));
        closeStreamLocked();
    }
}

void DeviceTelemetry::addAggregateMetrics(AggregateMetricsCallback functionLikeCb) {
    if(!functionLikeCb) {
        return;
    }

    std::lock_guard<std::mutex> lock(aggregateMetricsMtx);
    aggregateMetricsCallbacks.push_back(std::make_shared<AggregateMetricsCallback>(std::move(functionLikeCb)));
}

void DeviceTelemetry::applyAggregateMetrics(nlohmann::json& properties) {
    std::vector<std::shared_ptr<AggregateMetricsCallback>> callbacks;
    {
        std::lock_guard<std::mutex> lock(aggregateMetricsMtx);
        callbacks = aggregateMetricsCallbacks;
    }

    for(const auto& callback : callbacks) {
        try {
            if(callback && *callback) {
                (*callback)(properties);
            }
        } catch(const std::exception& ex) {
            DAI_WARN("Device telemetry aggregate metrics callback failed: {}", ex.what());
        } catch(...) {
            DAI_WARN("Device telemetry aggregate metrics callback failed with an unknown exception");
        }
    }
}

bool DeviceTelemetry::openStreamLocked(std::size_t maxWriteSize) {
    if(streamId != INVALID_STREAM_ID) {
        return true;
    }

    for(int retryCount = 0; retryCount < STREAM_OPEN_RETRIES; ++retryCount) {
        streamId = XLinkOpenStream(0, device::XLINK_CHANNEL_TELEMETRY, static_cast<int>(maxWriteSize));
        if(streamId != INVALID_STREAM_ID) {
            return true;
        }
        std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);
    }

    streamId = INVALID_STREAM_ID;
    return false;
}

void DeviceTelemetry::closeStreamLocked() {
    if(streamId != INVALID_STREAM_ID) {
        XLinkCloseStream(streamId);
        streamId = INVALID_STREAM_ID;
    }
}

}  // namespace dai
