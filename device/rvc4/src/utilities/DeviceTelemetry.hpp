#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "XLink/XLinkPublicDefines.h"

namespace dai {

class DeviceTelemetry {
   public:
    using AggregateMetricsCallback = std::function<void(nlohmann::json&)>;

    static DeviceTelemetry& getInstance();

    DeviceTelemetry(const DeviceTelemetry&) = delete;
    DeviceTelemetry& operator=(const DeviceTelemetry&) = delete;

    void start();
    void stop();

    void event(std::string eventName, nlohmann::json properties = {});  // Prefix depthai-only events with depthai_; do not prefix global (stack-wide) events

    void addAggregateMetrics(AggregateMetricsCallback functionLikeCb);

   private:
    DeviceTelemetry() = default;
    ~DeviceTelemetry();

    bool openStreamLocked(std::size_t maxWriteSize);
    void closeStreamLocked();
    void applyAggregateMetrics(nlohmann::json& properties);

    std::mutex mtx;
    std::mutex aggregateMetricsMtx;
    std::vector<std::shared_ptr<AggregateMetricsCallback>> aggregateMetricsCallbacks;
    streamId_t streamId{INVALID_STREAM_ID};
};

}  // namespace dai
