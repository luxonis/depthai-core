#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

namespace dai {
class DeviceBase;
class PipelineImpl;
}  // namespace dai

namespace dai {
namespace utility {

class Telemetry {
   public:
    using AggregateMetricsCallback = std::function<void(nlohmann::json&)>;
    using AggregateMetricsHandle = std::uint64_t;

    static Telemetry& getInstance();
    static std::string getTemporaryTelemetryDeviceId(const std::string& mxid);
    static bool isTelemetryEnabled();
    static void setTelemetryUsesPython(bool value);
    void init();

    Telemetry(const Telemetry&) = delete;
    Telemetry& operator=(const Telemetry&) = delete;
    Telemetry(Telemetry&&) = delete;
    Telemetry& operator=(Telemetry&&) = delete;

    void event(std::string eventName, nlohmann::json properties);  // Prefix depthai-only events with depthai_, not global events

    void event(const DeviceBase& device, std::string eventName, nlohmann::json properties);

    void event(const PipelineImpl& pipeline, std::string eventName, nlohmann::json properties);  // Nodes: use the device overload (last owner joins itself)

    AggregateMetricsHandle addAggregateMetrics(AggregateMetricsCallback functionLikeCb);

    void removeAggregateMetrics(AggregateMetricsHandle handle);

   private:
    Telemetry();
    ~Telemetry();

    class Impl;
    std::unique_ptr<Impl> impl;
};

}  // namespace utility
}  // namespace dai
