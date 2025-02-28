#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/BenchmarkInProperties.hpp>

namespace dai {
namespace node {

class BenchmarkIn : public DeviceNodeCRTP<DeviceNode, BenchmarkIn, BenchmarkInProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "BenchmarkIn";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Receive messages as fast as possible
     */
    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Passthrough for input messages (so the node can be placed between other nodes)
     */
    Output passthrough{*this, {"passthrough", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Send a benchmark report when the set number of messages are received
     */
    Output report{*this, {"report", DEFAULT_GROUP, {{{DatatypeEnum::BenchmarkReport, false}}}}};

    /**
     * Specify how many messages to measure for each report
     */
    void sendReportEveryNMessages(uint32_t n);

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    /**
     * Log the reports as warnings
     */
    void logReportsAsWarnings(bool logReportsAsWarnings);

    /**
     * Attach latencies to the report
     */
    void measureIndividualLatencies(bool attachLatencies);

    void run() override;

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai
