#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/internal/PipelineEventAggregationProperties.hpp>

namespace dai {
namespace node {
namespace internal {

/**
 * @brief Sync node. Performs syncing between image frames
 */
class PipelineEventAggregation : public DeviceNodeCRTP<DeviceNode, PipelineEventAggregation, PipelineEventAggregationProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "PipelineEventAggregation";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * A map of inputs
     */
    InputMap inputs{*this, "inputs", {DEFAULT_NAME, DEFAULT_GROUP, true, 32, {{{DatatypeEnum::PipelineEvent, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input PipelineEventAggregationConfig message with state request parameters
     */
    Input request{*this, {"request", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::PipelineEventAggregationConfig, false}}}, false}};

    /**
     * Output message of type PipelineState
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::PipelineState, false}}}}};

    /**
     * Continuous output message of type PipelineState
     */
    Output outTrace{*this, {"outTrace", DEFAULT_GROUP, {{{DatatypeEnum::PipelineState, false}}}}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

    PipelineEventAggregation& setTraceOutput(bool enable);
};

}  // namespace internal
}  // namespace node
}  // namespace dai
