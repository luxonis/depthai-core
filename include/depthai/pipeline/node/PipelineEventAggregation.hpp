#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/PipelineEventAggregationProperties.hpp>

namespace dai {
namespace node {

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
    InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::PipelineEvent, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * A map of outputs
     */
    OutputMap outputs{*this, "outputs", {DEFAULT_NAME, DEFAULT_GROUP, {{{DatatypeEnum::PipelineEvent, false}}}}};

    /**
     * Output message of type TODO
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, false}}}}};

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
};

}  // namespace node
}  // namespace dai
