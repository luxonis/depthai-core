#pragma once

#include "depthai/pipeline/ThreadedHostNode.hpp"

namespace dai {
namespace node {

/**
 * @brief PipelineStateMerge node. Merges PipelineState messages from device and host into a single output.
 */
class PipelineStateMerge : public CustomThreadedNode<PipelineStateMerge> {
    bool hasDeviceNodes = false;
    bool hasHostNodes = false;

    bool allowReconfiguration = true;

   public:
    constexpr static const char* NAME = "PipelineStateMerge";

    Input inputDevice{*this, {"inputDevice", DEFAULT_GROUP, false, 4, {{DatatypeEnum::PipelineState, false}}}};
    Input inputHost{*this, {"inputHost", DEFAULT_GROUP, false, 4, {{DatatypeEnum::PipelineState, false}}}};

    /**
     * Input PipelineEventAggregationConfig message with state request parameters
     */
    Input request{*this, {"request", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::PipelineEventAggregationConfig, false}}}, false}};

    /**
     * Output PipelineEventAggregationConfig message with state request parameters
     */
    Output outRequest{*this, {"outRequest", DEFAULT_GROUP, {{{DatatypeEnum::PipelineEventAggregationConfig, false}}}}};

    /**
     * Output message of type
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::PipelineState, false}}}}};

    std::shared_ptr<PipelineStateMerge> build(bool hasDeviceNodes, bool hasHostNodes);

    PipelineStateMerge& setAllowConfiguration(bool allow);

    void run() override;
};

}  // namespace node
}  // namespace dai
