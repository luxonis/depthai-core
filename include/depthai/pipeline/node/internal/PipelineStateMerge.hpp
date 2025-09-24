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

   public:
    constexpr static const char* NAME = "PipelineStateMerge";

    Input inputDevice{*this, {"inputDevice", DEFAULT_GROUP, false, 4, {{DatatypeEnum::PipelineState, false}}}};
    Input inputHost{*this, {"inputHost", DEFAULT_GROUP, false, 4, {{DatatypeEnum::PipelineState, false}}}};

    std::shared_ptr<PipelineStateMerge> build(bool hasDeviceNodes, bool hasHostNodes);

    /**
     * Output message of type
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::PipelineState, false}}}}};

    void run() override;
};

}  // namespace node
}  // namespace dai
