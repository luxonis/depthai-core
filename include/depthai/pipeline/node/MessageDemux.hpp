#pragma once

#include "depthai-shared/properties/MessageDemuxProperties.hpp"
#include "depthai/pipeline/Node.hpp"

namespace dai {
namespace node {

class MessageDemux : public NodeCRTP<Node, MessageDemux, MessageDemuxProperties> {
   public:
    constexpr static const char* NAME = "MessageDemux";
    MessageDemux(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    MessageDemux(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Input message of type MessageGroup
     */
    Input input{*this, "input", Input::Type::SReceiver, {{DatatypeEnum::MessageGroup, false}}};

    /**
     * A map of outputs, where keys are same as in the input MessageGroup
     */
    OutputMap outputs;
};

}  // namespace node
}  // namespace dai
