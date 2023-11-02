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
     * TODO
     */
    Input input{*this, "input", Input::Type::SReceiver, {{DatatypeEnum::MessageGroup, false}}};

    /**
     * TODO
     */
    OutputMap outputs;
};

}  // namespace node
}  // namespace dai
