#pragma once

#include "depthai/pipeline/Node.hpp"
#include "depthai/properties/MessageDemuxProperties.hpp"

namespace dai {
namespace node {

class MessageDemux : public NodeCRTP<Node, MessageDemux, MessageDemuxProperties> {
   public:
    constexpr static const char* NAME = "MessageDemux";

    /**
     * Input message of type MessageGroup
     */
    Input input{true, *this, "input", Input::Type::SReceiver, {{DatatypeEnum::MessageGroup, false}}};

    /**
     * A map of outputs, where keys are same as in the input MessageGroup
     */
    OutputMap outputs{true, *this, "outputs", Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}})};
};

}  // namespace node
}  // namespace dai
