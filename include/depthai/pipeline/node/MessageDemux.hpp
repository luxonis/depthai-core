#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/MessageDemuxProperties.hpp"

namespace dai {
namespace node {

class MessageDemux : public DeviceNodeCRTP<DeviceNode, MessageDemux, MessageDemuxProperties> {
   public:
    constexpr static const char* NAME = "MessageDemux";
    using DeviceNodeCRTP::DeviceNodeCRTP;
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
