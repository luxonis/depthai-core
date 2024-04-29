#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/MessageDemuxProperties.hpp"

namespace dai {
namespace node {

class MessageDemux : public DeviceNodeCRTP<DeviceNode, MessageDemux, MessageDemuxProperties> {
   public:
    constexpr static const char* NAME = "MessageDemux";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    std::shared_ptr<MessageDemux> build() {
        isBuild = true; 
        return std::static_pointer_cast<MessageDemux>(shared_from_this());
    }
    /**
     * Input message of type MessageGroup
     */
    Input input{*this, {.name = "input", .types = {{DatatypeEnum::MessageGroup, false}}}};

    /**
     * A map of outputs, where keys are same as in the input MessageGroup
     */
    OutputMap outputs{*this, "outputs", {.types = {{DatatypeEnum::Buffer, true}}}};
};

}  // namespace node
}  // namespace dai
