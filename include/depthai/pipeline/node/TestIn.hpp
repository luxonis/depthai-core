#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai-shared/properties/TestInProperties.hpp>

namespace dai {
namespace node {

class TestIn : public NodeCRTP<DeviceNode, TestIn, TestInProperties> {
   public:
    constexpr static const char* NAME = "TestIn";
    using NodeCRTP::NodeCRTP;

    /**
     *  Send messages out as fast as possible
     */
    Output out{true, *this, "out", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Message to send
    */
    Input input{true, *this, "input", Input::Type::SReceiver, true, 1, {{DatatypeEnum::Buffer, true}}};

    /**
     * Sets number of messages in pool
     * @param num number of messages in pool
     */
    void setNumMessagesToSend(int num);
};

}  // namespace node
}  // namespace dai
