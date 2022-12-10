#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai-shared/properties/TestInProperties.hpp>

namespace dai {
namespace node {

class TestOut : public NodeCRTP<DeviceNode, TestOut, TestInProperties> {
   public:
    constexpr static const char* NAME = "TestOut";
    using NodeCRTP::NodeCRTP;

    /**
     * Receive messages as fast as possible
    */
    Input input{true, *this, "input", Input::Type::SReceiver, true, 4, {{DatatypeEnum::Buffer, true}}};

};

}  // namespace node
}  // namespace dai
