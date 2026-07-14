#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/BatchItem.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/TestNodeProperties.hpp"

namespace dai {
namespace node {

class TestNode : public DeviceNodeCRTP<DeviceNode, TestNode, TestNodeProperties>, public HostRunnable {
   public:
    static constexpr const char* NAME = "TestNode";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    TestNode() = default;
    TestNode(std::unique_ptr<Properties> props);

    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::BatchItem, false}}}}};

    void setRunOnHost(bool runOnHost);
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = true;
};

}  // namespace node
}  // namespace dai
