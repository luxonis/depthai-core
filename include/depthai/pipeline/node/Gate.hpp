#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/datatype/GateControl.hpp"
#include "depthai/properties/GateProperties.hpp"

namespace dai {
namespace node {

class Gate : public DeviceNodeCRTP<DeviceNode, Gate, GateProperties> {
   protected:
    Properties& getProperties() override {
        return properties;
    }

   public:
    constexpr static const char* NAME = "Gate";

    using DeviceNodeCRTP::DeviceNodeCRTP;

    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    Input inputControl{*this, {"inputControl", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::GateControl, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    void setRunOnHost(bool runOnHost);

    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = false;

    std::shared_ptr<GateControl> sendMessages();

    std::shared_ptr<GateControl> sendMessages(int numMessages);

    std::shared_ptr<GateControl> waitFotCommand();
};

}  // namespace node
}  // namespace dai
