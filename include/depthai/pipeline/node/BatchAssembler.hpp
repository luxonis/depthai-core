#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/BatchItem.hpp"
#include "depthai/pipeline/datatype/MessageBatch.hpp"
#include "depthai/properties/BatchAssemblerProperties.hpp"

namespace dai {
namespace node {

class BatchAssembler : public DeviceNodeCRTP<DeviceNode, BatchAssembler, BatchAssemblerProperties>, public HostRunnable {
   public:
    static constexpr const char* NAME = "BatchAssembler";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    BatchAssembler() = default;
    BatchAssembler(std::unique_ptr<Properties> props);

    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::BatchItem, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::MessageBatch, false}}}}};

    void setRunOnHost(bool runOnHost);
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = true;
};

}  // namespace node
}  // namespace dai
