#include "depthai/pipeline/node/BatchAssembler.hpp"

#include <optional>
#include <stdexcept>
#include <vector>

namespace dai {
namespace node {

BatchAssembler::BatchAssembler(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, BatchAssembler, BatchAssemblerProperties>(std::move(props)) {}

void BatchAssembler::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool BatchAssembler::runOnHost() const {
    return runOnHostVar;
}

void BatchAssembler::run() {
    std::shared_ptr<BatchItem> pending;

    while(mainLoop()) {
        std::shared_ptr<BatchItem> first = pending;
        pending.reset();
        if(!first) {
            auto blockEvent = inputBlockEvent();
            first = input.get<BatchItem>();
        }

        if(!first) {
            continue;
        }

        // A zero-sized BatchItem represents an empty batch. Forward it as an
        // empty MessageBatch while preserving the batch metadata.
        if(first->batchSize == 0) {
            auto emptyBatch = std::make_shared<MessageBatch>();
            emptyBatch->setBufferMetadataFrom(first);
            {
                auto blockEvent = outputBlockEvent();
                output.send(emptyBatch);
            }
            continue;
        }

        std::vector<std::shared_ptr<Buffer>> gathered(first->batchSize);
        std::vector<bool> received(first->batchSize, false);
        std::size_t receivedCount = 0;
        const auto expectedBatchIndex = first->batchIndex;
        const auto expectedBatchSize = first->batchSize;
        std::optional<std::chrono::steady_clock::time_point> commonTimestamp;
        bool timestampsMatch = true;

        auto storePayload = [&](const std::shared_ptr<BatchItem>& iterable) {
            if(!iterable) {
                throw std::runtime_error("Expected BatchItem message while collecting iterable batch");
            }
            if(iterable->batchIndex != expectedBatchIndex) {
                pending = iterable;
                return false;
            }
            if(iterable->batchSize != expectedBatchSize) {
                throw std::runtime_error("Encountered BatchItem item with a different batchSize while collecting iterable batch");
            }
            if(iterable->itemIndex >= gathered.size()) {
                throw std::runtime_error("BatchItem itemIndex exceeds declared batchSize");
            }

            const auto timestamp = iterable->getTimestamp();
            if(commonTimestamp && *commonTimestamp != timestamp) {
                timestampsMatch = false;
            } else if(!commonTimestamp) {
                commonTimestamp = timestamp;
            }

            auto payload = iterable->getPayload();
            if(payload && !std::dynamic_pointer_cast<Buffer>(payload)) {
                throw std::runtime_error("BatchAssembler node only supports BatchItem payloads derived from Buffer");
            }

            gathered[iterable->itemIndex] = iterable->getPayload<Buffer>();
            if(!received[iterable->itemIndex]) {
                received[iterable->itemIndex] = true;
                receivedCount++;
            }

            return receivedCount < gathered.size();
        };

        if(storePayload(first)) {
            for(std::uint32_t collected = 1; collected < expectedBatchSize; collected++) {
                std::shared_ptr<BatchItem> iterable;
                {
                    auto blockEvent = inputBlockEvent();
                    iterable = input.get<BatchItem>();
                }

                if(!storePayload(iterable)) {
                    break;
                }
            }
        }

        auto bufferVector = std::make_shared<MessageBatch>(std::move(gathered));
        for(const auto& buffer : bufferVector->getBuffers()) {
            if(buffer) {
                bufferVector->setSequenceNum(buffer->getSequenceNum());
                break;
            }
        }
        if(timestampsMatch && commonTimestamp) {
            bufferVector->setTimestamp(*commonTimestamp);
        }

        {
            auto blockEvent = outputBlockEvent();
            output.send(bufferVector);
        }
    }
}

}  // namespace node
}  // namespace dai
