#include "depthai/pipeline/node/SplitterNode.hpp"

#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/MessageGroup.hpp"

namespace dai {
namespace node {

SplitterNode::~SplitterNode() = default;

bool SplitterNode::runOnHost() const {
    return runOnHostVar;
}

void SplitterNode::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

void SplitterNode::run() {
    auto& logger = ThreadedNode::pimpl->logger;

    while(mainLoop()) {
        std::shared_ptr<Buffer> inputBuffer;

        {
            auto inputBlockEvent = this->inputBlockEvent();
            inputBuffer = input.get<Buffer>();
        }

        if(inputBuffer->getDatatype() != DatatypeEnum::MessageGroup) {  // just passthrough
            {
                auto blockEvent = this->outputBlockEvent();
                output.send(inputBuffer);
            }
        } else if(inputBuffer->getDatatype() == DatatypeEnum::MessageGroup) {
            auto messageGroup = std::dynamic_pointer_cast<MessageGroup>(inputBuffer);

            {
                auto blockEvent = this->outputBlockEvent();
                for(auto& entry : messageGroup->group) {
                    if(messageGroup->isLeaf(entry.first)) {
                        auto outputBuffer = std::dynamic_pointer_cast<Buffer>(entry.second);
                        output.send(outputBuffer);
                    }
                }
            }
        } else {
            logger->error("Unsupported datatype {} received in SplitterNode, only MessageGroup is supported.", static_cast<int>(inputBuffer->getDatatype()));
        }
    }
}

}  // namespace node
}  // namespace dai
