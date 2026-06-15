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

        if(inputBuffer->getDatatype() == DatatypeEnum::MissingDataMessage) {
            continue;
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
                for(const auto& nodeIdx : messageGroup->breadthFirstOrder(0)) {
                    if(messageGroup->isLeaf(nodeIdx)) {
                        auto buffer_msg = std::dynamic_pointer_cast<Buffer>(messageGroup->getNode(nodeIdx));
                        if(buffer_msg != nullptr && buffer_msg->getDatatype() != DatatypeEnum::MissingDataMessage) {
                            // logger->warn("Sending buffer with node index {} and datatype {} with size: {}",
                            //              nodeIdx,
                            //              (int)buffer_msg->getDatatype(),
                            //              buffer_msg->getData().size());
                            output.send(buffer_msg);
                        }
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
