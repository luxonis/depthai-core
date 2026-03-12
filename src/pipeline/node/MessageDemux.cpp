#include "depthai/pipeline/node/MessageDemux.hpp"

#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {

void MessageDemux::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool MessageDemux::runOnHost() const {
    return runOnHostVar;
}

void MessageDemux::run() {
    auto& logger = pimpl->logger;

    while(mainLoop()) {
        std::shared_ptr<MessageGroup> message = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            message = input.get<dai::MessageGroup>();
        }
        if(!message) {
            logger->error("Received message is not a message group - skipping");
            continue;
        }
        {
            auto blockEvent = this->outputBlockEvent();

            // Check that the outputs match the messages
            for(auto& output : outputs) {
                auto messageName = output.first.second;
                auto messageToSend = message->get(messageName);
                if(messageToSend == nullptr) {
                    logger->error("Message group does not contain message with name: {}", output.first.second);
                    continue;
                } else {
                    output.second.send(messageToSend);
                }
            }
        }
    }
}

void MessageDemux::setProcessor(ProcessorType proc) {
    properties.processor = proc;
}

ProcessorType MessageDemux::getProcessor() const {
    return properties.processor;
}

}  // namespace node
}  // namespace dai
