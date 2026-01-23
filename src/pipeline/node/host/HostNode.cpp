#include "depthai/pipeline/node/host/HostNode.hpp"

#include <memory>

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace node {
void HostNode::buildStage1() {
    // If the user has been explicit about the sync node, set it
    if(syncOnHost.has_value()) {
        sync->setRunOnHost(syncOnHost.value());
    } else {
        sync->setRunOnHost(true);
    }
    sync->out.link(input);
}

void HostNode::run() {
    volatile uint32_t eventSequenceNum = 0;
    while(isRunning()) {
        uint32_t evSeqNum = eventSequenceNum++;
        this->pipelineEventDispatcher->startTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", evSeqNum);
        // Get input
        std::shared_ptr<dai::MessageGroup> in;
        {
            auto blockEvent = this->inputBlockEvent();
            in = input.get<dai::MessageGroup>();
        }
        // Create a lambda that captures the class as a shared pointer and the message
        // TODO(Morato) - optimize this for performance
        auto processAndSendGroup = [self = std::static_pointer_cast<HostNode>(shared_from_this()), in, evSeqNum]() {
            // Run the user-defined function to process the group
            auto out = self->processGroup(in);

            // Send the output, if there is any
            if(out) {
                auto blockEvent = self->outputBlockEvent();
                self->out.send(out);
            }
            self->pipelineEventDispatcher->endTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", evSeqNum);
        };

        if(sendProcessToPipeline) {
            // Send the processing to the pipeline
            getParentPipeline().addTask(processAndSendGroup);
        } else {
            // Process and send the group
            processAndSendGroup();
        }
    }
}
}  // namespace node
}  // namespace dai
