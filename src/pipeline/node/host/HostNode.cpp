#include "depthai/pipeline/node/host/HostNode.hpp"

#include <memory>

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace node {
void HostNode::buildStage1() {
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
        std::shared_ptr<dai::MessageGroup> in;
        {
            auto blockEvent = this->inputBlockEvent();
            in = input.get<dai::MessageGroup>();
        }
        // Create a lambda that captures the class as a shared pointer and the message
        // TODO(Morato) - optimize this for performance
        auto processAndSendGroup = [self = std::static_pointer_cast<HostNode>(shared_from_this()), in, evSeqNum]() {
            auto out = self->processGroup(in);

            if(out) {
                auto blockEvent = self->outputBlockEvent();
                self->out.send(out);
            }
            self->pipelineEventDispatcher->endTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", evSeqNum);
        };

        if(sendProcessToPipeline) {
            getParentPipeline().addTask(processAndSendGroup);
        } else {
            processAndSendGroup();
        }
    }
}
}  // namespace node
}  // namespace dai
