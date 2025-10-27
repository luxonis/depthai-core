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
    while(isRunning()) {
        // Get input
        auto in = input.get<dai::MessageGroup>();
        // Create a lambda that captures the class as a shared pointer and the message
        // TODO(Morato) - optimize this for performance
        auto processAndSendGroup = [self = std::static_pointer_cast<HostNode>(shared_from_this()), in]() {
            // Run the user-defined function to process the group
            auto out = self->processGroup(in);

            // Send the output, if there is any
            if(out) {
                self->out.send(out);
            }
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
