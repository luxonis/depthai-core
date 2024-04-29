#include "depthai/pipeline/node/host/HostNode.hpp"

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
        // Run the user defined function
        auto out = processGroup(in);
        // Send the output
        if(out) {
            this->out.send(out);
        }
    }
}
}  // namespace node
}  // namespace dai
