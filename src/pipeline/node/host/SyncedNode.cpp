#include "depthai/pipeline/node/host/SyncedNode.hpp"

    namespace dai {
    namespace node {
    void SyncedNode::buildStage1() {
        // If the user has been explicit about the sync node, set it
        if(syncOnHost.has_value()) {
            sync->setRunOnHost(syncOnHost.value());
        } else {
            sync->setRunOnHost(true);
        }
        sync->out.link(input);
    }

    void SyncedNode::run() {
        while(isRunning()) {
            // Get input
            auto in = input.get<dai::MessageGroup>();
            // Run the user defined function
            auto out = runOnce(in);
            // Send the output
            if(out) {
                this->out.send(out);
            }
        }
    }
    }  // namespace node
}  // namespace dai