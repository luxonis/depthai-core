#include "depthai/pipeline/node/host/SyncedNode.hpp"

    namespace dai {
    namespace node {
    void SyncedNode::buildStage1() {
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