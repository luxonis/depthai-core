#include "depthai/beta/BetaNode.hpp"

#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace beta {

void BetaNode::buildStage1() {
    auto device = getDevice();
    if(device && device->getPlatform() == Platform::RVC2 && !runOnHost()) {
        setRunOnHost(true);
        ThreadedNode::pimpl->logger->info("Beta node '{}' cannot run on-device on RVC2. Running on host.", getName());
    }
}

}  // namespace beta
}  // namespace dai
