#include "depthai/pipeline/node/internal/PipelineStateMerge.hpp"

#include "depthai/pipeline/datatype/PipelineState.hpp"

#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {

std::shared_ptr<PipelineStateMerge> PipelineStateMerge::build(bool hasDeviceNodes, bool hasHostNodes) {
    this->hasDeviceNodes = hasDeviceNodes;
    this->hasHostNodes = hasHostNodes;
    return std::static_pointer_cast<PipelineStateMerge>(shared_from_this());
}

void mergeStates(std::shared_ptr<PipelineState>& outState, const std::shared_ptr<PipelineState>& inState) {
    for(const auto& [key, value] : inState->nodeStates) {
        if(outState->nodeStates.find(key) != outState->nodeStates.end()) {
            throw std::runtime_error("PipelineStateMerge: Duplicate node state for nodeId " + std::to_string(key));
        } else outState->nodeStates[key] = value;
    }
}
void PipelineStateMerge::run() {
    auto& logger = pimpl->logger;
    if(!hasDeviceNodes && !hasHostNodes) {
        logger->warn("PipelineStateMerge: both device and host nodes are disabled. Have you built the node?");
    }
    uint32_t sequenceNum = 0;
    while(isRunning()) {
        auto outState = std::make_shared<PipelineState>();
        if(hasDeviceNodes) {
            auto deviceState = inputDevice.get<dai::PipelineState>();
            if(deviceState != nullptr) {
                *outState = *deviceState;
            }
        }
        if(hasHostNodes) {
            auto hostState = inputHost.get<dai::PipelineState>();
            if(hostState != nullptr) {
                if(hasDeviceNodes) {
                    // merge
                    mergeStates(outState, hostState);
                    auto minTimestamp = std::min(outState->getTimestamp(), hostState->getTimestamp());
                    outState->setTimestamp(minTimestamp);
                    outState->sequenceNum = sequenceNum++;
                } else {
                    *outState = *hostState;
                }
            }
        }
        out.send(outState);
    }
}
}  // namespace node
}  // namespace dai
