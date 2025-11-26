#include "depthai/pipeline/node/internal/PipelineStateMerge.hpp"

#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {

std::shared_ptr<PipelineStateMerge> PipelineStateMerge::build(bool hasDeviceNodes, bool hasHostNodes) {
    this->hasDeviceNodes = hasDeviceNodes;
    this->hasHostNodes = hasHostNodes;
    return std::static_pointer_cast<PipelineStateMerge>(shared_from_this());
}

PipelineStateMerge& PipelineStateMerge::setAllowConfiguration(bool allow) {
    this->allowConfiguration = allow;
    return *this;
}

void mergeStates(std::shared_ptr<PipelineState>& outState, const std::shared_ptr<PipelineState>& inState) {
    for(const auto& [key, value] : inState->nodeStates) {
        if(outState->nodeStates.find(key) != outState->nodeStates.end()) {
            throw std::runtime_error("PipelineStateMerge: Duplicate node state for nodeId " + std::to_string(key));
        } else
            outState->nodeStates[key] = value;
    }
}
void PipelineStateMerge::run() {
    auto& logger = pimpl->logger;

    this->pipelineEventDispatcher->sendEvents = false;

    if(!hasDeviceNodes && !hasHostNodes) {
        logger->warn("PipelineStateMerge: both device and host nodes are disabled. Have you built the node?");
    }
    std::optional<PipelineEventAggregationConfig> currentConfig;
    uint32_t sequenceNum = 0;
    uint32_t configSequenceNum = 0;
    while(mainLoop()) {
        auto outState = std::make_shared<PipelineState>();
        bool waitForMatch = false;
        if(allowConfiguration && (!currentConfig.has_value() || (currentConfig.has_value() && !currentConfig->repeatIntervalSeconds.has_value()) || request.has())) {
            auto req = request.get<PipelineEventAggregationConfig>();
            if(req != nullptr) {
                currentConfig = *req;
                currentConfig->setSequenceNum(++configSequenceNum);
                outRequest.send(std::make_shared<PipelineEventAggregationConfig>(currentConfig.value()));
                waitForMatch = true;
            }
        }
        if(hasDeviceNodes) {
            auto deviceState = inputDevice.get<dai::PipelineState>();
            if(waitForMatch && deviceState != nullptr && currentConfig.has_value()) {
                while(isRunning() && deviceState->configSequenceNum != currentConfig->sequenceNum) {
                    deviceState = inputDevice.get<dai::PipelineState>();
                }
            }
            if(deviceState != nullptr) {
                *outState = *deviceState;
            }
        }
        if(hasHostNodes) {
            auto hostState = inputHost.get<dai::PipelineState>();
            if(waitForMatch && hostState != nullptr && currentConfig.has_value()) {
                while(isRunning() && hostState->configSequenceNum != currentConfig->sequenceNum) {
                    hostState = inputHost.get<dai::PipelineState>();
                    if(!isRunning()) break;
                }
            }
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
