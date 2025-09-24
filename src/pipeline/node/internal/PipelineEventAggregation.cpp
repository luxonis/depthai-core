#include "depthai/pipeline/node/internal/PipelineEventAggregation.hpp"

#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"
#include "depthai/utility/CircularBuffer.hpp"

namespace dai {
namespace node {

class NodeEventAggregation {
    int windowSize;

   public:
    NodeEventAggregation(int windowSize) : windowSize(windowSize), eventsBuffer(windowSize) {}
    NodeState state;
    utility::CircularBuffer<PipelineEvent> eventsBuffer;
    std::unordered_map<PipelineEvent::EventType, std::unique_ptr<utility::CircularBuffer<uint64_t>>> timingsBufferByType;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> timingsBufferByInstance;

    uint32_t eventCount = 0;

    void add(PipelineEvent& event) {
        // TODO optimize avg
        ++eventCount;
        eventsBuffer.add(event);
        state.events = eventsBuffer.getBuffer();
        if(timingsBufferByType.find(event.type) == timingsBufferByType.end()) {
            timingsBufferByType[event.type] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
        }
        if(timingsBufferByInstance.find(event.source) == timingsBufferByInstance.end()) {
            timingsBufferByInstance[event.source] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
        }
        timingsBufferByType[event.type]->add(event.duration);
        timingsBufferByInstance[event.source]->add(event.duration);
        // Calculate average duration and standard deviation from buffers
        state.timingsByType[event.type].averageMicros = 0;
        state.timingsByType[event.type].stdDevMicros = 0;
        state.timingsByInstance[event.source].averageMicros = 0;
        state.timingsByInstance[event.source].stdDevMicros = 0;
        auto bufferByType = timingsBufferByType[event.type]->getBuffer();
        auto bufferByInstance = timingsBufferByInstance[event.source]->getBuffer();
        if(!bufferByType.empty()) {
            uint64_t sum = 0;
            double variance = 0;
            for(auto v : bufferByType) {
                sum += v;
            }
            state.timingsByType[event.type].averageMicros = sum / bufferByType.size();
            // Calculate standard deviation
            for(auto v : bufferByType) {
                auto diff = v - state.timingsByType[event.type].averageMicros;
                variance += diff * diff;
            }
            variance /= bufferByType.size();
            state.timingsByType[event.type].stdDevMicros = (uint64_t)(std::sqrt(variance));
        }
        if(!bufferByInstance.empty()) {
            uint64_t sum = 0;
            double variance = 0;
            for(auto v : bufferByInstance) {
                sum += v;
            }
            state.timingsByInstance[event.source].averageMicros = sum / bufferByInstance.size();
            // Calculate standard deviation
            for(auto v : bufferByInstance) {
                auto diff = v - state.timingsByInstance[event.source].averageMicros;
                variance += diff * diff;
            }
            variance /= bufferByInstance.size();
            state.timingsByInstance[event.source].stdDevMicros = (uint64_t)(std::sqrt(variance));
        }
    }
};

void PipelineEventAggregation::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool PipelineEventAggregation::runOnHost() const {
    return runOnHostVar;
}

void PipelineEventAggregation::run() {
    std::unordered_map<int64_t, NodeEventAggregation> nodeStates;
    uint32_t sequenceNum = 0;
    while(isRunning()) {
        std::unordered_map<std::string, std::shared_ptr<PipelineEvent>> events;
        for(auto& [k, v] : inputs) {
            events[k.second] = v.get<PipelineEvent>();
        }
        for(auto& [k, event] : events) {
            if(event != nullptr) {
                if(nodeStates.find(event->nodeId) == nodeStates.end()) {
                    nodeStates.insert_or_assign(event->nodeId, NodeEventAggregation(properties.aggregationWindowSize));
                }
                nodeStates.at(event->nodeId).add(*event);
            }
        }
        auto outState = std::make_shared<PipelineState>();
        bool shouldSend = false;
        for(auto& [nodeId, nodeState] : nodeStates) {
            if(nodeState.eventCount >= properties.eventBatchSize) {
                outState->nodeStates[nodeId] = nodeState.state;
                if(!properties.sendEvents) outState->nodeStates[nodeId].events.clear();
                shouldSend = true;
                nodeState.eventCount = 0;
            }
        }
        outState->sequenceNum = sequenceNum++;
        outState->setTimestamp(std::chrono::steady_clock::now());
        outState->tsDevice = outState->ts;
        if(shouldSend) out.send(outState);
    }
}
}  // namespace node
}  // namespace dai
