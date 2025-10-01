#include "depthai/pipeline/node/internal/PipelineEventAggregation.hpp"

#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"
#include "depthai/utility/CircularBuffer.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {
namespace internal {

class NodeEventAggregation {
   private:
    std::shared_ptr<spdlog::async_logger> logger;

    int windowSize;

   public:
    NodeEventAggregation(int windowSize, std::shared_ptr<spdlog::async_logger> logger) : logger(logger), windowSize(windowSize), eventsBuffer(windowSize) {}
    NodeState state;
    utility::CircularBuffer<NodeState::DurationEvent> eventsBuffer;
    std::unordered_map<PipelineEvent::Type, std::unique_ptr<utility::CircularBuffer<uint64_t>>> timingsBufferByType;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> inputTimingsBuffers;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> outputTimingsBuffers;
    std::unique_ptr<utility::CircularBuffer<uint64_t>> mainLoopTimingsBuffer;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> otherTimingsBuffers;

    std::unordered_map<std::string, std::optional<PipelineEvent>> ongoingInputEvents;
    std::unordered_map<std::string, std::optional<PipelineEvent>> ongoingOutputEvents;
    std::optional<PipelineEvent> ongoingMainLoopEvent;
    std::unordered_map<std::string, std::optional<PipelineEvent>> ongoingOtherEvents;

    uint32_t eventCount = 0;

   private:
    inline bool updateIntervalBuffers(PipelineEvent& event) {
        using namespace std::chrono;
        auto& ongoingEvent = [&]() -> std::optional<PipelineEvent>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    throw std::runtime_error("LOOP event should not be an interval");
                case PipelineEvent::Type::INPUT:
                    return ongoingInputEvents[event.source];
                case PipelineEvent::Type::OUTPUT:
                    return ongoingOutputEvents[event.source];
                case PipelineEvent::Type::CUSTOM:
                    return ongoingOtherEvents[event.source];
            }
            return ongoingMainLoopEvent;  // To silence compiler warning
        }();
        auto& timingsBuffer = [&]() -> std::unique_ptr<utility::CircularBuffer<uint64_t>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    throw std::runtime_error("LOOP event should not be an interval");
                case PipelineEvent::Type::INPUT:
                    if(inputTimingsBuffers.find(event.source) == inputTimingsBuffers.end()) {
                        inputTimingsBuffers[event.source] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
                    }
                    return inputTimingsBuffers[event.source];
                case PipelineEvent::Type::OUTPUT:
                    if(outputTimingsBuffers.find(event.source) == outputTimingsBuffers.end()) {
                        outputTimingsBuffers[event.source] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
                    }
                    return outputTimingsBuffers[event.source];
                case PipelineEvent::Type::CUSTOM:
                    if(otherTimingsBuffers.find(event.source) == otherTimingsBuffers.end()) {
                        otherTimingsBuffers[event.source] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
                    }
                    return otherTimingsBuffers[event.source];
            }
            return mainLoopTimingsBuffer;  // To silence compiler warning
        }();
        if(ongoingEvent.has_value() && ongoingEvent->sequenceNum == event.sequenceNum && event.interval == PipelineEvent::Interval::END) {
            // End event
            NodeState::DurationEvent durationEvent;
            durationEvent.startEvent = *ongoingEvent;
            durationEvent.durationUs = duration_cast<microseconds>(event.getTimestamp() - ongoingEvent->getTimestamp()).count();
            eventsBuffer.add(durationEvent);
            state.events = eventsBuffer.getBuffer();

            if(timingsBufferByType.find(event.type) == timingsBufferByType.end()) {
                timingsBufferByType[event.type] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
            }
            timingsBufferByType[event.type]->add(durationEvent.durationUs);
            timingsBuffer->add(durationEvent.durationUs);

            ongoingEvent = std::nullopt;

            return true;
        } else {
            if(ongoingEvent.has_value()) {
                logger->warn("Ongoing event not finished before new one started. Event source: {}, node {}", ongoingEvent->source, event.nodeId);
            }
            if(event.interval == PipelineEvent::Interval::START) {
                // Start event
                ongoingEvent = event;
            }
            return false;
        }
    }

    inline bool updatePingBuffers(PipelineEvent& event) {
        using namespace std::chrono;
        auto& ongoingEvent = [&]() -> std::optional<PipelineEvent>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    return ongoingMainLoopEvent;
                case PipelineEvent::Type::CUSTOM:
                    return ongoingOtherEvents[event.source];
                case PipelineEvent::Type::INPUT:
                case PipelineEvent::Type::OUTPUT:
                    throw std::runtime_error("INPUT and OUTPUT events should not be pings");
            }
            return ongoingMainLoopEvent;  // To silence compiler warning
        }();
        auto& timingsBuffer = [&]() -> std::unique_ptr<utility::CircularBuffer<uint64_t>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    return mainLoopTimingsBuffer;
                case PipelineEvent::Type::CUSTOM:
                    if(otherTimingsBuffers.find(event.source) == otherTimingsBuffers.end()) {
                        otherTimingsBuffers[event.source] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
                    }
                    return otherTimingsBuffers[event.source];
                case PipelineEvent::Type::INPUT:
                case PipelineEvent::Type::OUTPUT:
                    throw std::runtime_error("INPUT and OUTPUT events should not be pings");
            }
            return mainLoopTimingsBuffer;  // To silence compiler warning
        }();
        if(ongoingEvent.has_value() && ongoingEvent->sequenceNum == event.sequenceNum - 1) {
            // End event
            NodeState::DurationEvent durationEvent;
            durationEvent.startEvent = *ongoingEvent;
            durationEvent.durationUs = duration_cast<microseconds>(event.getTimestamp() - ongoingEvent->getTimestamp()).count();
            eventsBuffer.add(durationEvent);
            state.events = eventsBuffer.getBuffer();

            if(timingsBufferByType.find(event.type) == timingsBufferByType.end()) {
                timingsBufferByType[event.type] = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
            }
            if(timingsBuffer == nullptr) {
                timingsBuffer = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
            }
            timingsBufferByType[event.type]->add(durationEvent.durationUs);
            timingsBuffer->add(durationEvent.durationUs);

            // Start event
            ongoingEvent = event;

            return true;
        } else if(ongoingEvent.has_value()) {
            logger->warn("Ongoing main loop event not finished before new one started. Event source: {}, node {}", ongoingEvent->source, event.nodeId);
        }
        // Start event
        ongoingEvent = event;

        return false;
    }

    inline void updateTimingStats(NodeState::TimingStats& stats, const utility::CircularBuffer<uint64_t>& buffer) {
        stats.minMicros = std::min(stats.minMicros, buffer.last());
        stats.maxMicros = std::max(stats.maxMicros, buffer.last());
        stats.averageMicrosRecent = 0;
        stats.stdDevMicrosRecent = 0;

        auto bufferByType = buffer.getBuffer();
        uint64_t sum = 0;
        double variance = 0;
        for(auto v : bufferByType) {
            sum += v;
        }
        stats.averageMicrosRecent = sum / bufferByType.size();
        // Calculate standard deviation
        for(auto v : bufferByType) {
            auto diff = v - stats.averageMicrosRecent;
            variance += diff * diff;
        }
        variance /= bufferByType.size();
        stats.stdDevMicrosRecent = (uint64_t)(std::sqrt(variance));

        std::sort(bufferByType.begin(), bufferByType.end());
        stats.minMicrosRecent = bufferByType.front();
        stats.maxMicrosRecent = bufferByType.back();
        stats.medianMicrosRecent = bufferByType[bufferByType.size() / 2];
        if(bufferByType.size() % 2 == 0) {
            stats.medianMicrosRecent = (stats.medianMicrosRecent + bufferByType[bufferByType.size() / 2 - 1]) / 2;
        }
    }

   public:
    void add(PipelineEvent& event) {
        using namespace std::chrono;
        ++eventCount;
        bool recalculateStats = false;
        if(event.interval == PipelineEvent::Interval::NONE) {
            recalculateStats = updatePingBuffers(event);
        } else {
            recalculateStats = updateIntervalBuffers(event);
        }
        if(recalculateStats) {
            // By type
            updateTimingStats(state.timingsByType[event.type], *timingsBufferByType[event.type]);
            // By instance
            switch(event.type) {
                case PipelineEvent::Type::CUSTOM:
                    updateTimingStats(state.otherStats[event.source], *otherTimingsBuffers[event.source]);
                    break;
                case PipelineEvent::Type::LOOP:
                    updateTimingStats(state.mainLoopStats, *mainLoopTimingsBuffer);
                    break;
                case PipelineEvent::Type::INPUT:
                    updateTimingStats(state.inputStates[event.source].timingStats, *inputTimingsBuffers[event.source]);
                    break;
                case PipelineEvent::Type::OUTPUT:
                    updateTimingStats(state.outputStates[event.source].timingStats, *outputTimingsBuffers[event.source]);
                    break;
            }
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
    auto& logger = pimpl->logger;
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
                    nodeStates.insert_or_assign(event->nodeId, NodeEventAggregation(properties.aggregationWindowSize, logger));
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

}  // namespace internal
}  // namespace node
}  // namespace dai
