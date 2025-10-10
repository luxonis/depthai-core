#include "depthai/pipeline/node/internal/PipelineEventAggregation.hpp"

#include <chrono>
#include <shared_mutex>

#include "depthai/pipeline/datatype/PipelineEvent.hpp"
#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"
#include "depthai/pipeline/datatype/PipelineState.hpp"
#include "depthai/utility/CircularBuffer.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace node {
namespace internal {

class NodeEventAggregation {
   private:
    struct FpsMeasurement {
        std::chrono::steady_clock::time_point time;
        int64_t sequenceNum;
    };

    std::shared_ptr<spdlog::async_logger> logger;

    uint32_t windowSize;
    uint32_t eventBatchSize;
    uint32_t eventWaitWindow;

   public:
    NodeEventAggregation(uint32_t windowSize, uint32_t eventBatchSize, uint32_t eventWaitWindow, std::shared_ptr<spdlog::async_logger> logger)
        : logger(logger), windowSize(windowSize), eventBatchSize(eventBatchSize), eventWaitWindow(eventWaitWindow), eventsBuffer(windowSize) {}
    NodeState state;
    utility::CircularBuffer<NodeState::DurationEvent> eventsBuffer;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> inputTimingsBuffers;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> outputTimingsBuffers;
    std::unique_ptr<utility::CircularBuffer<uint64_t>> inputsGetTimingsBuffer;
    std::unique_ptr<utility::CircularBuffer<uint64_t>> outputsSendTimingsBuffer;
    std::unique_ptr<utility::CircularBuffer<uint64_t>> mainLoopTimingsBuffer;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint64_t>>> otherTimingsBuffers;

    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<uint32_t>>> inputQueueSizesBuffers;

    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<FpsMeasurement>>> inputFpsBuffers;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<FpsMeasurement>>> outputFpsBuffers;
    std::unique_ptr<utility::CircularBuffer<FpsMeasurement>> inputsGetFpsBuffer;
    std::unique_ptr<utility::CircularBuffer<FpsMeasurement>> outputsSendFpsBuffer;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<FpsMeasurement>>> otherFpsBuffers;

    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>>> ongoingInputEvents;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>>> ongoingOutputEvents;
    std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>> ongoingGetInputsEvents;
    std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>> ongoingSendOutputsEvents;
    std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>> ongoingMainLoopEvents;
    std::unordered_map<std::string, std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>>> ongoingOtherEvents;

    uint32_t count = 0;

   private:
    inline std::optional<PipelineEvent>* findOngoingEvent(uint32_t sequenceNum, utility::CircularBuffer<std::optional<PipelineEvent>>& buffer) {
        for(auto rit = buffer.rbegin(); rit != buffer.rend(); ++rit) {
            if(rit->has_value() && rit->value().sequenceNum == sequenceNum) {
                return &(*rit);
            } else if(rit->has_value() && rit->value().sequenceNum < sequenceNum) {
                break;
            }
        }
        return nullptr;
    }

    inline bool updateIntervalBuffers(PipelineEvent& event) {
        using namespace std::chrono;
        std::unique_ptr<utility::CircularBuffer<uint64_t>> emptyIntBuffer;
        std::unique_ptr<utility::CircularBuffer<FpsMeasurement>> emptyTimeBuffer;

        auto& ongoingEvents = [&]() -> std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    throw std::runtime_error("LOOP event should not be an interval");
                case PipelineEvent::Type::INPUT:
                    return ongoingInputEvents[event.source];
                case PipelineEvent::Type::OUTPUT:
                    return ongoingOutputEvents[event.source];
                case PipelineEvent::Type::CUSTOM:
                    return ongoingOtherEvents[event.source];
                case PipelineEvent::Type::INPUT_BLOCK:
                    return ongoingGetInputsEvents;
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    return ongoingSendOutputsEvents;
            }
            return ongoingMainLoopEvents;  // To silence compiler warning
        }();
        auto& timingsBuffer = [&]() -> std::unique_ptr<utility::CircularBuffer<uint64_t>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    throw std::runtime_error("LOOP event should not be an interval");
                case PipelineEvent::Type::INPUT:
                    return inputTimingsBuffers[event.source];
                case PipelineEvent::Type::OUTPUT:
                    return outputTimingsBuffers[event.source];
                case PipelineEvent::Type::CUSTOM:
                    return otherTimingsBuffers[event.source];
                case PipelineEvent::Type::INPUT_BLOCK:
                    return inputsGetTimingsBuffer;
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    return outputsSendTimingsBuffer;
            }
            return emptyIntBuffer;  // To silence compiler warning
        }();
        auto& fpsBuffer = [&]() -> std::unique_ptr<utility::CircularBuffer<FpsMeasurement>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    throw std::runtime_error("LOOP event should not be an interval");
                case PipelineEvent::Type::INPUT:
                    return inputFpsBuffers[event.source];
                case PipelineEvent::Type::OUTPUT:
                    return outputFpsBuffers[event.source];
                case PipelineEvent::Type::CUSTOM:
                    return otherFpsBuffers[event.source];
                case PipelineEvent::Type::INPUT_BLOCK:
                    return inputsGetFpsBuffer;
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    return outputsSendFpsBuffer;
            }
            return emptyTimeBuffer;  // To silence compiler warning
        }();

        if(ongoingEvents == nullptr) ongoingEvents = std::make_unique<utility::CircularBuffer<std::optional<PipelineEvent>>>(eventWaitWindow);
        if(timingsBuffer == nullptr) timingsBuffer = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
        if(fpsBuffer == nullptr) fpsBuffer = std::make_unique<utility::CircularBuffer<FpsMeasurement>>(windowSize);

        auto* ongoingEvent = findOngoingEvent(event.sequenceNum, *ongoingEvents);

        if(ongoingEvent && ongoingEvent->has_value() && event.interval == PipelineEvent::Interval::END) {
            // End event
            NodeState::DurationEvent durationEvent;
            durationEvent.startEvent = ongoingEvent->value();
            durationEvent.durationUs = duration_cast<microseconds>(event.getTimestamp() - ongoingEvent->value().getTimestamp()).count();
            eventsBuffer.add(durationEvent);

            timingsBuffer->add(durationEvent.durationUs);
            fpsBuffer->add({durationEvent.startEvent.getTimestamp(), durationEvent.startEvent.getSequenceNum()});

            *ongoingEvent = std::nullopt;

            return true;
        } else if(event.interval == PipelineEvent::Interval::START) {
            // Start event
            ongoingEvents->add(event);
        }
        return false;
    }

    inline bool updatePingBuffers(PipelineEvent& event) {
        using namespace std::chrono;
        std::unique_ptr<utility::CircularBuffer<uint64_t>> emptyIntBuffer;
        std::unique_ptr<utility::CircularBuffer<FpsMeasurement>> emptyTimeBuffer;

        auto& ongoingEvents = [&]() -> std::unique_ptr<utility::CircularBuffer<std::optional<PipelineEvent>>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    return ongoingMainLoopEvents;
                case PipelineEvent::Type::CUSTOM:
                    return ongoingOtherEvents[event.source];
                case PipelineEvent::Type::INPUT:
                case PipelineEvent::Type::OUTPUT:
                case PipelineEvent::Type::INPUT_BLOCK:
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    throw std::runtime_error("INPUT and OUTPUT events should not be pings");
            }
            return ongoingMainLoopEvents;  // To silence compiler warning
        }();
        auto& timingsBuffer = [&]() -> std::unique_ptr<utility::CircularBuffer<uint64_t>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    return mainLoopTimingsBuffer;
                case PipelineEvent::Type::CUSTOM:
                    return otherTimingsBuffers[event.source];
                case PipelineEvent::Type::INPUT:
                case PipelineEvent::Type::OUTPUT:
                case PipelineEvent::Type::INPUT_BLOCK:
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    throw std::runtime_error("INPUT and OUTPUT events should not be pings");
            }
            return emptyIntBuffer;  // To silence compiler warning
        }();
        auto& fpsBuffer = [&]() -> std::unique_ptr<utility::CircularBuffer<FpsMeasurement>>& {
            switch(event.type) {
                case PipelineEvent::Type::LOOP:
                    break;
                case PipelineEvent::Type::CUSTOM:
                    return otherFpsBuffers[event.source];
                case PipelineEvent::Type::INPUT:
                case PipelineEvent::Type::OUTPUT:
                case PipelineEvent::Type::INPUT_BLOCK:
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    throw std::runtime_error("INPUT and OUTPUT events should not be pings");
            }
            return emptyTimeBuffer;  // To silence compiler warning
        }();

        if(ongoingEvents == nullptr) ongoingEvents = std::make_unique<utility::CircularBuffer<std::optional<PipelineEvent>>>(eventWaitWindow);
        if(timingsBuffer == nullptr) timingsBuffer = std::make_unique<utility::CircularBuffer<uint64_t>>(windowSize);
        if(fpsBuffer == nullptr) fpsBuffer = std::make_unique<utility::CircularBuffer<FpsMeasurement>>(windowSize);

        auto* ongoingEvent = findOngoingEvent(event.sequenceNum - 1, *ongoingEvents);

        if(ongoingEvent && ongoingEvent->has_value()) {
            // End event
            NodeState::DurationEvent durationEvent;
            durationEvent.startEvent = ongoingEvent->value();
            durationEvent.durationUs = duration_cast<microseconds>(event.getTimestamp() - ongoingEvent->value().getTimestamp()).count();
            eventsBuffer.add(durationEvent);

            timingsBuffer->add(durationEvent.durationUs);
            if(fpsBuffer) fpsBuffer->add({durationEvent.startEvent.getTimestamp(), durationEvent.startEvent.getSequenceNum()});

            // Start event
            ongoingEvents->add(event);

            return true;
        }
        // Start event
        ongoingEvents->add(event);

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
    inline void updateFpsStats(NodeState::Timing& timing, const utility::CircularBuffer<FpsMeasurement>& buffer) {
        if(buffer.size() < 2) return;
        auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(buffer.last().time - buffer.first().time).count();
        auto frameDiff = buffer.last().sequenceNum - buffer.first().sequenceNum;
        if(timeDiff > 0 && buffer.last().sequenceNum > buffer.first().sequenceNum) {
            timing.fps = frameDiff * (1e6f / (float)timeDiff);
        }
    }

   public:
    void add(PipelineEvent& event) {
        using namespace std::chrono;
        if(event.type == PipelineEvent::Type::INPUT && event.interval == PipelineEvent::Interval::END) {
            if(event.queueSize.has_value()) {
                if(inputQueueSizesBuffers.find(event.source) == inputQueueSizesBuffers.end()) {
                    inputQueueSizesBuffers[event.source] = std::make_unique<utility::CircularBuffer<uint32_t>>(windowSize);
                }
                inputQueueSizesBuffers[event.source]->add(*event.queueSize);
            } else {
                throw std::runtime_error(fmt::format("INPUT END event must have queue size set source: {}, node {}", event.source, event.nodeId));
            }
        }
        // Update states
        switch(event.type) {
            case PipelineEvent::Type::CUSTOM:
            case PipelineEvent::Type::LOOP:
                break;
            case PipelineEvent::Type::INPUT:
                if(event.queueSize.has_value()) state.inputStates[event.source].numQueued = *event.queueSize;
                switch(event.interval) {
                    case PipelineEvent::Interval::START:
                        state.inputStates[event.source].state = NodeState::InputQueueState::State::WAITING;
                        break;
                    case PipelineEvent::Interval::END:
                        state.inputStates[event.source].state = NodeState::InputQueueState::State::IDLE;
                        break;
                    case PipelineEvent::Interval::NONE:
                        if(event.status == -1 || event.status == -2) state.inputStates[event.source].state = NodeState::InputQueueState::State::BLOCKED;
                        break;
                }
                break;
            case PipelineEvent::Type::OUTPUT:
                if(event.interval == PipelineEvent::Interval::START)
                    state.outputStates[event.source].state = NodeState::OutputQueueState::State::SENDING;
                else if(event.interval == PipelineEvent::Interval::END)
                    state.outputStates[event.source].state = NodeState::OutputQueueState::State::IDLE;
                break;
            case PipelineEvent::Type::INPUT_BLOCK:
                if(event.interval == PipelineEvent::Interval::START)
                    state.state = NodeState::State::GETTING_INPUTS;
                else if(event.interval == PipelineEvent::Interval::END)
                    state.state = NodeState::State::PROCESSING;
                break;
            case PipelineEvent::Type::OUTPUT_BLOCK:
                if(event.interval == PipelineEvent::Interval::START)
                    state.state = NodeState::State::SENDING_OUTPUTS;
                else if(event.interval == PipelineEvent::Interval::END)
                    state.state = NodeState::State::PROCESSING;
                break;
        }
        bool addedEvent = false;
        if(event.interval == PipelineEvent::Interval::NONE && event.type != PipelineEvent::Type::INPUT && event.type != PipelineEvent::Type::OUTPUT) {
            addedEvent = updatePingBuffers(event);
        } else if(event.interval != PipelineEvent::Interval::NONE) {
            addedEvent = updateIntervalBuffers(event);
        }
        if(addedEvent /*  && ++count % eventBatchSize == 0 */) {  // TODO
            // By instance
            switch(event.type) {
                case PipelineEvent::Type::CUSTOM:
                    updateTimingStats(state.otherTimings[event.source].durationStats, *otherTimingsBuffers[event.source]);
                    updateFpsStats(state.otherTimings[event.source], *otherFpsBuffers[event.source]);
                    break;
                case PipelineEvent::Type::LOOP:
                    updateTimingStats(state.mainLoopTiming.durationStats, *mainLoopTimingsBuffer);
                    state.mainLoopTiming.fps = 1e6f / (float)state.mainLoopTiming.durationStats.averageMicrosRecent;
                    break;
                case PipelineEvent::Type::INPUT:
                    updateTimingStats(state.inputStates[event.source].timing.durationStats, *inputTimingsBuffers[event.source]);
                    updateFpsStats(state.inputStates[event.source].timing, *inputFpsBuffers[event.source]);
                    break;
                case PipelineEvent::Type::OUTPUT:
                    updateTimingStats(state.outputStates[event.source].timing.durationStats, *outputTimingsBuffers[event.source]);
                    updateFpsStats(state.outputStates[event.source].timing, *outputFpsBuffers[event.source]);
                    break;
                case PipelineEvent::Type::INPUT_BLOCK:
                    updateTimingStats(state.inputsGetTiming.durationStats, *inputsGetTimingsBuffer);
                    updateFpsStats(state.inputsGetTiming, *inputsGetFpsBuffer);
                    break;
                case PipelineEvent::Type::OUTPUT_BLOCK:
                    updateTimingStats(state.outputsSendTiming.durationStats, *outputsSendTimingsBuffer);
                    updateFpsStats(state.outputsSendTiming, *outputsSendFpsBuffer);
                    break;
            }
        }
        if(event.type == PipelineEvent::Type::INPUT && event.interval == PipelineEvent::Interval::END /*  && ++count % eventBatchSize == 0 */) {  // TODO
            auto& qStats = state.inputStates[event.source].queueStats;
            auto& qBuffer = *inputQueueSizesBuffers[event.source];
            qStats.maxQueued = std::max(qStats.maxQueued, *event.queueSize);
            auto qBufferData = qBuffer.getBuffer();
            std::sort(qBufferData.begin(), qBufferData.end());
            qStats.minQueuedRecent = qBufferData.front();
            qStats.maxQueuedRecent = qBufferData.back();
            qStats.medianQueuedRecent = qBufferData[qBufferData.size() / 2];
            if(qBufferData.size() % 2 == 0) {
                qStats.medianQueuedRecent = (qStats.medianQueuedRecent + qBufferData[qBufferData.size() / 2 - 1]) / 2;
            }
        }
    }
};

class PipelineEventHandler {
    std::unordered_map<int64_t, NodeEventAggregation> nodeStates;
    Node::InputMap* inputs;
    uint32_t aggregationWindowSize;
    uint32_t eventBatchSize;
    uint32_t eventWaitWindow;

    std::atomic<bool> running;

    std::thread thread;

    std::shared_ptr<spdlog::async_logger> logger;

    std::shared_mutex mutex;

   public:
    PipelineEventHandler(
        Node::InputMap* inputs, uint32_t aggregationWindowSize, uint32_t eventBatchSize, uint32_t eventWaitWindow, std::shared_ptr<spdlog::async_logger> logger)
        : inputs(inputs), aggregationWindowSize(aggregationWindowSize), eventBatchSize(eventBatchSize), eventWaitWindow(eventWaitWindow), logger(logger) {}
    void threadedRun() {
        while(running) {
            std::unordered_map<std::string, std::shared_ptr<PipelineEvent>> events;
            bool gotEvents = false;
            for(auto& [k, v] : *inputs) {
                events[k.second] = v.tryGet<PipelineEvent>();
                gotEvents = gotEvents || (events[k.second] != nullptr);
            }
            for(auto& [k, event] : events) {
                if(event != nullptr) {
                    if(nodeStates.find(event->nodeId) == nodeStates.end()) {
                        nodeStates.insert_or_assign(event->nodeId, NodeEventAggregation(aggregationWindowSize, eventBatchSize, eventWaitWindow, logger));
                    }
                    {
                        std::unique_lock lock(mutex);
                        nodeStates.at(event->nodeId).add(*event);
                    }
                }
            }
            if(!gotEvents) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    void run() {
        running = true;
        thread = std::thread(&PipelineEventHandler::threadedRun, this);
    }
    void stop() {
        running = false;
        if(thread.joinable()) thread.join();
    }
    bool getState(std::shared_ptr<PipelineState> outState, bool sendEvents) {
        std::shared_lock lock(mutex);
        bool updated = false;
        for(auto& [nodeId, nodeState] : nodeStates) {
            outState->nodeStates[nodeId] = nodeState.state;
            if(sendEvents) outState->nodeStates[nodeId].events = nodeState.eventsBuffer.getBuffer();
            if(nodeState.count % eventBatchSize == 0) updated = true;
        }
        return updated;
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

    PipelineEventHandler handler(&inputs, properties.aggregationWindowSize, properties.eventBatchSize, properties.eventWaitWindow, logger);
    handler.run();

    std::optional<PipelineEventAggregationConfig> currentConfig;
    uint32_t sequenceNum = 0;
    while(mainLoop()) {
        auto outState = std::make_shared<PipelineState>();
        bool gotConfig = false;
        if(!currentConfig.has_value() || (currentConfig.has_value() && !currentConfig->repeat) || request.has()) {
            auto req = request.get<PipelineEventAggregationConfig>();
            if(req != nullptr) {
                currentConfig = *req;
                gotConfig = true;
            }
        }
        if(gotConfig || (currentConfig.has_value() && currentConfig->repeat)) {
            bool sendEvents = false;
            if(currentConfig.has_value()) {
                for(const auto& nodeCfg : currentConfig->nodes) {
                    if(nodeCfg.events) {
                        sendEvents = true;
                        break;
                    }
                }
            }
            bool updated = handler.getState(outState, sendEvents);
            outState->sequenceNum = sequenceNum++;
            outState->configSequenceNum = currentConfig.has_value() ? currentConfig->sequenceNum : 0;
            outState->setTimestamp(std::chrono::steady_clock::now());
            outState->tsDevice = outState->ts;

            if(!currentConfig->nodes.empty()) {
                for(auto it = outState->nodeStates.begin(); it != outState->nodeStates.end();) {
                    auto nodeConfig = std::find_if(currentConfig->nodes.begin(), currentConfig->nodes.end(), [&](const NodeEventAggregationConfig& cfg) {
                        return cfg.nodeId == it->first;
                    });
                    if(nodeConfig == currentConfig->nodes.end()) {
                        it = outState->nodeStates.erase(it);
                    } else {
                        if(nodeConfig->inputs.has_value()) {
                            auto inputStates = it->second.inputStates;
                            it->second.inputStates.clear();
                            for(const auto& inputName : *nodeConfig->inputs) {
                                if(inputStates.find(inputName) != inputStates.end()) {
                                    it->second.inputStates[inputName] = inputStates[inputName];
                                }
                            }
                        }
                        if(nodeConfig->outputs.has_value()) {
                            auto outputStates = it->second.outputStates;
                            it->second.outputStates.clear();
                            for(const auto& outputName : *nodeConfig->outputs) {
                                if(outputStates.find(outputName) != outputStates.end()) {
                                    it->second.outputStates[outputName] = outputStates[outputName];
                                }
                            }
                        }
                        if(nodeConfig->others.has_value()) {
                            auto otherTimings = it->second.otherTimings;
                            it->second.otherTimings.clear();
                            for(const auto& otherName : *nodeConfig->others) {
                                if(otherTimings.find(otherName) != otherTimings.end()) {
                                    it->second.otherTimings[otherName] = otherTimings[otherName];
                                }
                            }
                        }
                        ++it;
                    }
                }
            }
            if(gotConfig || (currentConfig.has_value() && currentConfig->repeat && updated)) out.send(outState);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    handler.stop();
}

}  // namespace internal
}  // namespace node
}  // namespace dai
