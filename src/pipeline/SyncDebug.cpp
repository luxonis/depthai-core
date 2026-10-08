#include "pipeline/SyncDebug.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <unordered_map>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/CircularBuffer.hpp"

namespace dai {
namespace detail {
namespace syncdebug {
namespace {

constexpr std::size_t HISTORY_CAPACITY = 512;
enum class Stage { ARRIVED, ENQUEUED, EVICTED, CONSUMED, DISCARDED, EMITTED, GROUP_ENQUEUED, GROUP_EVICTED, DEQUEUED, COUNT };
constexpr std::array<const char*, static_cast<std::size_t>(Stage::COUNT)> STAGE_NAMES{
    "arrival", "enqueue", "eviction", "sync_consumption", "sync_discard", "group_emission", "group_enqueue", "group_eviction", "group_dequeue"};

std::int64_t steadyNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Frame {
    std::int64_t sequence = 0;
    std::int64_t deviceNs = 0;
    std::int64_t hostNs = 0;
    std::optional<std::int64_t> systemNs;
    std::int64_t exposureUs = 0;
    int fsync = 0;
    bool valid = false;
};

Frame metadata(const ADatatype& message) {
    Frame frame;
    const auto* buffer = dynamic_cast<const Buffer*>(&message);
    if(!buffer) return frame;
    frame.valid = true;
    frame.sequence = buffer->getSequenceNum();
    frame.deviceNs = std::chrono::duration_cast<std::chrono::nanoseconds>(buffer->getTimestampDevice().time_since_epoch()).count();
    frame.hostNs = std::chrono::duration_cast<std::chrono::nanoseconds>(buffer->getTimestamp().time_since_epoch()).count();
    if(const auto ts = buffer->getTimestampSystem()) {
        frame.systemNs = std::chrono::duration_cast<std::chrono::nanoseconds>(ts->time_since_epoch()).count();
    }
    if(const auto* image = dynamic_cast<const ImgFrame*>(&message)) {
        frame.exposureUs = image->getExposureTime().count();
        frame.fsync = static_cast<int>(image->getFsync());
    }
    return frame;
}

struct Event {
    std::int64_t steadyNs = 0;
    std::uint64_t groupId = 0;
    Frame frame;
    const char* reason = "";
    std::int64_t spreadNs = 0;
    std::int64_t thresholdNs = 0;
};

struct History {
    utility::CircularBuffer<Event> events{HISTORY_CAPACITY};
    std::uint64_t total = 0;
    void add(Event event) {
        ++total;
        events.add(std::move(event));
    }
};

struct Stream {
    explicit Stream(Input input) : input(std::move(input)) {}
    Input input;
    std::array<History, static_cast<std::size_t>(Stage::COUNT)> histories;
};

struct Queue {
    MessageQueue* address = nullptr;
    std::string name;
    unsigned capacity = 0;
    bool blocking = false;
    std::size_t highWatermark = 0;
    std::uint64_t arrivals = 0;
    std::uint64_t enqueued = 0;
    std::uint64_t evictions = 0;
    std::uint64_t rejected = 0;
    std::uint64_t zeroCapacityDiscards = 0;
    std::uint64_t pushCompletions = 0;
};

struct GroupIdentity {
    std::weak_ptr<ADatatype> message;
    std::uint64_t id = 0;
};

struct GroupEvent {
    std::int64_t steadyNs = 0;
    std::uint64_t id = 0;
    Stage stage = Stage::EMITTED;
    const char* phase = "setup";
    std::optional<std::size_t> sampleIndex;
};

struct ContextEvent {
    std::int64_t steadyNs = 0;
    const char* phase = "setup";
    std::optional<std::size_t> sampleIndex;
    std::uint64_t lastDequeuedGroupId = 0;
    std::optional<std::int64_t> gapNs;
    std::optional<double> limitSec;
};

struct Registry {
    std::atomic<bool> active{false};
    std::mutex mutex;
    std::unordered_map<const MessageQueue*, Handle> queues;
};

Registry& registry() {
    static Registry instance;
    return instance;
}

nlohmann::json frameJson(const Frame& frame) {
    nlohmann::json value = {{"valid_buffer", frame.valid},
                            {"sequence", frame.sequence},
                            {"device_ns", frame.deviceNs},
                            {"host_timestamp_ns", frame.hostNs},
                            {"exposure_us", frame.exposureUs},
                            {"fsync", frame.fsync}};
    value["system_ns"] = frame.systemNs ? nlohmann::json(*frame.systemNs) : nlohmann::json(nullptr);
    return value;
}

}  // namespace

struct State {
    std::mutex mutex;
    std::atomic<bool> frozen{false};
    bool dumped = false;
    std::atomic<std::uint64_t> recordingErrors{0};
    std::vector<Stream> streams;
    std::vector<Queue> queues;
    utility::CircularBuffer<GroupIdentity> identities{HISTORY_CAPACITY};
    // Separate stage rings prevent eviction/dequeue events from shortening the emitted history.
    std::array<utility::CircularBuffer<GroupEvent>, 4> groups{utility::CircularBuffer<GroupEvent>(HISTORY_CAPACITY),
                                                              utility::CircularBuffer<GroupEvent>(HISTORY_CAPACITY),
                                                              utility::CircularBuffer<GroupEvent>(HISTORY_CAPACITY),
                                                              utility::CircularBuffer<GroupEvent>(HISTORY_CAPACITY)};
    std::array<std::uint64_t, 4> groupTotals{};
    std::uint64_t nextGroupId = 0;
    std::uint64_t unmatchedGroupIds = 0;
    std::uint64_t lastDequeuedGroupId = 0;
    utility::CircularBuffer<ContextEvent> contexts{HISTORY_CAPACITY};
    std::uint64_t contextTotal = 0;
    std::int64_t startedSteadyNs = steadyNs();
    std::int64_t startedSystemNs = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::int64_t frozenSteadyNs = 0;
    const char* phase = "setup";
    std::optional<std::size_t> sampleIndex;
    std::optional<std::int64_t> gapNs;
    std::optional<double> limitSec;
    nlohmann::json configuration;

    std::uint64_t identify(const std::shared_ptr<ADatatype>& message) {
        for(const auto& identity : identities) {
            if(identity.message.lock() == message) return identity.id;
        }
        ++unmatchedGroupIds;
        return 0;
    }

    void record(Stage stage,
                std::size_t stream,
                const ADatatype& message,
                std::uint64_t id,
                const char* reason = "",
                std::int64_t spread = 0,
                std::int64_t threshold = 0,
                std::optional<std::int64_t> time = std::nullopt) {
        streams.at(stream).histories[static_cast<std::size_t>(stage)].add({time.value_or(steadyNs()), id, metadata(message), reason, spread, threshold});
    }

    void recordGroup(Stage stage, const std::shared_ptr<ADatatype>& message, std::optional<std::int64_t> time = std::nullopt) {
        const auto* group = dynamic_cast<const MessageGroup*>(message.get());
        if(!group) return;
        const auto id = stage == Stage::EMITTED ? ++nextGroupId : identify(message);
        if(stage == Stage::EMITTED) identities.add({message, id});
        if(stage == Stage::DEQUEUED) lastDequeuedGroupId = id;
        const std::size_t slot = static_cast<std::size_t>(stage) - static_cast<std::size_t>(Stage::EMITTED);
        groups.at(slot).add({time.value_or(steadyNs()), id, stage, phase, sampleIndex});
        ++groupTotals.at(slot);
        for(std::size_t i = 0; i < streams.size(); ++i) {
            const auto it = group->group.find(streams[i].input.name);
            if(it != group->group.end() && it->second) record(stage, i, *it->second, id, "", 0, 0, time);
        }
    }
};

namespace {
// Diagnostics cannot turn an otherwise successful SDK operation into a failure.
template <typename F>
void update(const Handle& handle, F&& operation) noexcept {
    if(!handle || handle.state->frozen.load(std::memory_order_relaxed)) return;
    try {
        std::lock_guard<std::mutex> lock(handle.state->mutex);
        if(!handle.state->frozen.load(std::memory_order_relaxed)) operation(*handle.state);
    } catch(...) {
        ++handle.state->recordingErrors;
    }
}

nlohmann::json queueJson(const Queue& queue) {
    return {{"name", queue.name},
            {"capacity", queue.capacity},
            {"blocking", queue.blocking},
            {"high_watermark", queue.highWatermark},
            {"arrivals", queue.arrivals},
            {"enqueued", queue.enqueued},
            {"evictions", queue.evictions},
            {"rejected", queue.rejected},
            {"zero_capacity_discards", queue.zeroCapacityDiscards},
            {"push_completions_recorded", queue.pushCompletions},
            {"pushes_without_completion_record", queue.arrivals - queue.pushCompletions}};
}

nlohmann::json historyJson(const History& history, bool success) {
    nlohmann::json result = {{"total", history.total}, {"history_overwrites", history.total - history.events.size()}};
    if(success) return result;
    result["events"] = nlohmann::json::array();
    for(const auto& event : history.events.getBuffer()) {
        auto entry = frameJson(event.frame);
        entry["steady_ns"] = event.steadyNs;
        entry["group_id"] = event.groupId;
        entry["reason"] = event.reason;
        entry["candidate_spread_ns"] = event.spreadNs;
        entry["grouping_threshold_ns"] = event.thresholdNs;
        result["events"].push_back(std::move(entry));
    }
    return result;
}

nlohmann::json groupHistoriesJson(const State& state, bool success) {
    auto result = nlohmann::json::array();
    for(std::size_t i = 0; i < state.groups.size(); ++i) {
        nlohmann::json value = {{"stage", STAGE_NAMES[static_cast<std::size_t>(Stage::EMITTED) + i]},
                                {"total", state.groupTotals[i]},
                                {"history_overwrites", state.groupTotals[i] - state.groups[i].size()}};
        if(!success) {
            value["events"] = nlohmann::json::array();
            for(const auto& event : state.groups[i].getBuffer()) {
                nlohmann::json entry = {{"group_id", event.id}, {"steady_ns", event.steadyNs}, {"phase", event.phase}};
                entry["sample_index"] = event.sampleIndex ? nlohmann::json(*event.sampleIndex) : nlohmann::json(nullptr);
                value["events"].push_back(std::move(entry));
            }
        }
        result.push_back(std::move(value));
    }
    return result;
}

nlohmann::json contextHistoryJson(const State& state) {
    auto result = nlohmann::json::array();
    for(const auto& event : state.contexts.getBuffer()) {
        nlohmann::json value = {{"steady_ns", event.steadyNs}, {"phase", event.phase}, {"last_dequeued_group_id", event.lastDequeuedGroupId}};
        value["sample_index"] = event.sampleIndex ? nlohmann::json(*event.sampleIndex) : nlohmann::json(nullptr);
        value["gap_ns"] = event.gapNs ? nlohmann::json(*event.gapNs) : nlohmann::json(nullptr);
        value["limit_seconds"] = event.limitSec ? nlohmann::json(*event.limitSec) : nlohmann::json(nullptr);
        result.push_back(std::move(value));
    }
    return result;
}

nlohmann::json snapshot(const State& state, bool success, const char* reason) {
    nlohmann::json result = {{"schema_version", 1},
                             {"success", success},
                             {"reason", reason},
                             {"phase", state.phase},
                             {"started_system_ns", state.startedSystemNs},
                             {"started_steady_ns", state.startedSteadyNs},
                             {"frozen_steady_ns", state.frozenSteadyNs},
                             {"history_capacity", HISTORY_CAPACITY},
                             {"recording_errors", state.recordingErrors.load()},
                             {"unmatched_group_ids", state.unmatchedGroupIds},
                             {"configuration", state.configuration}};
    result["sample_index"] = state.sampleIndex ? nlohmann::json(*state.sampleIndex) : nlohmann::json(nullptr);
    result["gap_ns"] = state.gapNs ? nlohmann::json(*state.gapNs) : nlohmann::json(nullptr);
    result["limit_seconds"] = state.limitSec ? nlohmann::json(*state.limitSec) : nlohmann::json(nullptr);
    result["queues"] = nlohmann::json::array();
    for(const auto& queue : state.queues) result["queues"].push_back(queueJson(queue));
    result["streams"] = nlohmann::json::array();
    for(std::size_t i = 0; i < state.streams.size(); ++i) {
        const auto& stream = state.streams[i];
        nlohmann::json value = {{"id", i}, {"name", stream.input.name}, {"device_id", stream.input.deviceId}};
        for(std::size_t stage = 0; stage < STAGE_NAMES.size(); ++stage) {
            value[STAGE_NAMES[stage]] = historyJson(stream.histories[stage], success);
        }
        result["streams"].push_back(std::move(value));
    }
    result["groups"] = groupHistoriesJson(state, success);
    result["context_history_overwrites"] = state.contextTotal - state.contexts.size();
    if(!success) result["contexts"] = contextHistoryJson(state);
    return result;
}
}  // namespace

bool requested() noexcept {
    const char* enabled = std::getenv("DEPTHAI_SYNC_DEBUG");
    return enabled && enabled[0] == '1' && enabled[1] == '\0';
}

Handle find(const MessageQueue* queue) noexcept {
    auto& entries = registry();
    if(!entries.active.load(std::memory_order_relaxed)) return {};
    try {
        std::lock_guard<std::mutex> lock(entries.mutex);
        const auto it = entries.queues.find(queue);
        return it == entries.queues.end() ? Handle{} : it->second;
    } catch(...) {
        return {};
    }
}

void arrival(const Handle& handle, const std::shared_ptr<ADatatype>& message) noexcept {
    update(handle, [&](State& state) {
        ++state.queues.at(handle.queueIndex).arrivals;
        if(handle.queueIndex < state.streams.size() && message) state.record(Stage::ARRIVED, handle.queueIndex, *message, 0);
    });
}

void pushed(const Handle& handle, const std::shared_ptr<ADatatype>& message, const SyncDebugQueueAccess::Diagnostics& diagnostics, bool accepted) noexcept {
    update(handle, [&](State& state) {
        auto& queue = state.queues.at(handle.queueIndex);
        queue.capacity = diagnostics.capacity;
        queue.blocking = diagnostics.blocking;
        queue.highWatermark = std::max({queue.highWatermark, diagnostics.sizeBefore, diagnostics.sizeAfter});
        ++queue.pushCompletions;
        queue.evictions += diagnostics.evictionCount;
        state.recordingErrors += diagnostics.captureErrors;
        queue.rejected += !accepted;
        queue.zeroCapacityDiscards += diagnostics.discardedIncoming;
        const bool output = handle.queueIndex == state.streams.size();
        const auto time =
            accepted ? std::optional<std::int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(diagnostics.completedAt.time_since_epoch()).count())
                     : std::nullopt;
        for(const auto& evicted : diagnostics.evicted) {
            if(!evicted) continue;
            if(output)
                state.recordGroup(Stage::GROUP_EVICTED, evicted, time);
            else
                state.record(Stage::EVICTED, handle.queueIndex, *evicted, 0, "queue_overwrite", 0, 0, time);
        }
        if(accepted && !diagnostics.discardedIncoming && message) {
            ++queue.enqueued;
            if(output)
                state.recordGroup(Stage::GROUP_ENQUEUED, message, time);
            else
                state.record(Stage::ENQUEUED, handle.queueIndex, *message, 0, "", 0, 0, time);
        }
    });
}

void consumed(const Handle& handle, const std::shared_ptr<ADatatype>& message) noexcept {
    update(handle, [&](State& state) {
        if(message) state.record(Stage::CONSUMED, handle.queueIndex, *message, 0);
    });
}

void discarded(const Handle& handle,
               const std::shared_ptr<ADatatype>& message,
               const char* reason,
               std::chrono::nanoseconds spread,
               std::chrono::nanoseconds threshold) noexcept {
    update(handle, [&](State& state) {
        if(message) state.record(Stage::DISCARDED, handle.queueIndex, *message, 0, reason, spread.count(), threshold.count());
    });
}

void emitted(const Handle& output, const std::shared_ptr<ADatatype>& group) noexcept {
    update(output, [&](State& state) { state.recordGroup(Stage::EMITTED, group); });
}

void dequeued(const Handle& output, const std::shared_ptr<ADatatype>& group) noexcept {
    update(output, [&](State& state) { state.recordGroup(Stage::DEQUEUED, group); });
}

void context(const Handle& handle,
             const char* phase,
             std::optional<std::size_t> sampleIndex,
             std::optional<std::chrono::system_clock::duration> gap,
             std::optional<double> limitSec,
             bool freeze) noexcept {
    update(handle, [&](State& state) {
        state.phase = phase;
        state.sampleIndex = sampleIndex;
        state.gapNs = gap ? std::optional<std::int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(*gap).count()) : std::nullopt;
        state.limitSec = limitSec;
        state.contexts.add({steadyNs(), phase, sampleIndex, state.lastDequeuedGroupId, state.gapNs, limitSec});
        ++state.contextTotal;
        if(freeze) {
            state.frozen = true;
            state.frozenSteadyNs = steadyNs();
        }
    });
}

Session::Session(std::shared_ptr<State> state) : state(std::move(state)) {}

std::unique_ptr<Session> Session::create(const std::vector<Input>& inputs, MessageQueue& output, nlohmann::json configuration) {
    if(!requested()) return nullptr;
    auto state = std::make_shared<State>();
    state->configuration = std::move(configuration);
    state->streams.reserve(inputs.size());
    state->queues.reserve(inputs.size() + 1);
    for(const auto& input : inputs) {
        if(!input.queue) throw std::invalid_argument("Sync debug input queue is null");
        state->streams.emplace_back(input);
        state->queues.push_back({input.queue, input.name, input.queue->getMaxSize(), input.queue->getBlocking()});
    }
    state->queues.push_back({&output, "sync_output", output.getMaxSize(), output.getBlocking()});
    auto session = std::make_unique<Session>(state);
    auto& entries = registry();
    std::lock_guard<std::mutex> lock(entries.mutex);
    for(std::size_t i = 0; i < state->queues.size(); ++i) {
        if(entries.queues.count(state->queues[i].address)) throw std::logic_error("Queue already registered for sync diagnostics");
    }
    for(std::size_t i = 0; i < state->queues.size(); ++i) entries.queues.emplace(state->queues[i].address, Handle{state, i});
    entries.active.store(true, std::memory_order_relaxed);
    return session;
}

Session::~Session() {
    auto& entries = registry();
    std::lock_guard<std::mutex> lock(entries.mutex);
    for(const auto& queue : state->queues) {
        const auto it = entries.queues.find(queue.address);
        if(it != entries.queues.end() && it->second.state == state) entries.queues.erase(it);
    }
    entries.active.store(!entries.queues.empty(), std::memory_order_relaxed);
}

void Session::freeze() noexcept {
    try {
        std::lock_guard<std::mutex> lock(state->mutex);
        if(!state->frozen) {
            state->frozen = true;
            state->frozenSteadyNs = steadyNs();
        }
    } catch(...) {
        ++state->recordingErrors;
    }
}

void Session::finish(bool success, const char* reason) noexcept {
    freeze();
    try {
        nlohmann::json document;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            if(state->dumped) return;
            state->dumped = true;
            document = snapshot(*state, success, reason);
        }
        const char* directory = std::getenv("DEPTHAI_SYNC_DEBUG_DIR");
        const std::filesystem::path parent = directory && *directory ? directory : "sync-debug";
        std::filesystem::create_directories(parent);
        const auto path =
            parent
            / ("sync-" + std::to_string(state->startedSystemNs) + "-" + std::to_string(state->startedSteadyNs) + (success ? "-success.json" : "-failure.json"));
        std::ofstream file(path);
        file.exceptions(std::ios::failbit | std::ios::badbit);
        file << document.dump(2) << '\n';
        file.close();
        std::cerr << "Sync debug artifact: " << path << '\n';
    } catch(const std::exception& ex) {
        std::cerr << "Unable to write sync debug artifact: " << ex.what() << '\n';
    } catch(...) {
        std::cerr << "Unable to write sync debug artifact\n";
    }
}

}  // namespace syncdebug
}  // namespace detail
}  // namespace dai
