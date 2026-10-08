#include "pipeline/SyncDebug.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <exception>
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

using TimePoint = std::chrono::steady_clock::time_point;

std::int64_t timeNs(TimePoint time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

struct TimingRecord {
    std::uint64_t id = 0;
    std::uint64_t groupId = 0;
    const char* phase = "setup";
    std::optional<std::size_t> sampleIndex;
    SyncTiming sync;
    SendTiming send;
    bool syncStarted = false;
    bool syncComplete = false;
    bool sendStarted = false;
    bool sendComplete = false;
    TimePoint syncStartPublished{};
    TimePoint syncSummaryPublished{};
    TimePoint sendStartPublished{};
    TimePoint sendSummaryPublished{};
    TimePoint dequeueRecorderBegin{};
    TimePoint dequeueMetadataFinished{};
};

// Starts are announced before waiting for State::mutex. Freeze copies these atomics once;
// racing/late announcements cannot mutate the frozen snapshot or publish late checkpoints.
struct TimingStarts {
    std::atomic<std::uint64_t> total{0};
    std::atomic<std::int64_t> lastStartNs{0};
    std::uint64_t frozenTotal = 0;
    std::int64_t frozenLastStartNs = 0;

    void announce(TimePoint start) {
        lastStartNs.store(timeNs(start), std::memory_order_relaxed);
        total.fetch_add(1, std::memory_order_relaxed);
    }
    void freeze(std::int64_t cutoff) {
        frozenTotal = total.load(std::memory_order_relaxed);
        const auto last = lastStartNs.load(std::memory_order_relaxed);
        frozenLastStartNs = last <= cutoff ? last : 0;
    }
};

enum class Duration {
    EMISSION_RECORDER,
    RECORDER_TO_OUTPUT_SEND,
    OUTPUT_SEND,
    SEND_ENTRY_TO_CALLBACKS,
    CALLBACKS,
    CALLBACKS_TO_PUSH,
    QUEUE_LOCK_WAIT,
    QUEUE_CAPACITY_WAIT,
    QUEUE_GUARD_INTERVAL,
    QUEUE_GUARD_EXCLUDING_CAPACITY_WAIT,
    ENQUEUE_ACTION,
    UNLOCK_TO_PUSH_RETURN,
    PRODUCER_PUSH_RECORDING,
    SEND_TAIL,
    SEND_SUMMARY_PUBLICATION,
    DEQUEUE_RECORDING,
    COUNT
};
constexpr std::array<const char*, static_cast<std::size_t>(Duration::COUNT)> DURATION_NAMES{"emission_recorder_ns",
                                                                                            "recorder_to_output_send_ns",
                                                                                            "output_send_ns",
                                                                                            "send_entry_to_callbacks_ns",
                                                                                            "callbacks_ns",
                                                                                            "callbacks_to_push_ns",
                                                                                            "queue_lock_wait_ns",
                                                                                            "queue_capacity_wait_ns",
                                                                                            "queue_guard_interval_ns",
                                                                                            "queue_guard_excluding_capacity_wait_ns",
                                                                                            "enqueue_action_ns",
                                                                                            "unlock_to_push_return_ns",
                                                                                            "producer_push_recording_ns",
                                                                                            "send_tail_ns",
                                                                                            "send_summary_publication_ns",
                                                                                            "dequeue_recording_ns"};

std::int64_t elapsedNs(TimePoint begin, TimePoint end) {
    return begin == TimePoint{} || end == TimePoint{} || end < begin ? -1 : timeNs(end) - timeNs(begin);
}

std::array<std::int64_t, DURATION_NAMES.size()> durations(const TimingRecord& record) {
    const auto& sync = record.sync;
    const auto& send = record.send;
    const auto& queue = send.queue;
    const auto guard = elapsedNs(queue.lockAcquiredAt, queue.guardReleasedAt);
    const auto capacity = elapsedNs(queue.capacityWaitStartedAt, queue.capacityWaitFinishedAt);
    const auto guardWithoutWait = queue.capacityWaitStartedAt == TimePoint{} ? guard : (guard >= capacity && capacity >= 0 ? guard - capacity : -1);
    return {elapsedNs(sync.beforeEmissionRecorder, sync.afterEmissionRecorder),
            elapsedNs(sync.afterEmissionRecorder, sync.beforeOutputSend),
            elapsedNs(sync.beforeOutputSend, sync.afterOutputSend),
            elapsedNs(send.entry, send.beforeCallbacks),
            elapsedNs(send.beforeCallbacks, send.afterCallbacks),
            elapsedNs(send.afterCallbacks, send.beforePush),
            elapsedNs(queue.lockWaitStartedAt, queue.lockAcquiredAt),
            capacity,
            guard,
            guardWithoutWait,
            elapsedNs(queue.enqueueStartedAt, send.enqueueCompleted),
            elapsedNs(queue.guardReleasedAt, send.afterPush),
            elapsedNs(send.afterPush, send.afterPushRecording),
            elapsedNs(send.afterPushRecording, send.exit),
            elapsedNs(send.exit, record.sendSummaryPublished),
            elapsedNs(record.dequeueRecorderBegin, record.dequeueMetadataFinished)};
}

struct DurationSummary {
    std::uint64_t samples = 0;
    std::int64_t maxNs = 0;
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
    // One preallocated, metadata-only delivery ring per selected output/session.
    utility::CircularBuffer<TimingRecord> timings{HISTORY_CAPACITY};
    std::uint64_t timingTotal = 0;
    TimingStarts syncStarts;
    TimingStarts sendStarts;
    std::uint64_t syncStartRecords = 0;
    std::uint64_t sendStartRecords = 0;
    std::uint64_t syncSummaries = 0;
    std::uint64_t sendSummaries = 0;
    std::uint64_t overwrittenPendingTimings = 0;
    std::uint64_t completionUpdatesWithoutRecord = 0;
    std::array<DurationSummary, DURATION_NAMES.size()> durationSummaries{};
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

    TimingRecord& addTiming(std::uint64_t groupId) {
        if(timings.size() == HISTORY_CAPACITY) {
            const auto& oldest = timings.at(0);
            overwrittenPendingTimings += (oldest.syncStarted && !oldest.syncComplete) || (oldest.sendStarted && !oldest.sendComplete);
        }
        TimingRecord record;
        record.id = ++timingTotal;
        record.groupId = groupId;
        record.phase = phase;
        record.sampleIndex = sampleIndex;
        return timings.add(std::move(record));
    }

    TimingRecord* findTiming(std::uint64_t id) {
        if(id == 0) return nullptr;
        for(auto it = timings.rbegin(); it != timings.rend(); ++it) {
            if(it->id == id) return &*it;
        }
        return nullptr;
    }

    TimingRecord* findGroupTiming(std::uint64_t groupId, bool beforeSend = false) {
        if(groupId == 0) return nullptr;
        for(auto it = timings.rbegin(); it != timings.rend(); ++it) {
            if(it->groupId == groupId && (!beforeSend || !it->sendStarted)) return &*it;
        }
        return nullptr;
    }

    void summarize(const TimingRecord& record, Duration first, Duration end) {
        const auto values = durations(record);
        for(auto i = static_cast<std::size_t>(first); i < static_cast<std::size_t>(end); ++i) {
            if(values[i] < 0) continue;
            auto& summary = durationSummaries[i];
            ++summary.samples;
            summary.maxNs = std::max(summary.maxNs, values[i]);
        }
    }

    void freezeRecording() {
        frozenSteadyNs = steadyNs();
        frozen.store(true, std::memory_order_relaxed);
        syncStarts.freeze(frozenSteadyNs);
        sendStarts.freeze(frozenSteadyNs);
    }

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

    std::uint64_t recordGroup(Stage stage,
                              const std::shared_ptr<ADatatype>& message,
                              std::optional<std::int64_t> time = std::nullopt,
                              SyncTiming* emissionTiming = nullptr) {
        const auto* group = dynamic_cast<const MessageGroup*>(message.get());
        if(!group) return 0;
        const auto id = stage == Stage::EMITTED ? ++nextGroupId : identify(message);
        if(stage == Stage::EMITTED) identities.add({message, id});
        if(emissionTiming) {
            auto& timing = addTiming(id);
            emissionTiming->recordId = timing.id;
            timing.sync = *emissionTiming;
            timing.syncStarted = true;
            timing.syncStartPublished = std::chrono::steady_clock::now();
            ++syncStartRecords;
        }
        if(stage == Stage::DEQUEUED) lastDequeuedGroupId = id;
        const std::size_t slot = static_cast<std::size_t>(stage) - static_cast<std::size_t>(Stage::EMITTED);
        groups.at(slot).add({time.value_or(steadyNs()), id, stage, phase, sampleIndex});
        ++groupTotals.at(slot);
        for(std::size_t i = 0; i < streams.size(); ++i) {
            const auto it = group->group.find(streams[i].input.name);
            if(it != group->group.end() && it->second) record(stage, i, *it->second, id, "", 0, 0, time);
        }
        return id;
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

nlohmann::json checkpointJson(TimePoint time) {
    return time == TimePoint{} ? nlohmann::json(nullptr) : nlohmann::json(timeNs(time));
}

nlohmann::json timingHistoryJson(const State& state, bool success) {
    nlohmann::json result = {{"queue_name", state.queues.back().name},
                             {"total", state.timingTotal},
                             {"history_overwrites", state.timingTotal - state.timings.size()},
                             {"overwritten_pending_records", state.overwrittenPendingTimings},
                             {"completion_updates_without_retained_record", state.completionUpdatesWithoutRecord},
                             {"sync_starts", state.syncStarts.frozenTotal},
                             {"sync_start_records", state.syncStartRecords},
                             {"sync_summaries_published", state.syncSummaries},
                             {"sync_starts_without_record", state.syncStarts.frozenTotal - state.syncStartRecords},
                             {"sync_without_summary", state.syncStarts.frozenTotal - state.syncSummaries},
                             {"send_starts", state.sendStarts.frozenTotal},
                             {"send_start_records", state.sendStartRecords},
                             {"send_summaries_published", state.sendSummaries},
                             {"send_starts_without_record", state.sendStarts.frozenTotal - state.sendStartRecords},
                             {"send_without_summary", state.sendStarts.frozenTotal - state.sendSummaries}};
    result["last_sync_start_ns"] = state.syncStarts.frozenLastStartNs ? nlohmann::json(state.syncStarts.frozenLastStartNs) : nlohmann::json(nullptr);
    result["last_send_start_ns"] = state.sendStarts.frozenLastStartNs ? nlohmann::json(state.sendStarts.frozenLastStartNs) : nlohmann::json(nullptr);
    result["duration_maxima"] = nlohmann::json::object();
    for(std::size_t i = 0; i < DURATION_NAMES.size(); ++i) {
        const auto& summary = state.durationSummaries[i];
        result["duration_maxima"][DURATION_NAMES[i]] = {{"samples", summary.samples},
                                                        {"max_ns", summary.samples ? nlohmann::json(summary.maxNs) : nlohmann::json(nullptr)}};
    }
    if(success) return result;
    result["events"] = nlohmann::json::array();
    for(const auto& record : state.timings.getBuffer()) {
        const auto& sync = record.sync;
        const auto& send = record.send;
        const auto& queue = send.queue;
        nlohmann::json entry = {{"record_id", record.id},
                                {"group_id", record.groupId},
                                {"phase", record.phase},
                                {"sync_started", record.syncStarted},
                                {"sync_complete", record.syncComplete},
                                {"sync_inflight", record.syncStarted && !record.syncComplete},
                                {"send_started", record.sendStarted},
                                {"send_complete", record.sendComplete},
                                {"send_inflight", record.sendStarted && !record.sendComplete}};
        entry["sample_index"] = record.sampleIndex ? nlohmann::json(*record.sampleIndex) : nlohmann::json(nullptr);
        entry["sync"] = {{"before_emission_recorder_ns", checkpointJson(sync.beforeEmissionRecorder)},
                         {"after_emission_recorder_ns", checkpointJson(sync.afterEmissionRecorder)},
                         {"before_output_send_ns", checkpointJson(sync.beforeOutputSend)},
                         {"after_output_send_ns", checkpointJson(sync.afterOutputSend)},
                         {"output_send_exception_ns", checkpointJson(sync.outputSendException)},
                         {"start_published_ns", checkpointJson(record.syncStartPublished)},
                         {"summary_published_ns", checkpointJson(record.syncSummaryPublished)}};
        entry["message_queue"] = {{"entry_ns", checkpointJson(send.entry)},
                                  {"before_callbacks_ns", checkpointJson(send.beforeCallbacks)},
                                  {"after_callbacks_ns", checkpointJson(send.afterCallbacks)},
                                  {"before_push_ns", checkpointJson(send.beforePush)},
                                  {"after_push_ns", checkpointJson(send.afterPush)},
                                  {"after_push_recording_ns", checkpointJson(send.afterPushRecording)},
                                  {"exit_ns", checkpointJson(send.exit)},
                                  {"lock_wait_started_ns", checkpointJson(queue.lockWaitStartedAt)},
                                  {"lock_acquired_ns", checkpointJson(queue.lockAcquiredAt)},
                                  {"capacity_wait_started_ns", checkpointJson(queue.capacityWaitStartedAt)},
                                  {"capacity_wait_finished_ns", checkpointJson(queue.capacityWaitFinishedAt)},
                                  {"enqueue_started_ns", checkpointJson(queue.enqueueStartedAt)},
                                  {"enqueue_completed_ns", checkpointJson(send.enqueueCompleted)},
                                  {"guard_released_ns", checkpointJson(queue.guardReleasedAt)},
                                  {"timed", send.timed},
                                  {"accepted", send.afterPush != TimePoint{} ? nlohmann::json(send.accepted) : nlohmann::json(nullptr)},
                                  {"exception", record.sendComplete ? nlohmann::json(send.exception) : nlohmann::json(nullptr)},
                                  {"start_published_ns", checkpointJson(record.sendStartPublished)},
                                  {"summary_published_ns", checkpointJson(record.sendSummaryPublished)}};
        entry["dequeue_recorder"] = {{"begin_ns", checkpointJson(record.dequeueRecorderBegin)},
                                     {"metadata_finished_ns", checkpointJson(record.dequeueMetadataFinished)}};
        entry["durations"] = nlohmann::json::object();
        const auto values = durations(record);
        for(std::size_t i = 0; i < values.size(); ++i) {
            entry["durations"][DURATION_NAMES[i]] = values[i] < 0 ? nlohmann::json(nullptr) : nlohmann::json(values[i]);
        }
        result["events"].push_back(std::move(entry));
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
    result["output_timings"] = timingHistoryJson(state, success);
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

SendOperation::SendOperation(const MessageQueue* queue, const std::shared_ptr<ADatatype>& message, bool timed) noexcept {
    // Timestamp precedes the registry lookup, queue guards, arrival recording and State::mutex.
    // With no registered sessions this is one atomic check and no clock call.
    if(!registry().active.load(std::memory_order_relaxed)) return;
    const auto entry = std::chrono::steady_clock::now();
    trace = find(queue);
    if(!trace || trace.queueIndex != trace.state->streams.size() || trace.state->frozen.load(std::memory_order_relaxed)) return;
    enabled = true;
    exceptionsOnEntry = std::uncaught_exceptions();
    timing.entry = entry;
    timing.timed = timed;
    diagnostics.timing.enabled = true;
    trace.state->sendStarts.announce(entry);
    update(trace, [&](State& state) {
        const auto groupId = dynamic_cast<const MessageGroup*>(message.get()) ? state.identify(message) : 0;
        auto* record = state.findGroupTiming(groupId, true);
        if(!record) record = &state.addTiming(groupId);
        timing.recordId = record->id;
        record->send = timing;
        record->sendStarted = true;
        record->sendStartPublished = std::chrono::steady_clock::now();
        ++state.sendStartRecords;
    });
}

SendOperation::~SendOperation() noexcept {
    if(!enabled) return;
    // Logical exit is before publishing this summary; Sync's out.send boundary also
    // measures this final publication. No queued/consumed message ownership is needed.
    timing.exit = std::chrono::steady_clock::now();
    timing.exception = std::uncaught_exceptions() > exceptionsOnEntry;
    timing.queue = diagnostics.timing;
    timing.enqueueCompleted = diagnostics.completedAt;
    update(trace, [&](State& state) {
        TimingRecord unretained;
        auto* record = state.findTiming(timing.recordId);
        if(!record) {
            ++state.completionUpdatesWithoutRecord;
            record = &unretained;
        }
        record->send = timing;
        record->sendComplete = true;
        record->sendSummaryPublished = std::chrono::steady_clock::now();
        ++state.sendSummaries;
        state.summarize(*record, Duration::SEND_ENTRY_TO_CALLBACKS, Duration::DEQUEUE_RECORDING);
    });
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

void emitted(const Handle& output, const std::shared_ptr<ADatatype>& group, SyncTiming& timing) noexcept {
    if(!output || output.state->frozen.load(std::memory_order_relaxed)) return;
    output.state->syncStarts.announce(timing.beforeEmissionRecorder);
    update(output, [&](State& state) { state.recordGroup(Stage::EMITTED, group, std::nullopt, &timing); });
}

void delivered(const Handle& output, const SyncTiming& timing) noexcept {
    update(output, [&](State& state) {
        TimingRecord unretained;
        auto* record = state.findTiming(timing.recordId);
        if(!record) {
            ++state.completionUpdatesWithoutRecord;
            record = &unretained;
        }
        record->sync = timing;
        record->syncComplete = true;
        record->syncSummaryPublished = std::chrono::steady_clock::now();
        ++state.syncSummaries;
        state.summarize(*record, Duration::EMISSION_RECORDER, Duration::SEND_ENTRY_TO_CALLBACKS);
    });
}

void dequeued(const Handle& output, const std::shared_ptr<ADatatype>& group) noexcept {
    if(!output || output.state->frozen.load(std::memory_order_relaxed)) return;
    const auto begin = std::chrono::steady_clock::now();
    update(output, [&](State& state) {
        const auto id = state.recordGroup(Stage::DEQUEUED, group, timeNs(begin));
        if(auto* record = state.findGroupTiming(id)) {
            record->dequeueRecorderBegin = begin;
            record->dequeueMetadataFinished = std::chrono::steady_clock::now();
            state.summarize(*record, Duration::DEQUEUE_RECORDING, Duration::COUNT);
        }
    });
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
            state.freezeRecording();
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
            state->freezeRecording();
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
