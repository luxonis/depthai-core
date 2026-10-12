#pragma once

// Private, host-only flight recorder. Not installed or exposed as SDK API.
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "nlohmann/json.hpp"

namespace dai {
namespace detail {

struct SyncDebugQueueAccess {
    using Message = std::shared_ptr<ADatatype>;
    using Diagnostics = QueuePushDiagnostics<Message>;
    using Callback = std::function<void(LockingQueueState, size_t)>;

    static bool push(LockingQueue<Message>& queue, const Message& message, Callback callback, Diagnostics* diagnostics) {
        return queue.pushImpl(message, std::move(callback), diagnostics);
    }

    static bool push(LockingQueue<Message>& queue, const Message& message, std::chrono::milliseconds timeout, Callback callback, Diagnostics* diagnostics) {
        return queue.tryWaitAndPushImpl(message, timeout, std::move(callback), diagnostics);
    }

    template <class T, typename Rep, typename Period>
    static std::shared_ptr<T> get(
        MessageQueue& queue, std::chrono::duration<Rep, Period> timeout, bool& hasTimedout, QueueReadTiming* timing, Message* popped) {
        return queue.getImpl<T>(timeout, hasTimedout, timing, popped);
    }
};

namespace syncdebug {

struct State;
struct Handle {
    std::shared_ptr<State> state;
    std::size_t queueIndex = 0;
    explicit operator bool() const noexcept {
        return state != nullptr;
    }
};

struct Input {
    MessageQueue* queue = nullptr;
    std::string name;
    std::string deviceId;
};

struct RelayContext {
    // Built once in XLinkInHost::run from canonical connected queues. No payloads.
    struct Selection {
        Handle handle;
        std::size_t source = 0;
        std::shared_ptr<void> nativeProgress;  // Stable private scalar slot; source vectors can grow at setup.
    };
    std::vector<Selection> inputs;
};
struct RelayTiming {
    std::chrono::steady_clock::time_point entry{}, readReturned{}, metadataAvailable{}, metadataFinished{}, beforeSendRecorder{}, beforeSend{}, afterSend{},
        exit{};
    std::chrono::steady_clock::time_point packetBegin{}, packetEnd{}, parseBegin{}, parseEnd{};
    std::chrono::steady_clock::time_point packetRecorderBegin{}, parseRecorderBegin{};
    std::int64_t packetWaitNs = 0, parseNs = 0;
    std::int64_t packetStartRecordingNs = 0, parseStartRecordingNs = 0;
    std::uint64_t packets = 0, parses = 0;
    std::uint64_t packetExceptions = 0, parseExceptions = 0;
    bool exception = false;
};
enum class RelayBoundary {
    NONE,
    OPERATION_ENTRY,
    PACKET_READ_ENTRY,
    PACKET_READ_EXIT,
    PARSE_ENTRY,
    PARSE_EXIT,
    READ_RETURN,
    METADATA_READY,
    OUT_SEND_ENTRY,
    OUT_SEND_RETURN,
    OPERATION_EXIT
};
// Source mapping takes canonical Input/MessageQueue addresses at run setup only.
std::shared_ptr<RelayContext> relayContext(const std::vector<const MessageQueue*>& queues, const std::string& stream, std::int64_t node) noexcept;
void relaySetupFailed() noexcept;
class RelayOperation {
   public:
    explicit RelayOperation(const std::shared_ptr<RelayContext>& context, const char* operation = "read_and_send") noexcept;
    ~RelayOperation() noexcept;
    RelayOperation(const RelayOperation&) = delete;
    RelayOperation& operator=(const RelayOperation&) = delete;
    explicit operator bool() const noexcept {
        return enabled;
    }
    void checkpoint(const ADatatype* message = nullptr) noexcept;
    // Scalar, lock-free native boundary publication; freeze samples it once.
    void nativeCheckpoint(RelayBoundary boundary, std::chrono::steady_clock::time_point time) noexcept;
    RelayTiming timing;

   private:
    std::shared_ptr<RelayContext> trace;
    std::array<std::uint64_t, 64> recordIds{};
    const char* operation;
    int exceptionsOnEntry = 0;
    bool enabled = false;
    RelayOperation* previous = nullptr;
};
// Thread-local operation hooks split the actual readMove API from parser calls,
// including nested MessageGroup/PacketizedData reads without changing signatures.
enum class RelayPart { PACKET_READ, PARSE };
class RelayPartOperation {
   public:
    explicit RelayPartOperation(RelayPart part) noexcept;
    ~RelayPartOperation() noexcept;
    RelayPartOperation(const RelayPartOperation&) = delete;
    RelayPartOperation& operator=(const RelayPartOperation&) = delete;

   private:
    RelayOperation* operation = nullptr;
    RelayPart part;
    std::chrono::steady_clock::time_point start{};
    int exceptionsOnEntry = 0;
};

// Zero time points mean an unpublished checkpoint, not evidence that the phase never ran.
struct SyncTiming {
    std::uint64_t recordId = 0;
    std::chrono::steady_clock::time_point beforeEmissionRecorder{};
    std::chrono::steady_clock::time_point afterEmissionRecorder{};
    std::chrono::steady_clock::time_point beforeOutputSend{};
    std::chrono::steady_clock::time_point afterOutputSend{};
    std::chrono::steady_clock::time_point outputSendException{};
};

struct SendTiming {
    std::uint64_t recordId = 0;
    std::chrono::steady_clock::time_point entry{};
    std::chrono::steady_clock::time_point afterLookup{};
    std::chrono::steady_clock::time_point beforeArrivalRecorder{};
    std::chrono::steady_clock::time_point afterArrivalRecorder{};
    std::chrono::steady_clock::time_point beforeCallbacks{};
    std::chrono::steady_clock::time_point afterCallbacks{};
    std::chrono::steady_clock::time_point beforePush{};
    std::chrono::steady_clock::time_point afterPush{};
    std::chrono::steady_clock::time_point afterPushRecording{};
    std::chrono::steady_clock::time_point beforeListeners{};
    std::chrono::steady_clock::time_point afterListeners{};
    std::chrono::steady_clock::time_point exit{};
    QueuePushTiming queue;
    std::chrono::steady_clock::time_point enqueueCompleted{};
    bool timed = false;
    bool accepted = false;
    bool exception = false;
};

// Stack-only operation, retaining no payload in the recorder. Destruction also records exceptions.
class SendOperation {
   public:
    SendOperation(const MessageQueue* queue, const std::shared_ptr<ADatatype>& message, bool timed = false) noexcept;
    ~SendOperation() noexcept;
    SendOperation(const SendOperation&) = delete;
    SendOperation& operator=(const SendOperation&) = delete;
    explicit operator bool() const noexcept {
        return enabled;
    }
    Handle trace;
    SendTiming timing;
    SyncDebugQueueAccess::Diagnostics diagnostics;

   private:
    int exceptionsOnEntry = 0;
    bool enabled = false;
};

// One serial GroupReader owns this context; the cached handle avoids a registry lookup
// before queue get. The post-get lookup is measured independently at its original site.
struct ReaderContext {
    Handle trace;
    const char* phase = "setup";
    std::optional<std::size_t> sampleIndex;
    std::uint64_t contextId = 0;
    std::chrono::steady_clock::time_point contextEntry{};
    std::chrono::steady_clock::time_point contextReturned{};
};

struct ReadTiming {
    std::uint64_t recordId = 0;
    std::uint64_t groupId = 0;
    std::uint64_t deliveryRecordId = 0;
    std::uint64_t contextId = 0;
    std::chrono::steady_clock::time_point contextEntry{};
    std::chrono::steady_clock::time_point contextReturned{};
    std::chrono::steady_clock::time_point entry{};
    std::chrono::steady_clock::time_point deadline{};
    std::chrono::steady_clock::time_point startPublished{};
    std::chrono::steady_clock::time_point deadlineChecked{};
    QueueReadTiming get;
    std::chrono::steady_clock::time_point typedGetReturned{};
    std::chrono::steady_clock::time_point beforeLookup{};
    std::chrono::steady_clock::time_point afterLookup{};
    std::chrono::steady_clock::time_point beforeDequeueRecorder{};
    std::chrono::steady_clock::time_point dequeueMetadataFinished{};
    std::chrono::steady_clock::time_point afterDequeueRecorder{};
    std::chrono::steady_clock::time_point beforeAnalysis{};
    std::chrono::steady_clock::time_point analysisFinished{};
    std::chrono::steady_clock::time_point exit{};
    bool deadlineExpired = false;
    bool timedOut = false;
    bool lookupFound = false;
    bool exception = false;
};

std::shared_ptr<ReaderContext> readerContext(const MessageQueue* queue) noexcept;

class ReadOperation {
   public:
    ReadOperation(const std::shared_ptr<ReaderContext>& context, std::chrono::steady_clock::time_point deadline) noexcept;
    ~ReadOperation() noexcept;
    ReadOperation(const ReadOperation&) = delete;
    ReadOperation& operator=(const ReadOperation&) = delete;
    explicit operator bool() const noexcept {
        return static_cast<bool>(trace);
    }
    void recordDequeue(const MessageQueue* queue) noexcept;
    ReadTiming timing;
    // Exact popped identity, including a failed typed cast; released with this stack operation.
    std::shared_ptr<ADatatype> popped;

   private:
    Handle trace;
    int exceptionsOnEntry = 0;
};

// Lookup is a single atomic check when no session is registered.
bool requested() noexcept;
Handle find(const MessageQueue* queue) noexcept;
void arrival(const Handle& handle, const std::shared_ptr<ADatatype>& message) noexcept;
void pushed(const Handle& handle, const std::shared_ptr<ADatatype>& message, const SyncDebugQueueAccess::Diagnostics& diagnostics, bool accepted) noexcept;
void consumed(const Handle& handle, const std::shared_ptr<ADatatype>& message) noexcept;
void discarded(const Handle& handle,
               const std::shared_ptr<ADatatype>& message,
               const char* reason,
               std::chrono::nanoseconds spread,
               std::chrono::nanoseconds threshold) noexcept;
void emitted(const Handle& output, const std::shared_ptr<ADatatype>& group, SyncTiming& timing) noexcept;
void delivered(const Handle& output, const SyncTiming& timing) noexcept;
void dequeued(const Handle& output, const std::shared_ptr<ADatatype>& group, ReadTiming* timing = nullptr) noexcept;
void context(const Handle& handle,
             const char* phase,
             std::optional<std::size_t> sampleIndex = std::nullopt,
             std::optional<std::chrono::system_clock::duration> gap = std::nullopt,
             std::optional<double> limitSec = std::nullopt,
             bool freeze = false,
             ReaderContext* reader = nullptr) noexcept;

class Session {
   public:
    // DEPTHAI_SYNC_DEBUG=1 enables recording; DEPTHAI_SYNC_DEBUG_DIR chooses the artifact directory.
    static std::unique_ptr<Session> create(const std::vector<Input>& inputs, MessageQueue& output, nlohmann::json configuration);
    explicit Session(std::shared_ptr<State> state);
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    Session(Session&&) = delete;
    Session& operator=(Session&&) = delete;
    void freeze() noexcept;
    // Failure: all bounded histories. Success: configuration, counters and timing maxima. Never masks the test failure.
    void finish(bool success, const char* reason) noexcept;

   private:
    std::shared_ptr<State> state;
};

}  // namespace syncdebug
}  // namespace detail
}  // namespace dai
