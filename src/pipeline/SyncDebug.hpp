#pragma once

// Private, host-only flight recorder. Not installed or exposed as SDK API.
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
    std::chrono::steady_clock::time_point beforeCallbacks{};
    std::chrono::steady_clock::time_point afterCallbacks{};
    std::chrono::steady_clock::time_point beforePush{};
    std::chrono::steady_clock::time_point afterPush{};
    std::chrono::steady_clock::time_point afterPushRecording{};
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
void dequeued(const Handle& output, const std::shared_ptr<ADatatype>& group) noexcept;
void context(const Handle& handle,
             const char* phase,
             std::optional<std::size_t> sampleIndex = std::nullopt,
             std::optional<std::chrono::system_clock::duration> gap = std::nullopt,
             std::optional<double> limitSec = std::nullopt,
             bool freeze = false) noexcept;

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
