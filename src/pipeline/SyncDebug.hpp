#pragma once

// Private, host-only flight recorder. Not installed or exposed as SDK API.
#include <chrono>
#include <cstddef>
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
void emitted(const Handle& output, const std::shared_ptr<ADatatype>& group) noexcept;
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
    // Failure: all bounded histories. Success: configuration and counters only. Never masks the test failure.
    void finish(bool success, const char* reason) noexcept;

   private:
    std::shared_ptr<State> state;
};

}  // namespace syncdebug
}  // namespace detail
}  // namespace dai
