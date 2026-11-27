#pragma once

// Consumer side of the peer-link transport (see PeerLinkProtocol.hpp). POSIX only.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "peerlink/PeerLinkProtocol.hpp"
#include "peerlink/PeerLinkSocket.hpp"

namespace spdlog {
class logger;
}

namespace dai {
namespace peerlink {

/**
 * Checks whether a peer-link listener answers at host:port.
 * @returns The listener id from the reply, or std::nullopt when nothing answers within `timeout`
 */
std::optional<std::string> probe(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout);

enum class ConnectStatus : std::uint8_t {
    CONNECTED,
    /// No endpoint accepted a TCP connection
    UNREACHABLE,
    /// The producer is reachable but has not registered the stream yet
    UNKNOWN_STREAM,
    BAD_TOKEN,
    BAD_VERSION,
    /// The endpoint is not a peer-link listener, or it closed the connection during the handshake
    PROTOCOL_ERROR,
};

const char* toString(ConnectStatus status);

/// One connection to a producer stream. Not thread safe, except interrupt().
class PeerLinkClient {
   public:
    struct Config {
        /// "host:port" of the producer, tried in order
        std::vector<std::string> endpoints;
        std::string streamName;
        Token token{};
        /// Listener id of the producer; an endpoint that answers with another id is skipped. Empty accepts any.
        std::string listenerId;
        std::chrono::milliseconds connectTimeout{1000};
        /// The link is dead when no byte arrives for this long (the producer sends heartbeats when idle)
        std::chrono::milliseconds livenessTimeout{3000};
    };

    explicit PeerLinkClient(Config config);
    ~PeerLinkClient();
    PeerLinkClient(const PeerLinkClient&) = delete;
    PeerLinkClient& operator=(const PeerLinkClient&) = delete;
    PeerLinkClient(PeerLinkClient&&) = delete;
    PeerLinkClient& operator=(PeerLinkClient&&) = delete;

    /**
     * Tries the endpoints in order. The first endpoint where the producer (the expected listener) answers decides
     * the status; an endpoint that does not answer, answers with another listener id or breaks the handshake is
     * skipped.
     * @param detail Receives a human readable reason for a failure
     */
    ConnectStatus connect(std::string* detail = nullptr);

    /**
     * Receives the next message, with all parts of a MessageGroup.
     * @returns nullptr when the connection is lost or interrupted; connect() again to continue
     */
    std::shared_ptr<ADatatype> receive();

    void disconnect();
    /// Unblocks a connect() or receive() in another thread. Thread safe.
    void interrupt();
    /// Replaces the producer endpoints for the next connect(), for a producer that came back at other addresses. Thread safe.
    void setEndpoints(std::vector<std::string> endpoints);

    bool isConnected() const {
        return socket.valid();
    }
    /// Endpoint of the current connection
    const std::string& getEndpoint() const {
        return endpoint;
    }
    /// Why the last receive() returned nullptr
    const std::string& getLastError() const {
        return lastError;
    }

   private:
    // UNREACHABLE also for an endpoint where another device answers
    ConnectStatus connectEndpoint(const std::string& target, std::string& detail);
    std::shared_ptr<ADatatype> receiveMessage();
    bool receiveExact(void* data, std::size_t size);

    const Config config;
    // Guarded by endpointsMutex, see setEndpoints()
    std::vector<std::string> endpoints;
    std::mutex endpointsMutex;
    Socket socket;
    std::string endpoint;
    std::string lastError;
    // Guards the socket descriptor against interrupt() from another thread
    std::mutex socketMutex;
    std::atomic<bool> interrupted{false};
};

/**
 * A consumer that reconnects by itself: it retries the producer with back-off until the producer registers
 * the stream, and again after the link is lost.
 */
class PeerLinkReceiver {
   public:
    struct Config {
        PeerLinkClient::Config client;
        std::chrono::milliseconds minRetryInterval{100};
        std::chrono::milliseconds maxRetryInterval{1000};
        /// Logger for connection events; the depthai logger when null
        std::shared_ptr<spdlog::logger> logger;
    };

    explicit PeerLinkReceiver(Config config);
    ~PeerLinkReceiver();
    PeerLinkReceiver(const PeerLinkReceiver&) = delete;
    PeerLinkReceiver& operator=(const PeerLinkReceiver&) = delete;
    PeerLinkReceiver(PeerLinkReceiver&&) = delete;
    PeerLinkReceiver& operator=(PeerLinkReceiver&&) = delete;

    /**
     * Blocks until a message arrives, connecting and reconnecting as needed.
     * @returns nullptr once stop() was called
     */
    std::shared_ptr<ADatatype> receive();
    /// Makes receive() return nullptr, now and from then on. Thread safe.
    void stop();
    /**
     * Replaces the producer endpoints, for a producer that came back at other addresses. A connection in place is
     * kept; the next connection attempt, which starts right away when one is pending, uses the new endpoints.
     * Thread safe.
     */
    void setEndpoints(std::vector<std::string> endpoints);

    bool isConnected() const {
        return connected;
    }
    /// Successful connections so far
    std::uint64_t getConnectionCount() const {
        return connections;
    }

   private:
    spdlog::logger& log() const;
    // Waits for the retry interval, returns false when stopped
    bool waitRetry(std::chrono::milliseconds interval);

    const Config config;
    PeerLinkClient client;
    std::atomic<bool> stopped{false};
    std::atomic<bool> connected{false};
    std::atomic<std::uint64_t> connections{0};
    std::mutex waitMutex;
    std::condition_variable waitCv;
    // Set by setEndpoints() to cut a retry wait short; guarded by waitMutex
    bool endpointsChanged = false;
    std::chrono::milliseconds retryInterval;
    std::optional<ConnectStatus> lastFailure;
};

}  // namespace peerlink
}  // namespace dai
