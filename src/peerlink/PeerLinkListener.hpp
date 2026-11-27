#pragma once

// Producer side of the peer-link transport (see PeerLinkProtocol.hpp). POSIX only.

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "peerlink/PeerLinkProtocol.hpp"
#include "peerlink/PeerLinkSocket.hpp"

namespace spdlog {
class logger;
}

namespace dai {
namespace peerlink {

class PeerLinkListener;

/**
 * One stream of a producer. It sends to at most one consumer; with no consumer attached, messages are
 * dropped, so the producing pipeline never waits for a consumer.
 */
class PeerLinkStream {
   public:
    struct Stats {
        std::uint64_t sent = 0;
        /// Messages dropped because no consumer was attached or the send failed
        std::uint64_t dropped = 0;
        std::uint64_t bytesSent = 0;
        /// Consumers accepted so far
        std::uint64_t connections = 0;
    };

    PeerLinkStream(std::string name, const Token& token, std::chrono::milliseconds sendTimeout, std::shared_ptr<spdlog::logger> logger);
    ~PeerLinkStream();
    PeerLinkStream(const PeerLinkStream&) = delete;
    PeerLinkStream& operator=(const PeerLinkStream&) = delete;
    PeerLinkStream(PeerLinkStream&&) = delete;
    PeerLinkStream& operator=(PeerLinkStream&&) = delete;

    /**
     * Sends a message to the attached consumer. Blocks while the consumer applies TCP back-pressure; when no
     * byte can be sent for the send timeout, the consumer is disconnected.
     * @returns False when the message was dropped
     */
    bool send(const std::shared_ptr<ADatatype>& message);

    bool hasConsumer() const;
    Stats getStats() const;
    const std::string& getName() const {
        return name;
    }
    const Token& getToken() const {
        return token;
    }

   private:
    friend class PeerLinkListener;

    // Sends the reply and makes the socket the consumer. A send to the previous consumer is aborted first: a
    // restarted consumer may leave a half-open connection behind, and the newest consumer wins.
    void attach(Socket socket, const std::vector<std::uint8_t>& reply, const std::string& peer);
    // Disconnects the consumer, if any
    void detach(const char* reason);
    // Sends a heartbeat when nothing was sent for `interval`. Never blocks: skips the stream while a send is in
    // progress or while the consumer has not read what was sent (then it is not idle either).
    void heartbeatIfIdle(std::chrono::steady_clock::time_point now, std::chrono::milliseconds interval);

    std::shared_ptr<Socket> currentConsumer() const;
    // The caller holds sendMutex
    IoResult sendMessage(const Socket& socket, const std::shared_ptr<ADatatype>& message);
    IoResult sendFrame(const Socket& socket, FrameKind kind, const iovec* payload, int payloadCount, std::uint64_t payloadSize);
    // Drops `socket` as the consumer, unless another consumer replaced it meanwhile
    void disconnect(const std::shared_ptr<Socket>& socket, const char* reason);
    spdlog::logger& log() const;

    const std::string name;
    const Token token;
    const std::chrono::milliseconds sendTimeout;
    const std::shared_ptr<spdlog::logger> logger;

    // Held for the whole send of a message, so the frames of two messages never interleave
    std::mutex sendMutex;
    // Guarded by sendMutex
    std::chrono::steady_clock::time_point lastSend;
    // Guards consumer and consumerPeer; never held while waiting on the socket
    mutable std::mutex consumerMutex;
    std::shared_ptr<Socket> consumer;
    std::string consumerPeer;

    std::atomic<std::uint64_t> sent{0};
    std::atomic<std::uint64_t> dropped{0};
    std::atomic<std::uint64_t> bytesSent{0};
    std::atomic<std::uint64_t> connections{0};
};

/**
 * Accepts consumers on one TCP port and hands each to the stream it names in its Hello. Also answers
 * probes, so a host can check that one device reaches another before it chooses a direct link.
 */
class PeerLinkListener {
   public:
    struct Config {
        std::string bindAddress = "0.0.0.0";
        /// 0 picks a free port (tests)
        std::uint16_t port = PEERLINK_DEFAULT_PORT;
        /// Returned in every reply, see PeerLinkInfo::listenerId
        std::string listenerId;
        std::chrono::milliseconds heartbeatInterval{1000};
        std::chrono::milliseconds sendTimeout{2000};
        std::chrono::milliseconds handshakeTimeout{2000};
        /// Logger for connection events; the depthai logger when null
        std::shared_ptr<spdlog::logger> logger;
    };

    explicit PeerLinkListener(Config config);
    ~PeerLinkListener();
    PeerLinkListener(const PeerLinkListener&) = delete;
    PeerLinkListener& operator=(const PeerLinkListener&) = delete;
    PeerLinkListener(PeerLinkListener&&) = delete;
    PeerLinkListener& operator=(PeerLinkListener&&) = delete;

    /// Binds the port and starts accepting. @throws std::runtime_error when the port cannot be bound
    void start();
    /// Stops accepting and disconnects all consumers. Registered streams stay registered.
    void stop();
    bool isRunning() const {
        return running;
    }
    /// The bound port, valid after start()
    std::uint16_t getPort() const {
        return boundPort;
    }
    const std::string& getListenerId() const {
        return config.listenerId;
    }

    /// Registers a stream. @throws std::runtime_error when a stream with this name is already registered
    std::shared_ptr<PeerLinkStream> registerStream(const std::string& name, const Token& token);
    /// Unregisters the stream and disconnects its consumer
    void unregisterStream(const std::shared_ptr<PeerLinkStream>& stream);

   private:
    // A handshake runs on its own thread, so a slow or silent peer never holds up the others
    struct Handshake {
        std::thread thread;
        std::shared_ptr<std::atomic<bool>> done;
    };

    void acceptLoop();
    void heartbeatLoop();
    void handleConnection(Socket socket, const std::string& peer);
    // Joins the finished handshake threads, or all of them
    void joinHandshakes(bool all);
    spdlog::logger& log() const;

    const Config config;
    Socket listenSocket;
    std::uint16_t boundPort = 0;
    std::atomic<bool> running{false};
    std::thread acceptThread;
    std::thread heartbeatThread;
    std::mutex heartbeatMutex;
    std::condition_variable heartbeatCv;
    // Only used by the accept thread
    std::vector<Handshake> handshakes;

    mutable std::mutex streamsMutex;
    std::unordered_map<std::string, std::shared_ptr<PeerLinkStream>> streams;
};

}  // namespace peerlink
}  // namespace dai
