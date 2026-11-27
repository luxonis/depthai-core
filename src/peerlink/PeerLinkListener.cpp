#include "peerlink/PeerLinkListener.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace peerlink {

namespace listenerdetail {

constexpr std::chrono::milliseconds ACCEPT_POLL_INTERVAL{200};
// Connections beyond this many unfinished handshakes are closed right away
constexpr std::size_t MAX_PENDING_HANDSHAKES = 16;

// A message the consumer can parse: not empty, and no empty part in a group (at any depth)
bool isSendable(const std::shared_ptr<ADatatype>& message) {
    if(message == nullptr) return false;
    if(const auto group = std::dynamic_pointer_cast<MessageGroup>(message)) {
        return std::all_of(group->group.begin(), group->group.end(), [](const auto& part) { return isSendable(part.second); });
    }
    return true;
}

std::string formatPeer(const sockaddr_storage& address) {
    std::array<char, INET6_ADDRSTRLEN> buffer{};
    if(address.ss_family == AF_INET) {
        const auto* in = reinterpret_cast<const sockaddr_in*>(&address);
        inet_ntop(AF_INET, &in->sin_addr, buffer.data(), buffer.size());
        return formatEndpoint(buffer.data(), ntohs(in->sin_port));
    }
    if(address.ss_family == AF_INET6) {
        const auto* in6 = reinterpret_cast<const sockaddr_in6*>(&address);
        inet_ntop(AF_INET6, &in6->sin6_addr, buffer.data(), buffer.size());
        return formatEndpoint(buffer.data(), ntohs(in6->sin6_port));
    }
    return "unknown peer";
}

}  // namespace listenerdetail

// PeerLinkStream

PeerLinkStream::PeerLinkStream(std::string name, const Token& token, std::chrono::milliseconds sendTimeout, std::shared_ptr<spdlog::logger> logger)
    : name(std::move(name)), token(token), sendTimeout(sendTimeout), logger(std::move(logger)) {}

PeerLinkStream::~PeerLinkStream() = default;

spdlog::logger& PeerLinkStream::log() const {
    return logger ? *logger : Logging::getInstance().logger;
}

std::shared_ptr<Socket> PeerLinkStream::currentConsumer() const {
    std::lock_guard<std::mutex> lock(consumerMutex);
    return consumer;
}

bool PeerLinkStream::send(const std::shared_ptr<ADatatype>& message) {
    // Checked before any byte goes out: a group frame announces its parts, so a part that cannot be sent would
    // leave the consumer waiting for it
    if(!listenerdetail::isSendable(message)) {
        dropped++;
        log().warn("Peer-link stream '{}': dropped an empty message or a MessageGroup with an empty part", name);
        return false;
    }
    std::lock_guard<std::mutex> sendLock(sendMutex);
    const auto socket = currentConsumer();
    if(socket == nullptr) {
        dropped++;
        return false;
    }
    const auto result = sendMessage(*socket, message);
    if(result != IoResult::OK) {
        dropped++;
        disconnect(socket, toString(result));
        return false;
    }
    sent++;
    return true;
}

bool PeerLinkStream::hasConsumer() const {
    return currentConsumer() != nullptr;
}

PeerLinkStream::Stats PeerLinkStream::getStats() const {
    Stats stats;
    stats.sent = sent;
    stats.dropped = dropped;
    stats.bytesSent = bytesSent;
    stats.connections = connections;
    return stats;
}

void PeerLinkStream::attach(Socket socket, const std::vector<std::uint8_t>& reply, const std::string& peer) {
    // Abort a send to the previous consumer, which may be stuck on a dead connection, so it releases sendMutex
    if(const auto previous = currentConsumer()) {
        log().info("Peer-link stream '{}': consumer {} replaces the previous one", name, peer);
        disconnect(previous, "replaced by a new consumer");
    }
    std::lock_guard<std::mutex> sendLock(sendMutex);
    auto attached = std::make_shared<Socket>(std::move(socket));
    setSendTimeout(*attached, sendTimeout);
    std::vector<std::uint8_t> replyBytes = reply;
    iovec buffer{replyBytes.data(), replyBytes.size()};
    const auto result = sendAll(*attached, &buffer, 1);
    if(result != IoResult::OK) {
        log().info("Peer-link stream '{}': reply to consumer {} failed: {}", name, peer, toString(result));
        return;
    }
    {
        std::lock_guard<std::mutex> lock(consumerMutex);
        consumer = attached;
        consumerPeer = peer;
    }
    lastSend = std::chrono::steady_clock::now();
    connections++;
    log().info("Peer-link stream '{}': consumer {} connected", name, peer);
}

void PeerLinkStream::detach(const char* reason) {
    if(const auto current = currentConsumer()) {
        disconnect(current, reason);
    }
}

void PeerLinkStream::heartbeatIfIdle(std::chrono::steady_clock::time_point now, std::chrono::milliseconds interval) {
    std::unique_lock<std::mutex> sendLock(sendMutex, std::try_to_lock);
    // A send in progress keeps the link alive by itself
    if(!sendLock.owns_lock() || now - lastSend < interval) {
        return;
    }
    const auto socket = currentConsumer();
    // A full send buffer means the consumer has data to read, and a 9-byte heartbeat fits whenever the socket
    // is writable, so the heartbeat never blocks the other streams' heartbeats
    if(socket == nullptr || !isWritable(*socket)) {
        return;
    }
    const auto result = sendFrame(*socket, FrameKind::HEARTBEAT, nullptr, 0, 0);
    if(result != IoResult::OK) {
        disconnect(socket, toString(result));
    }
}

IoResult PeerLinkStream::sendMessage(const Socket& socket, const std::shared_ptr<ADatatype>& message) {
    if(message == nullptr) {
        return IoResult::FAILED;
    }
    const auto metadata = StreamMessageParser::serializeMetadata(message);
    const auto group = std::dynamic_pointer_cast<MessageGroup>(message);
    // Like XLinkOut: a group frame carries only the group's metadata, the parts follow as their own frames
    span<std::uint8_t> data;
    if(group == nullptr && message->data != nullptr) {
        data = message->data->getData();
    }
    const std::array<iovec, 2> payload{iovec{data.data(), data.size()}, iovec{const_cast<std::uint8_t*>(metadata.data()), metadata.size()}};
    auto result = sendFrame(socket, FrameKind::MESSAGE, payload.data(), static_cast<int>(payload.size()), data.size() + metadata.size());
    if(result != IoResult::OK || group == nullptr) {
        return result;
    }
    for(const auto& part : group->group) {
        result = sendMessage(socket, part.second);
        if(result != IoResult::OK) {
            return result;
        }
    }
    return IoResult::OK;
}

IoResult PeerLinkStream::sendFrame(const Socket& socket, FrameKind kind, const iovec* payload, int payloadCount, std::uint64_t payloadSize) {
    auto header = encodeFrameHeader(kind, payloadSize);
    std::array<iovec, 3> buffers{};
    buffers[0] = iovec{header.data(), header.size()};
    int count = 1;
    for(int i = 0; i < payloadCount && count < static_cast<int>(buffers.size()); i++) {
        buffers[count++] = payload[i];
    }
    const auto result = sendAll(socket, buffers.data(), count);
    if(result == IoResult::OK) {
        lastSend = std::chrono::steady_clock::now();
        bytesSent += PEERLINK_FRAME_HEADER_SIZE + payloadSize;
    }
    return result;
}

void PeerLinkStream::disconnect(const std::shared_ptr<Socket>& socket, const char* reason) {
    std::string peer;
    {
        std::lock_guard<std::mutex> lock(consumerMutex);
        if(consumer != socket) return;
        consumer.reset();
        peer = std::move(consumerPeer);
        consumerPeer.clear();
    }
    // Unblocks a send in progress; the descriptor closes with the last reference
    socket->shutdown();
    log().info("Peer-link stream '{}': consumer {} disconnected ({})", name, peer, reason);
}

// PeerLinkListener

PeerLinkListener::PeerLinkListener(Config config) : config(std::move(config)) {
    if(this->config.listenerId.size() > PEERLINK_MAX_LISTENER_ID_LENGTH) {
        throw std::invalid_argument("Peer-link listener id is too long");
    }
}

PeerLinkListener::~PeerLinkListener() {
    stop();
}

spdlog::logger& PeerLinkListener::log() const {
    return config.logger ? *config.logger : Logging::getInstance().logger;
}

void PeerLinkListener::start() {
    if(running) return;
    listenSocket = listenTcp(config.bindAddress, config.port);
    boundPort = localPort(listenSocket);
    running = true;
    acceptThread = std::thread([this]() { acceptLoop(); });
    heartbeatThread = std::thread([this]() { heartbeatLoop(); });
    log().info("Peer-link listener '{}' listening on {}", config.listenerId, formatEndpoint(config.bindAddress, boundPort));
}

void PeerLinkListener::stop() {
    {
        std::lock_guard<std::mutex> lock(heartbeatMutex);
        if(!running && !acceptThread.joinable() && !heartbeatThread.joinable()) return;
        running = false;
    }
    heartbeatCv.notify_all();
    // The accept thread joins the handshakes in flight before it exits
    if(acceptThread.joinable()) acceptThread.join();
    if(heartbeatThread.joinable()) heartbeatThread.join();
    listenSocket.close();
    std::vector<std::shared_ptr<PeerLinkStream>> current;
    {
        std::lock_guard<std::mutex> lock(streamsMutex);
        for(const auto& entry : streams) current.push_back(entry.second);
    }
    for(const auto& stream : current) {
        stream->detach("listener stopped");
    }
}

std::shared_ptr<PeerLinkStream> PeerLinkListener::registerStream(const std::string& name, const Token& token) {
    if(name.empty() || name.size() > PEERLINK_MAX_STREAM_NAME_LENGTH) {
        throw std::invalid_argument("Invalid peer-link stream name '" + name + "'");
    }
    std::lock_guard<std::mutex> lock(streamsMutex);
    if(streams.count(name) > 0) {
        throw std::runtime_error("Peer-link stream '" + name + "' is already registered");
    }
    auto stream = std::make_shared<PeerLinkStream>(name, token, config.sendTimeout, config.logger);
    streams.emplace(name, stream);
    return stream;
}

void PeerLinkListener::unregisterStream(const std::shared_ptr<PeerLinkStream>& stream) {
    if(stream == nullptr) return;
    {
        std::lock_guard<std::mutex> lock(streamsMutex);
        const auto it = streams.find(stream->getName());
        if(it != streams.end() && it->second == stream) {
            streams.erase(it);
        }
    }
    stream->detach("stream closed");
}

void PeerLinkListener::acceptLoop() {
    using listenerdetail::ACCEPT_POLL_INTERVAL;
    while(running) {
        joinHandshakes(false);
        pollfd pfd{listenSocket.get(), POLLIN, 0};
        const int ready = ::poll(&pfd, 1, static_cast<int>(ACCEPT_POLL_INTERVAL.count()));
        if(ready < 0 && errno != EINTR) {
            log().debug("Peer-link listener: poll() failed: {}", std::strerror(errno));
            std::this_thread::sleep_for(ACCEPT_POLL_INTERVAL);
        }
        if(ready <= 0 || !running) continue;
        sockaddr_storage address{};
        socklen_t length = sizeof(address);
        Socket socket(::accept(listenSocket.get(), reinterpret_cast<sockaddr*>(&address), &length));
        if(!socket.valid()) {
            // Out of descriptors or buffers leaves the connection in the backlog: back off instead of spinning
            if(errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK && errno != ECONNABORTED) {
                log().debug("Peer-link listener: accept() failed: {}", std::strerror(errno));
                std::this_thread::sleep_for(ACCEPT_POLL_INTERVAL);
            }
            continue;
        }
        const auto peer = listenerdetail::formatPeer(address);
        if(handshakes.size() >= listenerdetail::MAX_PENDING_HANDSHAKES) {
            log().debug("Peer-link listener: too many handshakes in progress, closing the connection from {}", peer);
            continue;
        }
        auto done = std::make_shared<std::atomic<bool>>(false);
        auto thread = std::thread([this, connection = std::move(socket), peer, done]() mutable {
            try {
                handleConnection(std::move(connection), peer);
            } catch(const std::exception& ex) {
                log().warn("Peer-link listener: handshake with {} failed: {}", peer, ex.what());
            }
            *done = true;
        });
        handshakes.push_back(Handshake{std::move(thread), std::move(done)});
    }
    joinHandshakes(true);
}

void PeerLinkListener::joinHandshakes(bool all) {
    for(auto it = handshakes.begin(); it != handshakes.end();) {
        if(all || *it->done) {
            it->thread.join();
            it = handshakes.erase(it);
        } else {
            ++it;
        }
    }
}

void PeerLinkListener::heartbeatLoop() {
    const auto tick = std::max(config.heartbeatInterval / 4, std::chrono::milliseconds(10));
    std::unique_lock<std::mutex> lock(heartbeatMutex);
    while(running) {
        heartbeatCv.wait_for(lock, tick, [this]() { return !running; });
        if(!running) break;
        lock.unlock();
        std::vector<std::shared_ptr<PeerLinkStream>> current;
        {
            std::lock_guard<std::mutex> streamsLock(streamsMutex);
            for(const auto& entry : streams) current.push_back(entry.second);
        }
        const auto now = std::chrono::steady_clock::now();
        for(const auto& stream : current) {
            stream->heartbeatIfIdle(now, config.heartbeatInterval);
        }
        lock.lock();
    }
}

void PeerLinkListener::handleConnection(Socket socket, const std::string& peer) {
    configureStreamSocket(socket);
    setReceiveTimeout(socket, config.handshakeTimeout);
    setSendTimeout(socket, config.handshakeTimeout);

    std::array<std::uint8_t, PEERLINK_HELLO_HEADER_SIZE> headerBytes{};
    if(receiveAll(socket, headerBytes.data(), headerBytes.size()) != IoResult::OK) {
        log().debug("Peer-link listener: no hello from {}", peer);
        return;
    }
    const auto hello = decodeHelloHeader(headerBytes.data());
    if(!hello || hello->nameLength > PEERLINK_MAX_STREAM_NAME_LENGTH) {
        log().debug("Peer-link listener: {} is not a peer-link consumer", peer);
        return;
    }
    std::string name(hello->nameLength, '\0');
    if(!name.empty() && receiveAll(socket, name.data(), name.size()) != IoResult::OK) {
        log().debug("Peer-link listener: incomplete hello from {}", peer);
        return;
    }

    auto reply = [&](ReplyStatus status) {
        auto bytes = encodeReply(Reply{status, PEERLINK_PROTOCOL_VERSION, config.listenerId});
        iovec buffer{bytes.data(), bytes.size()};
        sendAll(socket, &buffer, 1);
    };

    if(hello->version != PEERLINK_PROTOCOL_VERSION) {
        log().warn("Peer-link listener: {} uses protocol version {}, this device uses {}", peer, hello->version, PEERLINK_PROTOCOL_VERSION);
        reply(ReplyStatus::BAD_VERSION);
        return;
    }
    if((hello->flags & PEERLINK_HELLO_FLAG_PROBE) != 0) {
        log().debug("Peer-link listener: probe from {}", peer);
        reply(ReplyStatus::OK);
        return;
    }
    std::shared_ptr<PeerLinkStream> stream;
    {
        std::lock_guard<std::mutex> lock(streamsMutex);
        const auto it = streams.find(name);
        if(it != streams.end()) stream = it->second;
    }
    if(stream == nullptr) {
        log().debug("Peer-link listener: {} asked for unknown stream '{}'", peer, name);
        reply(ReplyStatus::RETRY_UNKNOWN_STREAM);
        return;
    }
    if(stream->getToken() != hello->token) {
        log().warn("Peer-link listener: {} asked for stream '{}' with a token of another pipeline", peer, name);
        reply(ReplyStatus::BAD_TOKEN);
        return;
    }
    // The consumer sends nothing after its hello; liveness on this side is the send timeout
    setReceiveTimeout(socket, std::chrono::milliseconds(0));
    stream->attach(std::move(socket), encodeReply(Reply{ReplyStatus::OK, PEERLINK_PROTOCOL_VERSION, config.listenerId}), peer);
}

}  // namespace peerlink
}  // namespace dai
