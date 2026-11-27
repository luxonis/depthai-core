#include "peerlink/PeerLinkClient.hpp"

#include <algorithm>
#include <array>
#include <utility>

#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace peerlink {

namespace clientdetail {

bool sendHello(const Socket& socket, const Hello& hello) {
    auto bytes = encodeHello(hello);
    iovec buffer{bytes.data(), bytes.size()};
    return sendAll(socket, &buffer, 1) == IoResult::OK;
}

bool readReply(const Socket& socket, ReplyHeader& header, std::string& listenerId) {
    std::array<std::uint8_t, PEERLINK_REPLY_HEADER_SIZE> bytes{};
    if(receiveAll(socket, bytes.data(), bytes.size()) != IoResult::OK) {
        return false;
    }
    header = decodeReplyHeader(bytes.data());
    listenerId.assign(header.listenerIdLength, '\0');
    return listenerId.empty() || receiveAll(socket, listenerId.data(), listenerId.size()) == IoResult::OK;
}

}  // namespace clientdetail

std::optional<std::string> probe(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout) {
    // The whole probe fits in `timeout`: the device answers the probePeer RPC within it
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    const auto remaining = [deadline]() {
        return std::max(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()), std::chrono::milliseconds(1));
    };
    Socket socket = connectTcp(host, port, timeout, nullptr, true);
    if(!socket.valid()) {
        return std::nullopt;
    }
    configureStreamSocket(socket);
    setSendTimeout(socket, remaining());
    Hello hello;
    hello.flags = PEERLINK_HELLO_FLAG_PROBE;
    if(!clientdetail::sendHello(socket, hello)) {
        return std::nullopt;
    }
    std::array<std::uint8_t, PEERLINK_REPLY_HEADER_SIZE> headerBytes{};
    if(receiveAllUntil(socket, headerBytes.data(), headerBytes.size(), deadline) != IoResult::OK) {
        return std::nullopt;
    }
    const auto header = decodeReplyHeader(headerBytes.data());
    if(header.status != static_cast<std::uint8_t>(ReplyStatus::OK)) {
        return std::nullopt;
    }
    std::string listenerId(header.listenerIdLength, '\0');
    if(!listenerId.empty() && receiveAllUntil(socket, listenerId.data(), listenerId.size(), deadline) != IoResult::OK) {
        return std::nullopt;
    }
    return listenerId;
}

const char* toString(ConnectStatus status) {
    switch(status) {
        case ConnectStatus::CONNECTED:
            return "connected";
        case ConnectStatus::UNREACHABLE:
            return "unreachable";
        case ConnectStatus::UNKNOWN_STREAM:
            return "stream not registered yet";
        case ConnectStatus::BAD_TOKEN:
            return "token mismatch";
        case ConnectStatus::BAD_VERSION:
            return "protocol version mismatch";
        case ConnectStatus::PROTOCOL_ERROR:
            return "protocol error";
    }
    return "unknown";
}

// PeerLinkClient

PeerLinkClient::PeerLinkClient(Config config) : config(std::move(config)), endpoints(this->config.endpoints) {}

void PeerLinkClient::setEndpoints(std::vector<std::string> newEndpoints) {
    std::lock_guard<std::mutex> lock(endpointsMutex);
    endpoints = std::move(newEndpoints);
}

PeerLinkClient::~PeerLinkClient() {
    disconnect();
}

ConnectStatus PeerLinkClient::connect(std::string* detail) {
    disconnect();
    std::string details;
    std::vector<std::string> targets;
    {
        std::lock_guard<std::mutex> lock(endpointsMutex);
        targets = endpoints;
    }
    for(const auto& target : targets) {
        if(interrupted) {
            details = "interrupted";
            break;
        }
        std::string reason;
        const auto status = connectEndpoint(target, reason);
        if(status == ConnectStatus::CONNECTED) {
            return status;
        }
        if(status != ConnectStatus::UNREACHABLE && status != ConnectStatus::PROTOCOL_ERROR) {
            // The producer answered; another address of the same device would answer the same
            if(detail) *detail = reason;
            return status;
        }
        if(!details.empty()) details += "; ";
        details += reason;
    }
    if(detail) *detail = targets.empty() ? "no producer endpoint" : details;
    return ConnectStatus::UNREACHABLE;
}

ConnectStatus PeerLinkClient::connectEndpoint(const std::string& target, std::string& detail) {
    const auto parsed = parseEndpoint(target);
    if(!parsed) {
        detail = "malformed endpoint '" + target + "'";
        return ConnectStatus::UNREACHABLE;
    }
    std::string error;
    Socket candidate = connectTcp(parsed->host, parsed->port, config.connectTimeout, &error);
    if(!candidate.valid()) {
        detail = target + ": " + error;
        return ConnectStatus::UNREACHABLE;
    }
    configureStreamSocket(candidate);
    setSendTimeout(candidate, config.connectTimeout);
    setReceiveTimeout(candidate, config.connectTimeout);
    {
        // Make the handshake interruptible
        std::lock_guard<std::mutex> lock(socketMutex);
        if(interrupted) {
            detail = "interrupted";
            return ConnectStatus::UNREACHABLE;
        }
        socket = std::move(candidate);
    }

    Hello hello;
    hello.token = config.token;
    hello.streamName = config.streamName;
    ReplyHeader reply;
    std::string listenerId;
    if(!clientdetail::sendHello(socket, hello) || !clientdetail::readReply(socket, reply, listenerId)) {
        disconnect();
        detail = target + ": no peer-link reply";
        return ConnectStatus::PROTOCOL_ERROR;
    }
    if(!config.listenerId.empty() && listenerId != config.listenerId) {
        // Another device has this address now (a DHCP lease changed hands, for example)
        disconnect();
        detail = target + ": listener '" + listenerId + "' is not the producer '" + config.listenerId + "'";
        return ConnectStatus::UNREACHABLE;
    }
    ConnectStatus status = ConnectStatus::PROTOCOL_ERROR;
    switch(static_cast<ReplyStatus>(reply.status)) {
        case ReplyStatus::OK:
            setReceiveTimeout(socket, config.livenessTimeout);
            endpoint = target;
            lastError.clear();
            return ConnectStatus::CONNECTED;
        case ReplyStatus::RETRY_UNKNOWN_STREAM:
            status = ConnectStatus::UNKNOWN_STREAM;
            break;
        case ReplyStatus::BAD_TOKEN:
            status = ConnectStatus::BAD_TOKEN;
            break;
        case ReplyStatus::BAD_VERSION:
            status = ConnectStatus::BAD_VERSION;
            break;
    }
    disconnect();
    detail = target + " (" + listenerId + "): " + toString(status);
    if(status == ConnectStatus::BAD_VERSION) {
        detail += ", producer version " + std::to_string(reply.version) + ", this version " + std::to_string(PEERLINK_PROTOCOL_VERSION);
    }
    return status;
}

std::shared_ptr<ADatatype> PeerLinkClient::receive() {
    if(!socket.valid()) {
        lastError = "not connected";
        return nullptr;
    }
    auto message = receiveMessage();
    if(message == nullptr) {
        disconnect();
    }
    return message;
}

std::shared_ptr<ADatatype> PeerLinkClient::receiveMessage() {
    while(true) {
        std::array<std::uint8_t, PEERLINK_FRAME_HEADER_SIZE> headerBytes{};
        if(!receiveExact(headerBytes.data(), headerBytes.size())) {
            return nullptr;
        }
        const auto header = decodeFrameHeader(headerBytes.data());
        if(header.kind == static_cast<std::uint8_t>(FrameKind::HEARTBEAT) && header.payloadSize == 0) {
            continue;
        }
        if(header.kind != static_cast<std::uint8_t>(FrameKind::MESSAGE)) {
            lastError = "unknown frame kind " + std::to_string(header.kind);
            return nullptr;
        }
        if(header.payloadSize > PEERLINK_MAX_PAYLOAD_SIZE) {
            lastError = "message of " + std::to_string(header.payloadSize) + " bytes exceeds the limit";
            return nullptr;
        }
        std::vector<std::uint8_t> payload(static_cast<std::size_t>(header.payloadSize));
        if(!receiveExact(payload.data(), payload.size())) {
            return nullptr;
        }
        std::shared_ptr<ADatatype> message;
        try {
            message = StreamMessageParser::parseMessage(std::move(payload));
        } catch(const std::exception& ex) {
            lastError = std::string("cannot parse message: ") + ex.what();
            return nullptr;
        }
        if(auto group = std::dynamic_pointer_cast<MessageGroup>(message)) {
            for(auto& part : group->group) {
                part.second = receiveMessage();
                if(part.second == nullptr) {
                    return nullptr;
                }
            }
        }
        return message;
    }
}

bool PeerLinkClient::receiveExact(void* data, std::size_t size) {
    if(size == 0) return true;
    const auto result = receiveAll(socket, data, size);
    if(result == IoResult::OK) {
        return true;
    }
    if(interrupted) {
        lastError = "interrupted";
    } else if(result == IoResult::TIMEOUT) {
        lastError = "no data for " + std::to_string(config.livenessTimeout.count()) + " ms";
    } else {
        lastError = toString(result);
    }
    return false;
}

void PeerLinkClient::disconnect() {
    std::lock_guard<std::mutex> lock(socketMutex);
    socket.close();
    endpoint.clear();
}

void PeerLinkClient::interrupt() {
    std::lock_guard<std::mutex> lock(socketMutex);
    interrupted = true;
    socket.shutdown();
}

// PeerLinkReceiver

PeerLinkReceiver::PeerLinkReceiver(Config config) : config(std::move(config)), client(this->config.client), retryInterval(this->config.minRetryInterval) {}

PeerLinkReceiver::~PeerLinkReceiver() {
    stop();
}

spdlog::logger& PeerLinkReceiver::log() const {
    return config.logger ? *config.logger : Logging::getInstance().logger;
}

std::shared_ptr<ADatatype> PeerLinkReceiver::receive() {
    const auto& name = config.client.streamName;
    while(!stopped) {
        if(!client.isConnected()) {
            std::string detail;
            const auto status = client.connect(&detail);
            if(stopped) break;
            if(status != ConnectStatus::CONNECTED) {
                // Log a new kind of failure once, repeats only at debug level
                const bool repeated = lastFailure == status;
                lastFailure = status;
                if(repeated || status == ConnectStatus::UNKNOWN_STREAM) {
                    log().debug("Peer-link stream '{}': {} ({}), retrying", name, toString(status), detail);
                } else if(status == ConnectStatus::BAD_TOKEN || status == ConnectStatus::BAD_VERSION) {
                    log().warn("Peer-link stream '{}': {} ({}), retrying", name, toString(status), detail);
                } else {
                    log().info("Peer-link stream '{}': {} ({}), retrying", name, toString(status), detail);
                }
                if(!waitRetry(retryInterval)) break;
                retryInterval = std::min(retryInterval * 2, config.maxRetryInterval);
                continue;
            }
            lastFailure.reset();
            retryInterval = config.minRetryInterval;
            connected = true;
            connections++;
            log().info("Peer-link stream '{}': connected to {}", name, client.getEndpoint());
        }
        const auto endpoint = client.getEndpoint();
        auto message = client.receive();
        if(message != nullptr) {
            return message;
        }
        connected = false;
        if(stopped) break;
        log().info("Peer-link stream '{}': connection to {} lost ({}), reconnecting", name, endpoint, client.getLastError());
    }
    connected = false;
    return nullptr;
}

void PeerLinkReceiver::stop() {
    {
        std::lock_guard<std::mutex> lock(waitMutex);
        stopped = true;
    }
    client.interrupt();
    waitCv.notify_all();
}

void PeerLinkReceiver::setEndpoints(std::vector<std::string> endpoints) {
    client.setEndpoints(std::move(endpoints));
    {
        std::lock_guard<std::mutex> lock(waitMutex);
        endpointsChanged = true;
    }
    waitCv.notify_all();
}

bool PeerLinkReceiver::waitRetry(std::chrono::milliseconds interval) {
    std::unique_lock<std::mutex> lock(waitMutex);
    waitCv.wait_for(lock, interval, [this]() { return stopped.load() || endpointsChanged; });
    if(endpointsChanged) {
        // New endpoints: try them now, from the shortest interval
        endpointsChanged = false;
        retryInterval = config.minRetryInterval / 2;
    }
    return !stopped;
}

}  // namespace peerlink
}  // namespace dai
