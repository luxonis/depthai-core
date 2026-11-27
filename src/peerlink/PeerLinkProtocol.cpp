#include "peerlink/PeerLinkProtocol.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace dai {
namespace peerlink {

namespace wire {

template <typename T>
void appendLE(std::vector<std::uint8_t>& out, T value) {
    for(std::size_t i = 0; i < sizeof(T); i++) {
        out.push_back(static_cast<std::uint8_t>((static_cast<std::uint64_t>(value) >> (8 * i)) & 0xFFU));
    }
}

template <typename T>
T readLE(const std::uint8_t* data) {
    std::uint64_t value = 0;
    for(std::size_t i = 0; i < sizeof(T); i++) {
        value |= static_cast<std::uint64_t>(data[i]) << (8 * i);
    }
    return static_cast<T>(value);
}

}  // namespace wire

std::vector<std::uint8_t> encodeHello(const Hello& hello) {
    if(hello.streamName.size() > PEERLINK_MAX_STREAM_NAME_LENGTH) {
        throw std::invalid_argument("Peer-link stream name is too long: " + hello.streamName);
    }
    std::vector<std::uint8_t> out;
    out.reserve(PEERLINK_HELLO_HEADER_SIZE + hello.streamName.size());
    out.insert(out.end(), PEERLINK_HELLO_MAGIC.begin(), PEERLINK_HELLO_MAGIC.end());
    wire::appendLE<std::uint16_t>(out, hello.version);
    wire::appendLE<std::uint16_t>(out, hello.flags);
    out.insert(out.end(), hello.token.begin(), hello.token.end());
    wire::appendLE<std::uint16_t>(out, static_cast<std::uint16_t>(hello.streamName.size()));
    out.insert(out.end(), hello.streamName.begin(), hello.streamName.end());
    return out;
}

std::optional<HelloHeader> decodeHelloHeader(const std::uint8_t* data) {
    if(std::memcmp(data, PEERLINK_HELLO_MAGIC.data(), PEERLINK_HELLO_MAGIC.size()) != 0) {
        return std::nullopt;
    }
    const std::uint8_t* p = data + PEERLINK_HELLO_MAGIC.size();
    HelloHeader header;
    header.version = wire::readLE<std::uint16_t>(p);
    p += 2;
    header.flags = wire::readLE<std::uint16_t>(p);
    p += 2;
    std::copy(p, p + PEERLINK_TOKEN_SIZE, header.token.begin());
    p += PEERLINK_TOKEN_SIZE;
    header.nameLength = wire::readLE<std::uint16_t>(p);
    return header;
}

std::vector<std::uint8_t> encodeReply(const Reply& reply) {
    if(reply.listenerId.size() > PEERLINK_MAX_LISTENER_ID_LENGTH) {
        throw std::invalid_argument("Peer-link listener id is too long: " + reply.listenerId);
    }
    std::vector<std::uint8_t> out;
    out.reserve(PEERLINK_REPLY_HEADER_SIZE + reply.listenerId.size());
    out.push_back(static_cast<std::uint8_t>(reply.status));
    wire::appendLE<std::uint16_t>(out, reply.version);
    out.push_back(static_cast<std::uint8_t>(reply.listenerId.size()));
    out.insert(out.end(), reply.listenerId.begin(), reply.listenerId.end());
    return out;
}

ReplyHeader decodeReplyHeader(const std::uint8_t* data) {
    ReplyHeader header;
    header.status = data[0];
    header.version = wire::readLE<std::uint16_t>(data + 1);
    header.listenerIdLength = data[3];
    return header;
}

std::array<std::uint8_t, PEERLINK_FRAME_HEADER_SIZE> encodeFrameHeader(FrameKind kind, std::uint64_t payloadSize) {
    std::array<std::uint8_t, PEERLINK_FRAME_HEADER_SIZE> out{};
    out[0] = static_cast<std::uint8_t>(kind);
    for(std::size_t i = 0; i < 8; i++) {
        out[1 + i] = static_cast<std::uint8_t>((payloadSize >> (8 * i)) & 0xFFU);
    }
    return out;
}

FrameHeader decodeFrameHeader(const std::uint8_t* data) {
    FrameHeader header;
    header.kind = data[0];
    header.payloadSize = wire::readLE<std::uint64_t>(data + 1);
    return header;
}

const char* toString(ReplyStatus status) {
    switch(status) {
        case ReplyStatus::OK:
            return "OK";
        case ReplyStatus::RETRY_UNKNOWN_STREAM:
            return "unknown stream";
        case ReplyStatus::BAD_TOKEN:
            return "token mismatch";
        case ReplyStatus::BAD_VERSION:
            return "protocol version mismatch";
    }
    return "unknown status";
}

nlohmann::json toJson(const PeerLinkInfo& info) {
    return nlohmann::json{
        {"protocolVersion", info.protocolVersion},
        {"port", info.port},
        {"addresses", info.addresses},
        {"listenerId", info.listenerId},
    };
}

PeerLinkInfo peerLinkInfoFromJson(const nlohmann::json& json) {
    try {
        PeerLinkInfo info;
        info.protocolVersion = json.at("protocolVersion").get<std::uint16_t>();
        info.port = json.at("port").get<std::uint16_t>();
        info.addresses = json.at("addresses").get<std::vector<std::string>>();
        info.listenerId = json.at("listenerId").get<std::string>();
        return info;
    } catch(const nlohmann::json::exception& ex) {
        throw std::runtime_error(std::string("Malformed peer-link info: ") + ex.what());
    }
}

std::string formatEndpoint(const std::string& host, std::uint16_t port) {
    if(host.find(':') != std::string::npos) {
        return "[" + host + "]:" + std::to_string(port);
    }
    return host + ":" + std::to_string(port);
}

std::optional<Endpoint> parseEndpoint(const std::string& endpoint) {
    std::string host;
    std::string portString;
    if(!endpoint.empty() && endpoint.front() == '[') {
        const auto close = endpoint.find(']');
        if(close == std::string::npos || close + 1 >= endpoint.size() || endpoint[close + 1] != ':') {
            return std::nullopt;
        }
        host = endpoint.substr(1, close - 1);
        portString = endpoint.substr(close + 2);
    } else {
        const auto colon = endpoint.rfind(':');
        if(colon == std::string::npos || endpoint.find(':') != colon) {
            return std::nullopt;
        }
        host = endpoint.substr(0, colon);
        portString = endpoint.substr(colon + 1);
    }
    if(host.empty() || portString.empty() || portString.size() > 5
       || !std::all_of(portString.begin(), portString.end(), [](char c) { return c >= '0' && c <= '9'; })) {
        return std::nullopt;
    }
    const auto port = std::stoul(portString);
    if(port == 0 || port > 65535) {
        return std::nullopt;
    }
    return Endpoint{host, static_cast<std::uint16_t>(port)};
}

}  // namespace peerlink
}  // namespace dai
