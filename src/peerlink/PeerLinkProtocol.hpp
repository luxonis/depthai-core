#pragma once

// Peer-link: direct device-to-device transport for cross-device links (see Pipeline::setCrossDeviceLinkMode).
//
// The producing device runs one TCP listener per process. A consuming device connects to it, sends a Hello
// that names the stream it wants and the pipeline token, and receives a Reply. After an OK reply the
// producer sends frames: a MESSAGE frame carries one message in the exact XLink packet layout
// (data || metadata || datatype || metadata size || marker, see StreamMessageParser), a HEARTBEAT frame
// carries nothing and keeps an idle link alive. A MessageGroup is sent as the group frame, then one frame
// per part, in the order of MessageGroup::group.
//
//   Hello  (consumer -> producer): magic(8) | u16 version | u16 flags | token(16) | u16 nameLength | name
//   Reply  (producer -> consumer): u8 status | u16 version | u8 listenerIdLength | listenerId
//   Frame  (producer -> consumer): u8 kind | u64 payloadSize | payload
//
// All integers are little endian. This header only encodes and decodes; it has no socket code, so the host
// library can use it on every platform.

#include <array>
#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace dai {
namespace peerlink {

/// Version of the wire protocol. Both ends and the host must use the same version.
constexpr std::uint16_t PEERLINK_PROTOCOL_VERSION = 1;
/// TCP port of the peer-link listener on a device
constexpr std::uint16_t PEERLINK_DEFAULT_PORT = 11494;

constexpr std::size_t PEERLINK_TOKEN_SIZE = 16;
/// Random value generated per pipeline build; keeps stale sessions and other pipelines apart. Not a security boundary.
using Token = std::array<std::uint8_t, PEERLINK_TOKEN_SIZE>;

constexpr std::array<std::uint8_t, 8> PEERLINK_HELLO_MAGIC = {'D', 'A', 'I', 'P', 'E', 'E', 'R', 0};
constexpr std::size_t PEERLINK_MAX_STREAM_NAME_LENGTH = 256;
constexpr std::size_t PEERLINK_MAX_LISTENER_ID_LENGTH = 255;
/// Upper bound for one message (a 48 MP RGB frame is 144 MiB); the consumer allocates the announced size up front
constexpr std::uint64_t PEERLINK_MAX_PAYLOAD_SIZE = std::uint64_t{256} << 20;

constexpr std::size_t PEERLINK_HELLO_HEADER_SIZE = PEERLINK_HELLO_MAGIC.size() + 2 + 2 + PEERLINK_TOKEN_SIZE + 2;
constexpr std::size_t PEERLINK_REPLY_HEADER_SIZE = 1 + 2 + 1;
constexpr std::size_t PEERLINK_FRAME_HEADER_SIZE = 1 + 8;

/// Hello flag: only check that a listener is reachable, the stream name is empty
constexpr std::uint16_t PEERLINK_HELLO_FLAG_PROBE = 1U << 0U;

enum class ReplyStatus : std::uint8_t {
    OK = 0,
    /// The producer has no stream with this name (yet). The consumer retries.
    RETRY_UNKNOWN_STREAM = 1,
    /// The stream exists but belongs to another pipeline build
    BAD_TOKEN = 2,
    /// The versions differ; the reply carries the producer's version
    BAD_VERSION = 3,
};

enum class FrameKind : std::uint8_t {
    MESSAGE = 0,
    HEARTBEAT = 1,
};

struct Hello {
    std::uint16_t version = PEERLINK_PROTOCOL_VERSION;
    std::uint16_t flags = 0;
    Token token{};
    std::string streamName;
};

/// Fixed part of a Hello, the stream name of nameLength bytes follows it
struct HelloHeader {
    std::uint16_t version = 0;
    std::uint16_t flags = 0;
    Token token{};
    std::uint16_t nameLength = 0;
};

struct Reply {
    ReplyStatus status = ReplyStatus::OK;
    std::uint16_t version = PEERLINK_PROTOCOL_VERSION;
    std::string listenerId;
};

/// Fixed part of a Reply, the listener id of listenerIdLength bytes follows it
struct ReplyHeader {
    std::uint8_t status = 0;
    std::uint16_t version = 0;
    std::uint8_t listenerIdLength = 0;
};

struct FrameHeader {
    std::uint8_t kind = 0;
    std::uint64_t payloadSize = 0;
};

/// What a device reports about its peer-link listener (RPC getPeerLinkInfo)
struct PeerLinkInfo {
    std::uint16_t protocolVersion = 0;
    std::uint16_t port = 0;
    /// IPv4 addresses of the device's network interfaces, without loopback
    std::vector<std::string> addresses;
    /// Identifies the listener in probe replies, so a probe can tell the producer from another device at the same address
    std::string listenerId;
};

/// @throws std::invalid_argument when the stream name is longer than PEERLINK_MAX_STREAM_NAME_LENGTH
std::vector<std::uint8_t> encodeHello(const Hello& hello);
/// @returns std::nullopt when the magic does not match. `data` must hold PEERLINK_HELLO_HEADER_SIZE bytes.
std::optional<HelloHeader> decodeHelloHeader(const std::uint8_t* data);

/// @throws std::invalid_argument when the listener id is longer than PEERLINK_MAX_LISTENER_ID_LENGTH
std::vector<std::uint8_t> encodeReply(const Reply& reply);
/// `data` must hold PEERLINK_REPLY_HEADER_SIZE bytes
ReplyHeader decodeReplyHeader(const std::uint8_t* data);

std::array<std::uint8_t, PEERLINK_FRAME_HEADER_SIZE> encodeFrameHeader(FrameKind kind, std::uint64_t payloadSize);
/// `data` must hold PEERLINK_FRAME_HEADER_SIZE bytes
FrameHeader decodeFrameHeader(const std::uint8_t* data);

const char* toString(ReplyStatus status);

nlohmann::json toJson(const PeerLinkInfo& info);
/// @throws std::runtime_error when a required field is missing or has the wrong type
PeerLinkInfo peerLinkInfoFromJson(const nlohmann::json& json);

struct Endpoint {
    std::string host;
    std::uint16_t port = 0;
};

/// "host:port", or "[host]:port" for an IPv6 address
std::string formatEndpoint(const std::string& host, std::uint16_t port);
/// Parses "host:port" or "[ipv6]:port"; std::nullopt when malformed
std::optional<Endpoint> parseEndpoint(const std::string& endpoint);

}  // namespace peerlink
}  // namespace dai
