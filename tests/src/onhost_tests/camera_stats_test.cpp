#include <catch2/catch_test_macros.hpp>
#include <limits>
#include <vector>

#include "depthai/common/CameraStats.hpp"

TEST_CASE("Camera statistics preserve unavailable values and 64-bit counters", "[camera-stats]") {
    dai::CameraStats stats;
    stats.socket = dai::CameraBoardSocket::CAM_D;
    stats.cameraId = 7;
    stats.status = dai::CameraStats::Status::AVAILABLE;
    stats.sessionId = "capture-session/source-generation";
    stats.timestampNs = (uint64_t{1} << 40) + 3;
    stats.cslTimestampNs = stats.timestampNs - 1;
    stats.completedCaptures = std::optional<uint64_t>{};
    stats.halDeviceErrors = std::optional<uint64_t>{0};
    stats.halRequestErrors = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 2};
    stats.halResultErrors = std::optional<uint64_t>{};
    stats.halBufferErrors = std::optional<uint64_t>{0};
    stats.cslErrorEvents = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 5};
    stats.linkStalledEvents = std::optional<uint64_t>{};
    stats.csidFatalEvents = std::optional<uint64_t>{0};
    stats.csidFifoOverflowEvents = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 8};
    stats.csidRecoveryOverflowEvents = std::optional<uint64_t>{};
    stats.csidLaneFifoOverflowEvents = std::optional<uint64_t>{0};
    stats.csidPixelCountMismatchEvents = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 11};
    stats.csidPacketHeaderCorruptionEvents = std::optional<uint64_t>{};
    stats.csidMissingPacketHeaderEvents = std::optional<uint64_t>{0};
    stats.csidSensorSwitchingEvents = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 14};
    stats.csidUnboundedFrameEvents = std::optional<uint64_t>{};
    stats.icpNoMemoryEvents = std::optional<uint64_t>{0};
    stats.icpSystemFailureEvents = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 17};
    stats.csidMissingEotEvents = std::optional<uint64_t>{};
    stats.csidPayloadCorruptionEvents = std::optional<uint64_t>{0};
    stats.sensorStreamOffFailureEvents = std::optional<uint64_t>{std::numeric_limits<uint64_t>::max() - 20};
    const std::vector<dai::CameraStats> expected{stats, dai::CameraStats{}};
    std::vector<uint8_t> bytes;
    std::vector<dai::CameraStats> actual;

    SECTION("RPC JSON MessagePack representation") {
        REQUIRE(dai::utility::serialize<dai::SerializationType::JSON_MSGPACK>(expected, bytes));
        REQUIRE(dai::utility::deserialize<dai::SerializationType::JSON_MSGPACK>(bytes.data(), bytes.size(), actual));
    }
    SECTION("Shared libnop representation") {
        REQUIRE(dai::utility::serialize<dai::SerializationType::LIBNOP>(expected, bytes));
        REQUIRE(dai::utility::deserialize<dai::SerializationType::LIBNOP>(bytes.data(), bytes.size(), actual));
    }
    REQUIRE(nlohmann::json(actual) == nlohmann::json(expected));
    REQUIRE(actual[0].halDeviceErrors == 0);
    REQUIRE_FALSE(actual[0].completedCaptures.has_value());
    REQUIRE(actual[0].halRequestErrors.value() > std::numeric_limits<uint32_t>::max());
}

TEST_CASE("Camera statistics lifecycle states survive RPC serialization", "[camera-stats]") {
    for(const auto status : {dai::CameraStats::Status::AVAILABLE,
                             dai::CameraStats::Status::INACTIVE,
                             dai::CameraStats::Status::UNSUPPORTED,
                             dai::CameraStats::Status::UNAVAILABLE}) {
        dai::CameraStats source;
        source.status = status;
        std::vector<uint8_t> bytes;
        dai::CameraStats decoded;
        REQUIRE(dai::utility::serialize<dai::SerializationType::JSON_MSGPACK>(source, bytes));
        REQUIRE(dai::utility::deserialize<dai::SerializationType::JSON_MSGPACK>(bytes.data(), bytes.size(), decoded));
        REQUIRE(decoded.status == status);
        REQUIRE_FALSE(decoded.completedCaptures);
        REQUIRE_FALSE(decoded.cslErrorEvents);
    }
}
