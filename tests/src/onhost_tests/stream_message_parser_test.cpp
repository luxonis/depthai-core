#include <catch2/catch_all.hpp>
#include <limits>
#include <depthai/beta/datatype/ToFStereoFusionConfig.hpp>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

// TODO(themarpe) - fuzz me instead

constexpr auto MARKER_SIZE = 16;

TEST_CASE("Correct message") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessage(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMetadata(des);

    REQUIRE(ser == ser2);
}

TEST_CASE("Incorrect message bad size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 100;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Incorrect message negative size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 200;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Incorrect message too small size") {
    std::vector<uint8_t> ser = {0, 1, 2};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Incorrect message too small size 2") {
    std::vector<uint8_t> ser = {0, 1, 1};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Correct message") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessage(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMetadata(des);

    REQUIRE(ser == ser2);
}

TEST_CASE("Raw - Incorrect message bad size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 100;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message negative size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 200;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message too small size") {
    std::vector<uint8_t> ser = {0, 1, 2};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message too small size 2") {
    std::vector<uint8_t> ser = {0, 1, 1};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

#ifndef DEPTHAI_HAVE_BETA
TEST_CASE("Beta parser Config has a clear error when beta support is disabled") {
    dai::Buffer buffer;
    auto serialized = dai::StreamMessageParser::serializeMetadata(buffer);

    constexpr std::size_t TRAILER_SIZE = 8 + MARKER_SIZE;
    REQUIRE(serialized.size() >= TRAILER_SIZE);
    const auto datatype = static_cast<std::uint32_t>(dai::DatatypeEnum::FastSAMParserConfig);
    const auto datatypeOffset = serialized.size() - TRAILER_SIZE;
    serialized[datatypeOffset] = static_cast<std::uint8_t>(datatype);
    serialized[datatypeOffset + 1] = static_cast<std::uint8_t>(datatype >> 8U);
    serialized[datatypeOffset + 2] = static_cast<std::uint8_t>(datatype >> 16U);
    serialized[datatypeOffset + 3] = static_cast<std::uint8_t>(datatype >> 24U);

    streamPacketDesc_t packet{};
    packet.data = serialized.data();
    packet.length = serialized.size();
    packet.fd = -1;

    REQUIRE_THROWS_WITH(dai::StreamMessageParser::parseMessage(&packet),
                        Catch::Matchers::ContainsSubstring("Cannot parse beta datatype: depthai-core was built without beta support"));
}
#endif

TEST_CASE("ToF stereo fusion config survives transport", "[fusion][config]") {
    dai::beta::ToFStereoFusionConfig config;
    for(float threshold : {0.0f, 0.75f, 1.0f}) {
        config.setConfidenceThreshold(threshold);
        auto bytes = dai::StreamMessageParser::serializeMetadata(config);
        streamPacketDesc_t packet{};
        packet.data = bytes.data();
        packet.length = bytes.size();
        packet.fd = -1;
        auto parsed = std::dynamic_pointer_cast<dai::beta::ToFStereoFusionConfig>(dai::StreamMessageParser::parseMessage(&packet));
        REQUIRE(parsed);
        REQUIRE(parsed->getDatatype() == dai::DatatypeEnum::ToFStereoFusionConfig);
        REQUIRE(parsed->confidenceThreshold == threshold);
    }
    REQUIRE_THROWS_AS(config.setConfidenceThreshold(-0.1f), std::invalid_argument);
    REQUIRE_THROWS_AS(config.setConfidenceThreshold(1.1f), std::invalid_argument);
    REQUIRE_THROWS_AS(config.setConfidenceThreshold(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
}
