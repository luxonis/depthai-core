#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <cstring>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

// TODO(themarpe) - fuzz me instead

constexpr auto MARKER_SIZE = 16;

TEST_CASE("Promoted ImgDetectionsFilterConfig parses without beta dispatch") {
    dai::ImgDetectionsFilterConfig config;
    config.minConfidence = 0.42f;
    config.labelsToKeep = std::vector<uint32_t>{2, 7};
    const auto metadata = dai::StreamMessageParser::serializeMetadata(config);

    auto restored = std::dynamic_pointer_cast<dai::ImgDetectionsFilterConfig>(dai::StreamMessageParser::parseMessage(metadata));
    REQUIRE(restored != nullptr);
    REQUIRE(restored->minConfidence == config.minConfidence);
    REQUIRE(restored->labelsToKeep == config.labelsToKeep);
    REQUIRE(dai::StreamMessageParser::serializeMetadata(restored) == metadata);
}

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

TEST_CASE("Message parsing retains transferred payload storage", "[copy]") {
    dai::ImgFrame frame;
    frame.setSize(4, 2).setType(dai::ImgFrame::Type::RAW8);
    const std::vector<uint8_t> pixels{1, 2, 3, 4, 5, 6, 7, 8};
    frame.setData(pixels);
    const auto metadata = dai::StreamMessageParser::serializeMetadata(frame);
    auto serialized = pixels;
    serialized.insert(serialized.end(), metadata.begin(), metadata.end());
    auto* original = serialized.data();
    auto message = dai::StreamMessageParser::parseMessage(std::move(serialized));
    REQUIRE(message->data->getData().data() == original);
    REQUIRE(message->data->getSize() == pixels.size());
    REQUIRE(std::equal(pixels.begin(), pixels.end(), message->data->getData().begin()));
    REQUIRE(dai::StreamMessageParser::serializeMetadata(message) == metadata);

    dai::StreamPacketDesc packet;
    packet.length = pixels.size() + metadata.size();
    packet.data = static_cast<uint8_t*>(std::malloc(packet.length));
    REQUIRE(packet.data != nullptr);
    std::memcpy(packet.data, pixels.data(), pixels.size());
    std::memcpy(packet.data + pixels.size(), metadata.data(), metadata.size());
    auto* packetStorage = packet.data;
    auto moved = dai::StreamMessageParser::parseMessage(std::move(packet));
    REQUIRE(packet.data == nullptr);
    REQUIRE(moved->data->getData().data() == packetStorage);
    REQUIRE(moved->data->getSize() == pixels.size());
    REQUIRE(std::equal(pixels.begin(), pixels.end(), moved->data->getData().begin()));

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(std::vector<uint8_t>{1, 2, 3}));
}
