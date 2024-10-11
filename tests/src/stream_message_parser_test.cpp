#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

// TODO(themarpe) - fuzz me instead

constexpr auto MARKER_SIZE = 16;

TEST_CASE("Correct message") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessageToADatatype(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMessage(des);

    REQUIRE(ser == ser2);
}

TEST_CASE("Correct message, but padding corrupted, a warning should be printed") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);
    ser[ser.size() - 1] = 0x55;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessageToADatatype(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMessage(des);
    // Just for the binary compare to match, cause the same corruption in re-serialized msg
    ser2[ser.size() - 1] = 0x55;

    REQUIRE(ser == ser2);
}

TEST_CASE("Incorrect message bad size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 100;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessageToADatatype(&packet));
}

TEST_CASE("Incorrect message negative size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 200;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessageToADatatype(&packet));
}

TEST_CASE("Incorrect message too small size") {
    std::vector<uint8_t> ser = {0, 1, 2};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessageToADatatype(&packet));
}

TEST_CASE("Incorrect message too small size 2") {
    std::vector<uint8_t> ser = {0, 1, 1};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessageToADatatype(&packet));
}

TEST_CASE("Raw - Correct message") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessage(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMessage(des);

    REQUIRE(ser == ser2);
}

TEST_CASE("Raw - Incorrect message bad size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 100;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message negative size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMessage(frm);

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
