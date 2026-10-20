#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "basalt/BasaltVIOGeometry.hpp"
#include "depthai/pipeline/datatype/Odometry.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"

namespace {
// Exact legacy field list, deliberately independent of the new Odometry decoder.
struct LegacyOdometry : dai::TransformData {
    dai::Point3d velocity;
    DEPTHAI_SERIALIZE(LegacyOdometry, sequenceNum, ts, tsDevice, tsSystem, transform, velocity);
};
struct FutureExtension {
    std::uint32_t version = 99;
    dai::Extrinsics extrinsics;
    NOP_STRUCTURE(FutureExtension, version, extrinsics);
};
dai::Extrinsics reference() {
    dai::Extrinsics result({{0, -1, 0}, {0, 0, -1}, {1, 0, 0}}, {10, 20, 30}, dai::CameraBoardSocket::CAM_B, dai::LengthUnit::CENTIMETER);
    result.toDeviceId = "test-device";
    return result;
}
}  // namespace

TEST_CASE("Odometry tracked frame survives all message serialization formats", "[odometry][serialization]") {
    dai::Odometry original(1, 2, 3, 0, 0, 0, 1, {4, 5, 6});
    original.setSequenceNum(42);
    original.trackedFrameExtrinsics = reference();
    for(const auto type : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
        const auto data = dai::utility::serialize(original, type);
        dai::Odometry copy;
        REQUIRE(dai::utility::deserialize(data, copy, type));
        REQUIRE(copy.getSequenceNum() == 42);
        REQUIRE(copy.getTranslation().x == 1);
        REQUIRE(copy.velocity.z == 6);
        REQUIRE(copy.trackedFrameExtrinsics);
        REQUIRE(copy.trackedFrameExtrinsics->isEqualExtrinsics(*original.trackedFrameExtrinsics));
        REQUIRE(copy.trackedFrameExtrinsics->toDeviceId == "test-device");

        LegacyOdometry legacy;
        REQUIRE(dai::utility::deserialize(data, legacy, type));
        REQUIRE(legacy.getTranslation().y == 2);
        REQUIRE(legacy.velocity.y == 5);
        // Reading an old message into a reused object must clear stale metadata.
        REQUIRE(dai::utility::deserialize(dai::utility::serialize(legacy, type), copy, type));
        REQUIRE_FALSE(copy.trackedFrameExtrinsics);
        REQUIRE(copy.velocity.z == 6);
    }
}

TEST_CASE("Odometry without frame metadata retains the legacy binary encoding", "[odometry][serialization]") {
    dai::Odometry message(1, 2, 3, 0, 0, 0, 1, {4, 5, 6});
    LegacyOdometry legacy;
    legacy.transform = message.transform;
    legacy.velocity = message.velocity;
    legacy.setTimestamp(message.getTimestamp());
    legacy.setTimestampDevice(message.getTimestampDevice());
    REQUIRE(dai::utility::serialize(message) == dai::utility::serialize(legacy));
}

TEST_CASE("Odometry frame metadata round trips through the stream parser", "[odometry][serialization]") {
    dai::Odometry original(1, 2, 3, 0, 0, 0, 1, {4, 5, 6});
    SECTION("present") {
        original.trackedFrameExtrinsics = reference();
    }
    SECTION("absent") {}
    auto bytes = dai::StreamMessageParser::serializeMetadata(original);
    streamPacketDesc_t packet{};
    packet.data = bytes.data();
    packet.length = bytes.size();
    packet.fd = -1;
    const auto copy = std::dynamic_pointer_cast<dai::Odometry>(dai::StreamMessageParser::parseMessage(&packet));
    REQUIRE(copy);
    REQUIRE(copy->getDatatype() == dai::DatatypeEnum::Odometry);
    REQUIRE(dai::utility::serialize(*copy) == dai::utility::serialize(original));
}

TEST_CASE("Odometry rejects truncated and unsupported frame extensions", "[odometry][serialization]") {
    dai::Odometry original;
    auto bytes = dai::utility::serialize(original);
    SECTION("truncated") {
        original.trackedFrameExtrinsics = reference();
        bytes = dai::utility::serialize(original);
        bytes.pop_back();
    }
    SECTION("unsupported version") {
        const auto extension = dai::utility::serialize(FutureExtension{99, reference()});
        bytes.insert(bytes.end(), extension.begin(), extension.end());
    }
    SECTION("trailing garbage") {
        bytes.push_back(0);
    }
    dai::Odometry copy;
    REQUIRE_THROWS(dai::utility::deserialize(bytes, copy));
    REQUIRE_FALSE(copy.trackedFrameExtrinsics);
}

TEST_CASE("Basalt tracked metadata includes FLU axes and a non-first calibration reference", "[basalt][geometry]") {
    dai::CalibrationHandler calibration;
    const std::vector<std::vector<float>> rotation{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    calibration.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, {{300, 0, 320}, {0, 300, 200}, {0, 0, 1}}, 640, 400);
    calibration.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, {{300, 0, 320}, {0, 300, 200}, {0, 0, 1}}, 640, 400);
    calibration.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_A, rotation, {10, 20, 30}, {11, 22, 33});
    dai::ImgFrame first;
    first.setInstanceNum(static_cast<int>(dai::CameraBoardSocket::CAM_B));
    auto input = reference();
    input.toCameraSocket = dai::CameraBoardSocket::CAM_A;
    first.transformation.setExtrinsics(input);
    const auto metadata = dai::node::detail::getBasaltTrackedFrameExtrinsics(calibration, first, "test-device");
    const auto matrix = metadata.getTransformationMatrix(false, dai::LengthUnit::METER);
    REQUIRE(metadata.toCameraSocket == dai::CameraBoardSocket::CAM_A);
    REQUIRE(metadata.toDeviceId == "test-device");
    REQUIRE(matrix[0][3] == Catch::Approx(0.1));
    REQUIRE(matrix[1][3] == Catch::Approx(0.2));
    REQUIRE(matrix[2][3] == Catch::Approx(0.3));
    // Camera B is rotated 90 degrees around reference Z. Compose that rotation
    // with FLU -> optical: forward becomes reference Z, left becomes -Y.
    REQUIRE(matrix[2][0] == Catch::Approx(1));
    REQUIRE(matrix[1][1] == Catch::Approx(-1));
    REQUIRE(matrix[0][2] == Catch::Approx(1));
    REQUIRE_THROWS(dai::node::detail::getBasaltTrackedFrameExtrinsics(calibration, first, "another-device"));
}

TEST_CASE("Odometry explicit null JSON metadata clears a reused message", "[odometry][serialization]") {
    dai::Odometry message;
    message.trackedFrameExtrinsics = reference();
    nlohmann::json json = message;
    json["trackedFrameExtrinsics"] = nullptr;
    json.get_to(message);
    REQUIRE_FALSE(message.trackedFrameExtrinsics);
    json.erase("transform");
    REQUIRE_THROWS(json.get_to(message));
}
