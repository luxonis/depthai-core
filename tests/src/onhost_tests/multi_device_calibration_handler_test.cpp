#include <catch2/catch_all.hpp>
#include <depthai/properties/GlobalProperties.hpp>
#include <depthai/utility/Serialization.hpp>
#include <limits>
#include <stdexcept>

#include "depthai/device/MultiDeviceCalibrationHandler.hpp"

using namespace dai;

namespace {

Extrinsics makeExtrinsics(const std::string& toDeviceId, CameraBoardSocket toSocket, const Point3f& translation, LengthUnit unit = LengthUnit::METER) {
    Extrinsics result({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, translation, toSocket, unit);
    result.toDeviceId = toDeviceId;
    return result;
}

}  // namespace

TEST_CASE("Multi-device calibration resolves a cross-device edge to the deterministic origin") {
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-b";
    edge.fromSocket = CameraBoardSocket::CAM_A;
    edge.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {1.0f, 2.0f, 3.0f});

    const MultiDeviceCalibrationHandler handler({edge});

    REQUIRE(handler.getDeviceSocket("device-a") == CameraBoardSocket::CAM_A);
    REQUIRE(handler.getDeviceSocket("device-b") == CameraBoardSocket::CAM_A);
    REQUIRE_FALSE(handler.getDeviceSocket("device-c").has_value());

    const auto rootBridge = handler.getExtrinsicsToOrigin("device-a", CameraBoardSocket::CAM_A);
    REQUIRE(rootBridge.has_value());
    REQUIRE(rootBridge->toDeviceId == "device-a");
    REQUIRE(rootBridge->toCameraSocket == CameraBoardSocket::CAM_A);
    REQUIRE(rootBridge->translation.x == Catch::Approx(0.0f));
    REQUIRE(rootBridge->translation.y == Catch::Approx(0.0f));
    REQUIRE(rootBridge->translation.z == Catch::Approx(0.0f));
    REQUIRE(rootBridge->lengthUnit == LengthUnit::METER);

    const auto childBridge = handler.getExtrinsicsToOrigin("device-b", CameraBoardSocket::CAM_A);
    REQUIRE(childBridge.has_value());
    REQUIRE(childBridge->toDeviceId == "device-a");
    REQUIRE(childBridge->translation.x == Catch::Approx(1.0f));
    REQUIRE(childBridge->translation.y == Catch::Approx(2.0f));
    REQUIRE(childBridge->translation.z == Catch::Approx(3.0f));
    REQUIRE(childBridge->lengthUnit == LengthUnit::METER);
}

TEST_CASE("Multi-device calibration handles chains, components, units, and serialization") {
    MultiDeviceExtrinsics first;
    first.fromDeviceId = "device-c";
    first.fromSocket = CameraBoardSocket::CAM_A;
    first.extrinsics = makeExtrinsics("device-b", CameraBoardSocket::CAM_A, {100.0f, 0.0f, 0.0f}, LengthUnit::CENTIMETER);

    MultiDeviceExtrinsics second;
    second.fromDeviceId = "device-b";
    second.fromSocket = CameraBoardSocket::CAM_A;
    second.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {2.0f, 0.0f, 0.0f}, LengthUnit::METER);

    MultiDeviceExtrinsics separate;
    separate.fromDeviceId = "device-z";
    separate.fromSocket = CameraBoardSocket::CAM_B;
    separate.extrinsics = makeExtrinsics("device-y", CameraBoardSocket::CAM_B, {1.0f, 0.0f, 0.0f}, LengthUnit::METER);

    const MultiDeviceCalibrationHandler handler({first, second, separate});

    const auto chainBridge = handler.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(chainBridge.has_value());
    REQUIRE(chainBridge->toDeviceId == "device-a");
    REQUIRE(chainBridge->translation.x == Catch::Approx(3.0f));
    REQUIRE(chainBridge->lengthUnit == LengthUnit::METER);

    const auto separateBridge = handler.getExtrinsicsToOrigin("device-z", CameraBoardSocket::CAM_B);
    REQUIRE(separateBridge.has_value());
    REQUIRE(separateBridge->toDeviceId == "device-y");
    REQUIRE(separateBridge->translation.x == Catch::Approx(1.0f));

    const auto serialized = utility::serialize(handler, SerializationType::JSON);
    MultiDeviceCalibrationHandler roundTrip;
    REQUIRE(utility::deserialize(serialized, roundTrip, SerializationType::JSON));
    const auto roundTripBridge = roundTrip.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(roundTripBridge.has_value());
    REQUIRE(roundTripBridge->toDeviceId == "device-a");
    REQUIRE(roundTripBridge->translation.x == Catch::Approx(3.0f));

    GlobalProperties properties;
    properties.multiDeviceCalibration = handler;
    for(const auto serializationType : {SerializationType::LIBNOP, SerializationType::JSON}) {
        const auto propertiesData = utility::serialize(properties, serializationType);
        GlobalProperties propertiesRoundTrip;
        REQUIRE(utility::deserialize(propertiesData, propertiesRoundTrip, serializationType));
        REQUIRE(propertiesRoundTrip.multiDeviceCalibration.has_value());
        const auto propertiesBridge = propertiesRoundTrip.multiDeviceCalibration->getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
        REQUIRE(propertiesBridge.has_value());
        REQUIRE(propertiesBridge->translation.x == Catch::Approx(3.0f));
    }
}

TEST_CASE("Multi-device calibration rejects invalid graph structure") {
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-a";
    edge.fromSocket = CameraBoardSocket::CAM_A;
    edge.extrinsics = makeExtrinsics("device-b", CameraBoardSocket::CAM_A, {});

    REQUIRE_NOTHROW(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{}));

    auto invalidSource = edge;
    invalidSource.fromDeviceId.clear();
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({invalidSource}), std::invalid_argument);

    auto invalidSocket = edge;
    invalidSocket.fromSocket = CameraBoardSocket::AUTO;
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({invalidSocket}), std::invalid_argument);

    auto nonFiniteTranslation = edge;
    nonFiniteTranslation.extrinsics.translation.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({nonFiniteTranslation}), std::invalid_argument);

    auto invalidRotation = edge;
    invalidRotation.extrinsics.rotationMatrix[0][0] = 2.0f;
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({invalidRotation}), std::invalid_argument);

    auto sameDevice = edge;
    sameDevice.extrinsics.toDeviceId = "device-a";
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({sameDevice}), std::invalid_argument);

    auto conflictingSocket = edge;
    conflictingSocket.fromDeviceId = "device-b";
    conflictingSocket.fromSocket = CameraBoardSocket::CAM_B;
    conflictingSocket.extrinsics = makeExtrinsics("device-c", CameraBoardSocket::CAM_A, {});
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({edge, conflictingSocket}), std::invalid_argument);

    MultiDeviceExtrinsics second;
    second.fromDeviceId = "device-b";
    second.fromSocket = CameraBoardSocket::CAM_A;
    second.extrinsics = makeExtrinsics("device-c", CameraBoardSocket::CAM_A, {});
    MultiDeviceExtrinsics cycle;
    cycle.fromDeviceId = "device-c";
    cycle.fromSocket = CameraBoardSocket::CAM_A;
    cycle.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {});
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({edge, second, cycle}), std::invalid_argument);

    auto reverseDuplicate = edge;
    reverseDuplicate.fromDeviceId = "device-b";
    reverseDuplicate.fromSocket = CameraBoardSocket::CAM_A;
    reverseDuplicate.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {});
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler({edge, reverseDuplicate}), std::invalid_argument);

    const MultiDeviceCalibrationHandler handler({edge});
    REQUIRE_THROWS_AS(handler.getExtrinsicsToOrigin("device-a", CameraBoardSocket::CAM_B), std::invalid_argument);
}
