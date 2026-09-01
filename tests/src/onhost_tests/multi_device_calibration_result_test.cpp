#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <depthai/utility/Serialization.hpp>

using namespace dai;

namespace {

Extrinsics makeExtrinsics(const std::string& toDeviceId, CameraBoardSocket toSocket, const Point3f& translation) {
    Extrinsics result({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, translation, toSocket, LengthUnit::METER);
    result.toDeviceId = toDeviceId;
    return result;
}

}  // namespace

TEST_CASE("Multi-device calibration control exposes only lifecycle commands") {
    const auto start = MultiDeviceCalibrationControl::start();
    const auto stop = MultiDeviceCalibrationControl::stop();
    const auto reset = MultiDeviceCalibrationControl::reset();

    REQUIRE(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Start>(start->command));
    REQUIRE(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Stop>(stop->command));
    REQUIRE(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Reset>(reset->command));
}

TEST_CASE("Multi-device calibration result round-trips handler and edge diagnostics") {
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-b";
    edge.fromSocket = CameraBoardSocket::CAM_A;
    edge.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {1.0f, 2.0f, 3.0f});

    MultiDeviceCalibrationResult source;
    source.handler = MultiDeviceCalibrationHandler({edge});
    source.passed = true;
    source.complete = true;
    source.dataConfidence = 0.75;
    source.info = "one accepted edge";

    MultiDeviceCalibrationResult::EdgeDiagnostic diagnostic;
    diagnostic.fromDeviceId = "device-b";
    diagnostic.fromSocket = CameraBoardSocket::CAM_A;
    diagnostic.toDeviceId = "device-a";
    diagnostic.toSocket = CameraBoardSocket::CAM_A;
    diagnostic.accepted = true;
    diagnostic.dclConfidence = 0.9;
    diagnostic.reprojectionError = 0.25;
    diagnostic.sampsonError = 0.125;
    diagnostic.scaleSource = "stereo";
    diagnostic.scaleResidual = 0.01;
    source.diagnostics.push_back(diagnostic);

    for(const auto serializationType : {SerializationType::LIBNOP, SerializationType::JSON}) {
        const auto serialized = utility::serialize(source, serializationType);
        MultiDeviceCalibrationResult roundTrip;
        REQUIRE(utility::deserialize(serialized, roundTrip, serializationType));
        REQUIRE(roundTrip.handler.has_value());
        REQUIRE(roundTrip.passed);
        REQUIRE(roundTrip.complete);
        REQUIRE(roundTrip.dataConfidence == Catch::Approx(source.dataConfidence));
        REQUIRE(roundTrip.info == source.info);
        REQUIRE(roundTrip.diagnostics.size() == 1);
        REQUIRE(roundTrip.diagnostics.front().accepted);
        REQUIRE(roundTrip.diagnostics.front().scaleSource == "stereo");
        REQUIRE(roundTrip.diagnostics.front().scaleResidual == Catch::Approx(0.01));

        const auto resolved = roundTrip.handler->getExtrinsicsToOrigin("device-b", CameraBoardSocket::CAM_A);
        REQUIRE(resolved.has_value());
        REQUIRE(resolved->translation.x == Catch::Approx(1.0f));
    }
}

#ifdef DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT
TEST_CASE("Multi-device calibration node defaults and validates idle configuration") {
    Pipeline pipeline(false);
    auto node = pipeline.create<node::MultiDeviceCalibration>();

    REQUIRE(node->getSampleCount() == 10);
    REQUIRE_THROWS_AS(node->setSampleCount(0), std::runtime_error);
    REQUIRE_NOTHROW(node->setSampleCount(3));
    REQUIRE(node->getSampleCount() == 3);

    REQUIRE_THROWS_AS(node->setKnownDistance("device-a", CameraBoardSocket::CAM_A, "device-b", CameraBoardSocket::CAM_B, 0.0f), std::runtime_error);
    REQUIRE_THROWS_AS(node->setKnownDistance("device-a", CameraBoardSocket::CAM_A, "device-a", CameraBoardSocket::CAM_B, 1.0f), std::runtime_error);

    Extrinsics mismatchedDestination({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.0f, 0.0f, 0.0f}, CameraBoardSocket::CAM_C);
    mismatchedDestination.toDeviceId = "device-c";
    REQUIRE_THROWS_AS(node->setInitialGuess("device-a", CameraBoardSocket::CAM_A, "device-b", CameraBoardSocket::CAM_B, mismatchedDestination),
                      std::runtime_error);
}
#endif
