#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <depthai/utility/Serialization.hpp>

#include "beta/node/MultiDeviceCalibrationUtils.hpp"

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

TEST_CASE("Multi-device calibration result round-trips handler and aggregate quality") {
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-b";
    edge.fromSocket = CameraBoardSocket::CAM_A;
    edge.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {1.0f, 2.0f, 3.0f});

    MultiDeviceCalibrationResult source;
    source.handler = MultiDeviceCalibrationHandler({edge});
    source.passed = true;
    source.dataConfidence = 0.75;
    source.sampsonError = 0.125;
    source.info = "one accepted edge";

    for(const auto serializationType : {SerializationType::LIBNOP, SerializationType::JSON}) {
        const auto serialized = utility::serialize(source, serializationType);
        MultiDeviceCalibrationResult roundTrip;
        REQUIRE(utility::deserialize(serialized, roundTrip, serializationType));
        REQUIRE(roundTrip.handler.has_value());
        REQUIRE(roundTrip.passed);
        REQUIRE(roundTrip.dataConfidence == Catch::Approx(source.dataConfidence));
        REQUIRE(roundTrip.sampsonError == Catch::Approx(source.sampsonError));
        REQUIRE(roundTrip.info == source.info);

        const auto resolved = roundTrip.handler->getExtrinsicsToOrigin("device-b", CameraBoardSocket::CAM_A);
        REQUIRE(resolved.has_value());
        REQUIRE(resolved->translation.x == Catch::Approx(1.0f));
    }
}

TEST_CASE("Pairwise initial guesses retain rotated and disconnected components") {
    using beta::node::detail::MultiDeviceTransform;
    using beta::node::detail::PairwiseDeviceTransform;

    const MultiDeviceTransform referenceToE{{1.0f, 0.0f, 0.0f, 4.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    const MultiDeviceTransform bToC{{0.0f, -1.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    const MultiDeviceTransform cToD{{1.0f, 0.0f, 0.0f, 2.0f}, {0.0f, 1.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    const std::vector<PairwiseDeviceTransform> guesses{
        {"device-b", "device-c", bToC},
        {"device-c", "device-d", cToD},
        {"device-a", "device-e", referenceToE},
    };

    const auto referenceToDevice = beta::node::detail::makeReferenceRelativeTransforms("device-a", guesses);

    REQUIRE(referenceToDevice.size() == 5);
    REQUIRE(referenceToDevice.at("device-b") == beta::node::detail::identityTransform());
    REQUIRE(referenceToDevice.at("device-c") == bToC);
    const auto& referenceToD = referenceToDevice.at("device-d");
    REQUIRE(referenceToD[0][0] == Catch::Approx(0.0f));
    REQUIRE(referenceToD[0][1] == Catch::Approx(-1.0f));
    REQUIRE(referenceToD[1][0] == Catch::Approx(1.0f));
    REQUIRE(referenceToD[0][3] == Catch::Approx(3.0f));
    REQUIRE(referenceToD[1][3] == Catch::Approx(1.0f));
    REQUIRE(referenceToDevice.at("device-e") == referenceToE);

    const auto deviceDToReference = beta::node::detail::inverseRigidTransform(referenceToD);
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-d";
    edge.fromSocket = CameraBoardSocket::CAM_A;
    const auto translation = matrix::extractTranslationVector(deviceDToReference);
    edge.extrinsics = Extrinsics(matrix::extractRotationMatrix(deviceDToReference),
                                 Point3f(translation[0], translation[1], translation[2]),
                                 CameraBoardSocket::CAM_A,
                                 LengthUnit::METER);
    edge.extrinsics.toDeviceId = "device-a";

    const MultiDeviceCalibrationHandler handler({edge});
    const auto resolved = handler.getExtrinsicsToOrigin("device-d", CameraBoardSocket::CAM_A);
    REQUIRE(resolved.has_value());
    REQUIRE(resolved->translation.x == Catch::Approx(-1.0f));
    REQUIRE(resolved->translation.y == Catch::Approx(3.0f));
    REQUIRE(resolved->rotationMatrix[0][1] == Catch::Approx(1.0f));
    REQUIRE(resolved->rotationMatrix[1][0] == Catch::Approx(-1.0f));
}

#if defined(DEPTHAI_HAVE_BETA) && defined(DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT)
TEST_CASE("Multi-device calibration node defaults and validates idle configuration") {
    Pipeline pipeline(false);
    auto node = pipeline.create<beta::node::MultiDeviceCalibration>();

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

    Extrinsics forwardGuess({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {0.25f, 0.0f, 0.0f}, CameraBoardSocket::CAM_B, LengthUnit::METER);
    forwardGuess.toDeviceId = "device-b";
    REQUIRE_NOTHROW(node->setInitialGuess("device-a", CameraBoardSocket::CAM_A, "device-b", CameraBoardSocket::CAM_B, forwardGuess));

    Extrinsics reverseGuess({{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}, {-0.25f, 0.0f, 0.0f}, CameraBoardSocket::CAM_A, LengthUnit::METER);
    reverseGuess.toDeviceId = "device-a";
    REQUIRE_THROWS_AS(node->setInitialGuess("device-b", CameraBoardSocket::CAM_B, "device-a", CameraBoardSocket::CAM_A, reverseGuess), std::runtime_error);
}
#endif
