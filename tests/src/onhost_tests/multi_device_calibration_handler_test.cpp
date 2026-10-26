#include <catch2/catch_all.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/properties/GlobalProperties.hpp>
#include <depthai/utility/Serialization.hpp>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "depthai/beta/device/LocalCalibrationOrigin.hpp"
#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/device/CalibrationHandler.hpp"

using namespace dai;
using dai::beta::MultiDeviceCalibrationHandler;

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

    const MultiDeviceCalibrationHandler handler(std::vector<MultiDeviceExtrinsics>{edge});

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
    first.extrinsics.specTranslation = {50.0f, 0.0f, 0.0f};

    MultiDeviceExtrinsics second;
    second.fromDeviceId = "device-b";
    second.fromSocket = CameraBoardSocket::CAM_A;
    second.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {2.0f, 0.0f, 0.0f}, LengthUnit::METER);

    MultiDeviceExtrinsics separate;
    separate.fromDeviceId = "device-z";
    separate.fromSocket = CameraBoardSocket::CAM_B;
    separate.extrinsics = makeExtrinsics("device-y", CameraBoardSocket::CAM_B, {1.0f, 0.0f, 0.0f}, LengthUnit::METER);

    const MultiDeviceCalibrationHandler handler(std::vector<MultiDeviceExtrinsics>{first, second, separate});

    const auto chainBridge = handler.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(chainBridge.has_value());
    REQUIRE(chainBridge->toDeviceId == "device-a");
    REQUIRE(chainBridge->translation.x == Catch::Approx(3.0f));
    REQUIRE(chainBridge->lengthUnit == LengthUnit::METER);

    const auto separateBridge = handler.getExtrinsicsToOrigin("device-z", CameraBoardSocket::CAM_B);
    REQUIRE(separateBridge.has_value());
    REQUIRE(separateBridge->toDeviceId == "device-y");
    REQUIRE(separateBridge->translation.x == Catch::Approx(1.0f));

    const auto serialized = utility::serialize<SerializationType::JSON>(handler);
    MultiDeviceCalibrationHandler roundTrip;
    REQUIRE(utility::deserialize<SerializationType::JSON>(serialized, roundTrip));
    const auto roundTripBridge = roundTrip.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(roundTripBridge.has_value());
    REQUIRE(roundTripBridge->toDeviceId == "device-a");
    REQUIRE(roundTripBridge->translation.x == Catch::Approx(3.0f));

    const auto json = handler.toJson();
    REQUIRE(json.contains("graph"));
    for(const auto& jsonEdge : json.at("graph")) {
        REQUIRE(jsonEdge.at("extrinsics").at("lengthUnit") == static_cast<int32_t>(LengthUnit::CENTIMETER));
    }
    REQUIRE(json.at("graph").at(0).at("extrinsics").at("translation").at("x") == Catch::Approx(200.0f));
    REQUIRE(json.at("graph").at(1).at("extrinsics").at("translation").at("x") == Catch::Approx(100.0f));
    REQUIRE(json.at("graph").at(1).at("extrinsics").at("specTranslation").at("x") == Catch::Approx(50.0f));
    const auto jsonRoundTrip = MultiDeviceCalibrationHandler::fromJson(json);
    const auto jsonBridge = jsonRoundTrip.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(jsonBridge.has_value());
    REQUIRE(jsonBridge->translation.x == Catch::Approx(3.0f));
    REQUIRE(jsonRoundTrip.toJson().at("graph").at(1).at("extrinsics").at("specTranslation").at("x") == Catch::Approx(50.0f));

    const auto jsonPath = std::filesystem::temp_directory_path() / "depthai_multi_device_calibration_handler_test.json";
    REQUIRE(handler.toJsonFile(jsonPath));
    const MultiDeviceCalibrationHandler fileRoundTrip(jsonPath);
    const auto fileBridge = fileRoundTrip.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(fileBridge.has_value());
    REQUIRE(fileBridge->translation.x == Catch::Approx(3.0f));
    REQUIRE(std::filesystem::remove(jsonPath));

    // The pipeline carries only the plain edge list; the handler is rebuilt from it.
    GlobalProperties properties;
    properties.multiDeviceCalibration = handler.getGraph();
    for(const auto serializationType : {SerializationType::LIBNOP, SerializationType::JSON}) {
        const auto propertiesData = utility::serialize(properties, serializationType);
        GlobalProperties propertiesRoundTrip;
        REQUIRE(utility::deserialize(propertiesData, propertiesRoundTrip, serializationType));
        REQUIRE(propertiesRoundTrip.multiDeviceCalibration.has_value());
        const MultiDeviceCalibrationHandler propertiesHandler(*propertiesRoundTrip.multiDeviceCalibration);
        const auto propertiesBridge = propertiesHandler.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
        REQUIRE(propertiesBridge.has_value());
        REQUIRE(propertiesBridge->translation.x == Catch::Approx(3.0f));
    }

    Pipeline pipeline(false);
    REQUIRE_FALSE(pipeline.getMultiDeviceCalibration().has_value());
    pipeline.setMultiDeviceCalibration(handler.getGraph());
    REQUIRE(pipeline.getGlobalProperties().multiDeviceCalibration.has_value());
    REQUIRE(pipeline.getGlobalProperties().multiDeviceCalibration->size() == 3);
    const auto pipelineGraph = pipeline.getMultiDeviceCalibration();
    REQUIRE(pipelineGraph.has_value());
    const MultiDeviceCalibrationHandler pipelineHandler(*pipelineGraph);
    const auto pipelineBridge = pipelineHandler.getExtrinsicsToOrigin("device-c", CameraBoardSocket::CAM_A);
    REQUIRE(pipelineBridge.has_value());
    REQUIRE(pipelineBridge->toDeviceId == "device-a");
    REQUIRE(pipelineBridge->translation.x == Catch::Approx(3.0f));
    pipeline.clearMultiDeviceCalibration();
    REQUIRE_FALSE(pipeline.getMultiDeviceCalibration().has_value());

    // The pipeline validates the graph structure on every set, so an invalid graph never reaches the devices.
    auto invalidEdge = handler.getGraph().front();
    invalidEdge.fromDeviceId.clear();
    REQUIRE_THROWS_AS(pipeline.setMultiDeviceCalibration({invalidEdge}), std::invalid_argument);
    REQUIRE_FALSE(pipeline.getMultiDeviceCalibration().has_value());
    REQUIRE_NOTHROW(pipeline.setMultiDeviceCalibration({}));
    REQUIRE(pipeline.getMultiDeviceCalibration().has_value());
    REQUIRE(pipeline.getMultiDeviceCalibration()->empty());
}

TEST_CASE("Multi-device calibration rejects invalid graph structure") {
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-a";
    edge.fromSocket = CameraBoardSocket::CAM_A;
    edge.extrinsics = makeExtrinsics("device-b", CameraBoardSocket::CAM_A, {});

    REQUIRE_NOTHROW(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{}));

    auto invalidSource = edge;
    invalidSource.fromDeviceId.clear();
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{invalidSource}), std::invalid_argument);

    auto invalidSocket = edge;
    invalidSocket.fromSocket = CameraBoardSocket::AUTO;
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{invalidSocket}), std::invalid_argument);

    auto nonFiniteTranslation = edge;
    nonFiniteTranslation.extrinsics.translation.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{nonFiniteTranslation}), std::invalid_argument);

    auto nonFiniteSpecTranslation = edge;
    nonFiniteSpecTranslation.extrinsics.specTranslation.x = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{nonFiniteSpecTranslation}), std::invalid_argument);

    auto invalidRotation = edge;
    invalidRotation.extrinsics.rotationMatrix[0][0] = 2.0f;
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{invalidRotation}), std::invalid_argument);

    auto sameDevice = edge;
    sameDevice.extrinsics.toDeviceId = "device-a";
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{sameDevice}), std::invalid_argument);

    auto conflictingSocket = edge;
    conflictingSocket.fromDeviceId = "device-b";
    conflictingSocket.fromSocket = CameraBoardSocket::CAM_B;
    conflictingSocket.extrinsics = makeExtrinsics("device-c", CameraBoardSocket::CAM_A, {});
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{edge, conflictingSocket}), std::invalid_argument);

    MultiDeviceExtrinsics second;
    second.fromDeviceId = "device-b";
    second.fromSocket = CameraBoardSocket::CAM_A;
    second.extrinsics = makeExtrinsics("device-c", CameraBoardSocket::CAM_A, {});
    MultiDeviceExtrinsics cycle;
    cycle.fromDeviceId = "device-c";
    cycle.fromSocket = CameraBoardSocket::CAM_A;
    cycle.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {});
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{edge, second, cycle}), std::invalid_argument);

    auto reverseDuplicate = edge;
    reverseDuplicate.fromDeviceId = "device-b";
    reverseDuplicate.fromSocket = CameraBoardSocket::CAM_A;
    reverseDuplicate.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {});
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{edge, reverseDuplicate}), std::invalid_argument);

    const MultiDeviceCalibrationHandler handler(std::vector<MultiDeviceExtrinsics>{edge});
    REQUIRE_THROWS_AS(handler.getExtrinsicsToOrigin("device-a", CameraBoardSocket::CAM_B), std::invalid_argument);
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler::fromJson(nlohmann::json::object()), nlohmann::json::out_of_range);
    REQUIRE_THROWS_AS(MultiDeviceCalibrationHandler(std::filesystem::path("/path/that/does/not/exist.json")), std::runtime_error);
}

TEST_CASE("Extrinsics::withLengthUnit converts both translations and keeps everything else") {
    auto source = makeExtrinsics("device-b", CameraBoardSocket::CAM_C, {1.0f, 2.0f, 3.0f});
    source.lengthUnit = LengthUnit::METER;
    source.specTranslation = Point3f(0.5f, 0.25f, 0.125f);

    const auto centimeters = source.withLengthUnit(LengthUnit::CENTIMETER);
    REQUIRE(centimeters.lengthUnit == LengthUnit::CENTIMETER);
    REQUIRE(centimeters.translation.x == Catch::Approx(100.0f));
    REQUIRE(centimeters.translation.y == Catch::Approx(200.0f));
    REQUIRE(centimeters.translation.z == Catch::Approx(300.0f));
    REQUIRE(centimeters.specTranslation.x == Catch::Approx(50.0f));
    REQUIRE(centimeters.specTranslation.y == Catch::Approx(25.0f));
    REQUIRE(centimeters.specTranslation.z == Catch::Approx(12.5f));
    REQUIRE(centimeters.rotationMatrix == source.rotationMatrix);
    REQUIRE(centimeters.toCameraSocket == source.toCameraSocket);
    REQUIRE(centimeters.toDeviceId == source.toDeviceId);

    const auto roundTrip = centimeters.withLengthUnit(LengthUnit::METER);
    REQUIRE(roundTrip.translation.x == Catch::Approx(source.translation.x));
    REQUIRE(roundTrip.specTranslation.z == Catch::Approx(source.specTranslation.z));
    REQUIRE(source.withLengthUnit(LengthUnit::METER).isEqualExtrinsics(source));
}

TEST_CASE("Multi-device calibration validates a device's local calibration origin against the graph") {
    MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "device-b";
    edge.fromSocket = CameraBoardSocket::CAM_B;
    edge.extrinsics = makeExtrinsics("device-a", CameraBoardSocket::CAM_A, {1.0f, 0.0f, 0.0f});
    const MultiDeviceCalibrationHandler handler(std::vector<MultiDeviceExtrinsics>{edge});

    const auto describe = [](bool available, CameraBoardSocket presentSocket, CameraBoardSocket origin) {
        dai::beta::LocalCalibrationOrigin local;
        local.available = available;
        local.hasSocket = [presentSocket](CameraBoardSocket socket) { return socket == presentSocket; };
        local.originOf = [origin](CameraBoardSocket) { return origin; };
        return local;
    };

    SECTION("a device absent from the graph always passes") {
        dai::beta::LocalCalibrationOrigin none;
        REQUIRE_NOTHROW(handler.validateLocalOrigin("device-c", none));
    }
    SECTION("missing local calibration is rejected") {
        REQUIRE_THROWS_WITH(handler.validateLocalOrigin("device-b", describe(false, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_B)),
                            Catch::Matchers::ContainsSubstring("requires local calibration data for device 'device-b'"));
    }
    SECTION("a local calibration without the graph socket is rejected") {
        REQUIRE_THROWS_WITH(handler.validateLocalOrigin("device-b", describe(true, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_A)),
                            Catch::Matchers::ContainsSubstring("requires local calibration socket CAM_B on device 'device-b'"));
    }
    SECTION("a local origin that differs from the graph socket is rejected") {
        REQUIRE_THROWS_WITH(handler.validateLocalOrigin("device-b", describe(true, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_A)),
                            Catch::Matchers::ContainsSubstring("graph expects socket CAM_B, local calibration resolves to CAM_A"));
    }
    SECTION("a matching local origin passes on both ends of the edge") {
        REQUIRE_NOTHROW(handler.validateLocalOrigin("device-b", describe(true, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_B)));
        REQUIRE_NOTHROW(handler.validateLocalOrigin("device-a", describe(true, CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_A)));
    }
}

TEST_CASE("describeLocalCalibration applies the platform's local origin rule") {
    // Chain CAM_B -> CAM_C -> AUTO on a board without CAM_A: the chain root (CAM_C) is not the
    // lowest-id socket (CAM_B). This is the layout of a two-camera FFC calibrated as a stereo pair.
    EepromData eeprom;
    CameraInfo root;
    root.extrinsics.toCameraSocket = CameraBoardSocket::AUTO;
    CameraInfo child;
    child.extrinsics.toCameraSocket = CameraBoardSocket::CAM_C;
    child.extrinsics.rotationMatrix = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    child.extrinsics.translation = {7.5f, 0.0f, 0.0f};
    eeprom.cameraData[CameraBoardSocket::CAM_C] = root;
    eeprom.cameraData[CameraBoardSocket::CAM_B] = child;

    SECTION("RVC4 resolves the extrinsics chain root") {
        const auto local = dai::beta::describeLocalCalibration(eeprom, dai::Platform::RVC4);
        REQUIRE(local.available);
        REQUIRE(local.hasSocket(CameraBoardSocket::CAM_B));
        REQUIRE(local.hasSocket(CameraBoardSocket::CAM_C));
        REQUIRE_FALSE(local.hasSocket(CameraBoardSocket::CAM_A));
        REQUIRE(local.originOf(CameraBoardSocket::CAM_B) == CameraBoardSocket::CAM_C);
        REQUIRE(local.originOf(CameraBoardSocket::CAM_C) == CameraBoardSocket::CAM_C);
        REQUIRE(std::string(local.originRule) == "extrinsics chain root");
    }
    SECTION("RVC2 resolves the lowest-id socket for every socket") {
        const auto local = dai::beta::describeLocalCalibration(eeprom, dai::Platform::RVC2);
        REQUIRE(local.available);
        REQUIRE(local.originOf(CameraBoardSocket::CAM_B) == CameraBoardSocket::CAM_B);
        REQUIRE(local.originOf(CameraBoardSocket::CAM_C) == CameraBoardSocket::CAM_B);
        REQUIRE(std::string(local.originRule) == "lowest-id socket");
    }
    SECTION("the chain root rule rejects dangling links and cycles") {
        EepromData dangling = eeprom;
        dangling.cameraData[CameraBoardSocket::CAM_C].extrinsics.toCameraSocket = CameraBoardSocket::CAM_D;
        REQUIRE_THROWS_WITH(dai::beta::describeLocalCalibration(dangling, dai::Platform::RVC4).originOf(CameraBoardSocket::CAM_B),
                            Catch::Matchers::ContainsSubstring("Invalid camera link"));
        EepromData cyclic = eeprom;
        cyclic.cameraData[CameraBoardSocket::CAM_C].extrinsics.toCameraSocket = CameraBoardSocket::CAM_B;
        REQUIRE_THROWS_WITH(dai::beta::describeLocalCalibration(cyclic, dai::Platform::RVC4).originOf(CameraBoardSocket::CAM_B),
                            Catch::Matchers::ContainsSubstring("Cyclic extrinsics"));
    }
    SECTION("missing calibration is unavailable on every platform") {
        REQUIRE_FALSE(dai::beta::describeLocalCalibration(std::nullopt, dai::Platform::RVC2).available);
        REQUIRE_FALSE(dai::beta::describeLocalCalibration(std::nullopt, dai::Platform::RVC4).available);
    }

    // The same graph is admitted or rejected per platform, exactly as the firmware will decide.
    const auto graphWith = [](CameraBoardSocket socket) {
        MultiDeviceExtrinsics edge;
        edge.fromDeviceId = "virtual-device";
        edge.fromSocket = CameraBoardSocket::CAM_A;
        edge.extrinsics = makeExtrinsics("this-device", socket, {1.0f, 0.0f, 0.0f});
        return MultiDeviceCalibrationHandler(std::vector<MultiDeviceExtrinsics>{edge});
    };
    SECTION("an RVC2 device accepts the lowest-socket graph and rejects the chain-root graph") {
        const auto local = dai::beta::describeLocalCalibration(eeprom, dai::Platform::RVC2);
        REQUIRE_NOTHROW(graphWith(CameraBoardSocket::CAM_B).validateLocalOrigin("this-device", local));
        REQUIRE_THROWS_WITH(graphWith(CameraBoardSocket::CAM_C).validateLocalOrigin("this-device", local),
                            Catch::Matchers::ContainsSubstring("graph expects socket CAM_C, local calibration resolves to CAM_B (lowest-id socket)"));
    }
    SECTION("an RVC4 device accepts the chain-root graph and rejects the lowest-socket graph") {
        const auto local = dai::beta::describeLocalCalibration(eeprom, dai::Platform::RVC4);
        REQUIRE_NOTHROW(graphWith(CameraBoardSocket::CAM_C).validateLocalOrigin("this-device", local));
        REQUIRE_THROWS_WITH(graphWith(CameraBoardSocket::CAM_B).validateLocalOrigin("this-device", local),
                            Catch::Matchers::ContainsSubstring("graph expects socket CAM_B, local calibration resolves to CAM_C (extrinsics chain root)"));
    }
}
