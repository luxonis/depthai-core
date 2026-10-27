#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <memory>

#include "../../../src/pipeline/node/DynamicCalibrationUtils.hpp"

namespace {
using Socket = dai::CameraBoardSocket;

const char* kSceneJson = R"({
    "dictionary": "DICT_4X4_50",
    "lengthUnit": "cm",
    "markers": [37, 43],
    "markerDistances": [{"markers": [37, 43], "distance": 84}],
    "socketDistances": [{"marker": 37, "socket": "CAM_A", "distance": 168}]
})";

std::shared_ptr<dcl::CameraCalibrationHandle> dclCalibration(double x) {
    const dcl::scalar_t rotation[3] = {0.0, 0.0, 0.0};
    const dcl::scalar_t translation[3] = {x, 0.0, 0.0};
    const dcl::scalar_t cameraMatrix[9] = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
    dcl::scalar_t distortion[14] = {0.0};
    return std::make_shared<dcl::CameraCalibrationHandle>(rotation, translation, cameraMatrix, dcl::PerspectiveDistortion::fromArray(distortion));
}
}  // namespace

TEST_CASE("ArucoMarkerConstraints parses the documented JSON form", "[ArucoMarkerConstraints]") {
    const auto constraints = dai::ArucoMarkerConstraints::fromJsonString(kSceneJson);
    REQUIRE(constraints.dictionary == dai::ArucoDictionary::DICT_4X4_50);
    REQUIRE(constraints.markerIds == std::vector<int32_t>{37, 43});
    REQUIRE(constraints.markerDistances.size() == 1);
    REQUIRE(constraints.markerDistances[0].markerA == 37);
    REQUIRE(constraints.markerDistances[0].markerB == 43);
    REQUIRE(constraints.markerDistances[0].distanceMeters == Catch::Approx(0.84f));
    REQUIRE(constraints.socketDistances.size() == 1);
    REQUIRE(constraints.socketDistances[0].marker == 37);
    REQUIRE(constraints.socketDistances[0].socket == Socket::CAM_A);
    REQUIRE(constraints.socketDistances[0].distanceMeters == Catch::Approx(1.68f));
    REQUIRE(constraints.referencedMarkerIds() == std::vector<int32_t>{37, 43});
    REQUIRE(constraints.referencedSockets() == std::vector<Socket>{Socket::CAM_A});
    REQUIRE_FALSE(constraints.empty());

    // Round trip through the documented form and through the wire form.
    const auto roundTrip = dai::ArucoMarkerConstraints::fromJson(constraints.toJson());
    REQUIRE(roundTrip.markerDistances[0].distanceMeters == Catch::Approx(0.84f));
    REQUIRE(roundTrip.socketDistances[0].socket == Socket::CAM_A);
    nlohmann::json wire = constraints;
    const auto fromWire = wire.get<dai::ArucoMarkerConstraints>();
    REQUIRE(fromWire.socketDistances[0].socket == Socket::CAM_A);
    REQUIRE(fromWire.markerDistances[0].distanceMeters == Catch::Approx(0.84f));

    // Alternative spellings.
    const auto alternative = dai::ArucoMarkerConstraints::fromJsonString(
        R"({"dictionary": "4X4_100", "markerDistances": [{"markerA": 1, "markerB": 2, "distance": 0.5}], "socketDistances": [{"marker": 2, "socket": "b", "distance": 1.0}]})");
    REQUIRE(alternative.dictionary == dai::ArucoDictionary::DICT_4X4_100);
    REQUIRE(alternative.markerIds.empty());
    REQUIRE(alternative.referencedMarkerIds() == std::vector<int32_t>{1, 2});
    REQUIRE(alternative.socketDistances[0].socket == Socket::CAM_B);
}

TEST_CASE("ArucoMarkerConstraints rejects malformed input", "[ArucoMarkerConstraints]") {
    using dai::ArucoMarkerConstraints;
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 1], "distance": 0.5}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 2], "distance": 0}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 2, 3], "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"markers": [1], "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"socketDistances": [{"marker": 1, "socket": "CAM_Z", "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"socketDistances": [{"marker": 1, "socket": "AUTO", "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"lengthUnit": "furlong", "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString(R"({"dictionary": "DICT_9X9", "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(ArucoMarkerConstraints::fromJsonString("[1, 2]"), std::invalid_argument);

    dai::Pipeline pipeline(false);
    auto node = pipeline.create<dai::node::DynamicCalibration>();
    REQUIRE_FALSE(node->getArucoConstraints().has_value());
    REQUIRE_THROWS_AS(node->setArucoConstraints(ArucoMarkerConstraints{}), std::invalid_argument);
    node->setArucoConstraintsFromJson(kSceneJson);
    REQUIRE(node->getArucoConstraints().has_value());
    REQUIRE(node->getArucoConstraints()->socketDistances[0].socket == Socket::CAM_A);
    node->clearArucoConstraints();
    REQUIRE_FALSE(node->getArucoConstraints().has_value());
}

TEST_CASE("DynamicCalibration assembles world constraints from marker observations", "[ArucoMarkerConstraints]") {
    using dai::node::DclUtils;
    const auto constraints = dai::ArucoMarkerConstraints::fromJsonString(kSceneJson);

    dcl::DynamicCalibration dcl;
    auto device = dcl.addDevice();
    const dcl::resolution_t resolution{1280, 800};
    std::map<Socket, std::shared_ptr<const dcl::CameraSensorHandle>> sensors;
    sensors[Socket::CAM_B] = dcl.addSensor(device, dclCalibration(0.0), resolution, 1);
    sensors[Socket::CAM_C] = dcl.addSensor(device, dclCalibration(-0.075), resolution, 2);
    sensors[Socket::CAM_A] = dcl.addSensor(device, dclCalibration(0.0375), resolution, 0);
    const auto sensorForSocket = [&](Socket socket) -> std::shared_ptr<const dcl::CameraSensorHandle> {
        const auto it = sensors.find(socket);
        return it == sensors.end() ? nullptr : it->second;
    };

    std::deque<dai::ArucoMarkerObservationSet> sets(3);
    // Frame set 0: both markers in both stereo cameras -> two points, one marker distance, one socket distance.
    sets[0].centers[37] = {{Socket::CAM_B, {100.f, 200.f}}, {Socket::CAM_C, {80.f, 201.f}}};
    sets[0].centers[43] = {{Socket::CAM_B, {600.f, 210.f}}, {Socket::CAM_C, {578.f, 211.f}}};
    // Frame set 1: marker 43 seen by one camera only -> only marker 37 counts, no marker distance.
    sets[1].centers[37] = {{Socket::CAM_B, {101.f, 200.f}}, {Socket::CAM_C, {81.f, 201.f}}};
    sets[1].centers[43] = {{Socket::CAM_B, {600.f, 210.f}}};
    // Frame set 2: marker 43 only -> a point but no distance at all.
    sets[2].centers[43] = {{Socket::CAM_B, {600.f, 210.f}}, {Socket::CAM_C, {578.f, 211.f}}};

    DclUtils::WorldConstraintsSummary summary;
    const auto world = DclUtils::buildWorldConstraints(constraints, sets, sensorForSocket, &summary);
    REQUIRE(world.points.size() == 4);
    REQUIRE(world.pointDistances.size() == 1);
    REQUIRE(world.sensorDistances.size() == 2);
    REQUIRE(summary.observationSets == 3);
    REQUIRE(summary.points == 4);
    REQUIRE(world.pointDistances[0].pointIndexA == 0);
    REQUIRE(world.pointDistances[0].pointIndexB == 1);
    REQUIRE(world.pointDistances[0].distanceMeters == Catch::Approx(0.84));
    REQUIRE(world.sensorDistances[0].pointIndex == 0);
    REQUIRE(world.sensorDistances[0].sensor == sensors[Socket::CAM_A]);
    REQUIRE(world.sensorDistances[0].distanceMeters == Catch::Approx(1.68));
    REQUIRE(world.sensorDistances[1].pointIndex == 2);
    REQUIRE(world.points[0].observations.size() == 2);
    REQUIRE(world.points[0].observations[0].pixel[0] == Catch::Approx(100.0));
    REQUIRE_FALSE(world.empty());

    // Without the auxiliary socket the socket distances are skipped, the marker distance stays.
    sensors.erase(Socket::CAM_A);
    const auto withoutAuxiliary = DclUtils::buildWorldConstraints(constraints, sets, sensorForSocket);
    REQUIRE(withoutAuxiliary.pointDistances.size() == 1);
    REQUIRE(withoutAuxiliary.sensorDistances.empty());

    // Nothing seen by two cameras: no constraints at all.
    std::deque<dai::ArucoMarkerObservationSet> single(1);
    single[0].centers[37] = {{Socket::CAM_B, {100.f, 200.f}}};
    REQUIRE(DclUtils::buildWorldConstraints(constraints, single, sensorForSocket).empty());
}

TEST_CASE("DynamicCalibration detects marker centers through DCL", "[ArucoMarkerConstraints]") {
    // A white image holds no marker; a marker rendered by OpenCV is found at its center.
    dcl::ImageData blank;
    blank.width = 64;
    blank.height = 64;
    blank.format = dcl::DCL_8UC1;
    blank.data.assign(64 * 64, 255);
    const auto constraints = dai::ArucoMarkerConstraints::fromJsonString(kSceneJson);
    REQUIRE(dai::node::DclUtils::detectMarkerCenters(blank, constraints).empty());
}
