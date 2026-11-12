#include <array>
#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <map>
#include <memory>

#include "../../../src/pipeline/node/DynamicCalibrationUtils.hpp"

// After the depthai headers: the DCL headers define macros that collide with depthai enumerators.
#include <DynamicCalibration.hpp>

namespace {
using Socket = dai::CameraBoardSocket;

const char* kWorldJson = R"({
    "markerType": "aruco",
    "arucoDictionary": "DICT_4X4_50",
    "lengthUnit": "cm",
    "markers": [37, 43],
    "markerDistances": [{"markers": [37, 43], "distance": 84}],
    "socketDistances": [{"marker": 37, "socket": "CAM_A", "distance": 168}],
    "markerSizes": [{"marker": 43, "size": 15}]
})";

std::shared_ptr<dcl::CameraCalibrationHandle> dclCalibration(double x) {
    const dcl::scalar_t rotation[3] = {0.0, 0.0, 0.0};
    const dcl::scalar_t translation[3] = {x, 0.0, 0.0};
    const dcl::scalar_t cameraMatrix[9] = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
    dcl::scalar_t distortion[14] = {0.0};
    return std::make_shared<dcl::CameraCalibrationHandle>(rotation, translation, cameraMatrix, dcl::PerspectiveDistortion::fromArray(distortion));
}
}  // namespace

TEST_CASE("WorldConstraints parses the documented JSON form", "[WorldConstraints]") {
    const auto constraints = dai::WorldConstraints::fromJsonString(kWorldJson);
    REQUIRE(constraints.markerType == dai::MarkerType::ARUCO);
    REQUIRE(constraints.arucoDictionary == dai::ArucoDictionary::DICT_4X4_50);
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
    const auto roundTrip = dai::WorldConstraints::fromJson(constraints.toJson());
    REQUIRE(roundTrip.markerDistances[0].distanceMeters == Catch::Approx(0.84f));
    REQUIRE(roundTrip.markerType == dai::MarkerType::ARUCO);
    REQUIRE(constraints.toJson().at("markerType") == "ARUCO");
    REQUIRE(roundTrip.socketDistances[0].socket == Socket::CAM_A);
    nlohmann::json wire = constraints;
    const auto fromWire = wire.get<dai::WorldConstraints>();
    REQUIRE(fromWire.socketDistances[0].socket == Socket::CAM_A);
    REQUIRE(fromWire.markerDistances[0].distanceMeters == Catch::Approx(0.84f));

    REQUIRE(constraints.markerSizes.size() == 1);
    REQUIRE(constraints.markerSizes[0].marker == 43);
    REQUIRE(constraints.markerSizes[0].sizeMeters == Catch::Approx(0.15f));
    REQUIRE(roundTrip.markerSizes[0].sizeMeters == Catch::Approx(0.15f));
    REQUIRE(fromWire.markerSizes[0].marker == 43);

    // Alternative spellings.
    const auto alternative = dai::WorldConstraints::fromJsonString(
        R"({"dictionary": "4X4_100", "markerDistances": [{"markerA": 1, "markerB": 2, "distance": 0.5}], "socketDistances": [{"marker": 2, "socket": "b", "distance": 1.0}]})");
    REQUIRE(alternative.markerType == dai::MarkerType::ARUCO);
    REQUIRE(alternative.arucoDictionary == dai::ArucoDictionary::DICT_4X4_100);
    REQUIRE(alternative.markerIds.empty());
    REQUIRE(alternative.referencedMarkerIds() == std::vector<int32_t>{1, 2});
    REQUIRE(alternative.socketDistances[0].socket == Socket::CAM_B);

    // Dictionary names match regardless of case, including the AprilTag names with their lowercase "h", and
    // the name toJson() writes parses back.
    for(const char* spelling : {"DICT_APRILTAG_36h11", "DICT_APRILTAG_36H11", "dict_apriltag_36h11", "apriltag_36h11"}) {
        REQUIRE(dai::arucoDictionaryFromString(spelling) == dai::ArucoDictionary::DICT_APRILTAG_36h11);
    }
    for(const auto dictionary : {dai::ArucoDictionary::DICT_APRILTAG_16h5,
                                 dai::ArucoDictionary::DICT_APRILTAG_25h9,
                                 dai::ArucoDictionary::DICT_APRILTAG_36h10,
                                 dai::ArucoDictionary::DICT_APRILTAG_36h11,
                                 dai::ArucoDictionary::DICT_ARUCO_ORIGINAL,
                                 dai::ArucoDictionary::DICT_7X7_1000}) {
        auto apriltag = constraints;
        apriltag.arucoDictionary = dictionary;
        REQUIRE(dai::arucoDictionaryFromString(dai::toString(dictionary)) == dictionary);
        REQUIRE(dai::WorldConstraints::fromJson(apriltag.toJson()).arucoDictionary == dictionary);
        REQUIRE(dai::WorldConstraints::fromJsonString(apriltag.toJson().dump()).arucoDictionary == dictionary);
    }
}

TEST_CASE("WorldConstraints rejects malformed input", "[WorldConstraints]") {
    using dai::WorldConstraints;
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 1], "distance": 0.5}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 2], "distance": 0}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 2, 3], "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markers": [1], "markerDistances": [{"markers": [1, 2], "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"socketDistances": [{"marker": 1, "socket": "CAM_Z", "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"socketDistances": [{"marker": 1, "socket": "AUTO", "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"lengthUnit": "furlong", "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerType": "cctag", "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    // Luxonis Markers are a known type: parsed by name (any case) or number, round-tripped without a dictionary.
    const auto luxonis = WorldConstraints::fromJsonString(R"({"markerType": "luxonis", "markerDistances": [{"markers": [0, 3], "distance": 0.5}]})");
    REQUIRE(luxonis.markerType == dai::MarkerType::LUXONIS);
    REQUIRE(luxonis.toJson().at("markerType") == "LUXONIS");
    REQUIRE_FALSE(luxonis.toJson().contains("arucoDictionary"));
    REQUIRE(WorldConstraints::fromJson(luxonis.toJson()).markerType == dai::MarkerType::LUXONIS);
    REQUIRE(WorldConstraints::fromJsonString(R"({"markerType": 1})").markerType == dai::MarkerType::LUXONIS);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"dictionary": "DICT_9X9", "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString("[1, 2]"), std::invalid_argument);
    // Closed validation holes: socket out of range, negative or fractional marker ids, malformed JSON and missing keys
    // all surface as std::invalid_argument, never as a nlohmann exception.
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"socketDistances": [{"marker": 1, "socket": 99, "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerDistances": [{"markers": [-3, 2], "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markers": [1.5], "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerDistances": [{"markerA": 1, "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerDistances": [{"markers": [1, 2], "distance": "far"}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString("{not json"), std::invalid_argument);
    // Enumerators given as numbers are range-checked, as is the observation window.
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"arucoDictionary": 99, "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"markerType": 7, "markerDistances": [{"markers": [1, 2], "distance": 1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"maxObservationSets": 0, "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"maxObservationSets": -1, "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE_THROWS_AS(WorldConstraints::fromJsonString(R"({"maxObservationSets": 2.5, "markerDistances": [{"markers": [1, 2], "distance": 1}]})"),
                      std::invalid_argument);
    REQUIRE(WorldConstraints::fromJsonString(R"({"maxObservationSets": 3, "arucoDictionary": 4, "markerDistances": [{"markers": [1, 2], "distance": 1}]})")
                .maxObservationSets
            == 3);
    {
        WorldConstraints negative;
        negative.socketDistances.push_back({-1, Socket::CAM_A, 1.0f});
        REQUIRE_THROWS_AS(negative.validate(), std::invalid_argument);
        WorldConstraints unknownSocket;
        unknownSocket.socketDistances.push_back({1, static_cast<Socket>(99), 1.0f});
        REQUIRE_THROWS_AS(unknownSocket.validate(), std::invalid_argument);
        WorldConstraints unknownDictionary = WorldConstraints::fromJsonString(kWorldJson);
        unknownDictionary.arucoDictionary = static_cast<dai::ArucoDictionary>(99);
        REQUIRE_THROWS_AS(unknownDictionary.validate(), std::invalid_argument);
        WorldConstraints noWindow = WorldConstraints::fromJsonString(kWorldJson);
        noWindow.maxObservationSets = 0;
        REQUIRE_THROWS_AS(noWindow.validate(), std::invalid_argument);
    }

    dai::Pipeline pipeline(false);
    auto node = pipeline.create<dai::node::DynamicCalibration>();
    REQUIRE_FALSE(node->getWorldConstraints().has_value());
    REQUIRE_THROWS_AS(node->setWorldConstraints(WorldConstraints{}), std::invalid_argument);
    node->setWorldConstraintsFromJson(kWorldJson);
    REQUIRE(node->getWorldConstraints().has_value());
    REQUIRE(node->getWorldConstraints()->socketDistances[0].socket == Socket::CAM_A);
    node->clearWorldConstraints();
    REQUIRE_FALSE(node->getWorldConstraints().has_value());
}

TEST_CASE("Pipeline keeps world constraints for the implicit AutoCalibration node", "[WorldConstraints]") {
    dai::Pipeline pipeline(false);
    REQUIRE_FALSE(pipeline.getAutoCalibrationWorldConstraints().has_value());
    REQUIRE_THROWS_AS(pipeline.setAutoCalibrationWorldConstraints(dai::WorldConstraints{}), std::invalid_argument);
    pipeline.setAutoCalibrationWorldConstraintsFromJson(kWorldJson);
    REQUIRE(pipeline.getAutoCalibrationWorldConstraints().has_value());
    REQUIRE(pipeline.getAutoCalibrationWorldConstraints()->socketDistances[0].socket == Socket::CAM_A);
    pipeline.clearAutoCalibrationWorldConstraints();
    REQUIRE_FALSE(pipeline.getAutoCalibrationWorldConstraints().has_value());
}

TEST_CASE("DynamicCalibration assembles world constraints from marker observations", "[WorldConstraints]") {
    using dai::node::DclUtils;
    const auto constraints = dai::WorldConstraints::fromJsonString(kWorldJson);

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

    std::deque<dai::MarkerObservationSet> sets(3);
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

    // A socket distance to a camera that is itself an image input resolves to that connected sensor.
    auto toImageInput = constraints;
    toImageInput.socketDistances = {{37, Socket::CAM_C, 1.2f}};
    const auto withImageInput = DclUtils::buildWorldConstraints(toImageInput, sets, sensorForSocket);
    REQUIRE(withImageInput.sensorDistances.size() == 2);
    REQUIRE(withImageInput.sensorDistances[0].sensor == sensors[Socket::CAM_C]);
    REQUIRE(withImageInput.sensorDistances[0].distanceMeters == Catch::Approx(1.2));

    // A marker size adds one observation per sensor that saw the marker of a scene point.
    auto sized = constraints;
    sized.markerDistances.clear();
    sized.socketDistances.clear();
    sized.markerSizes = {{43, 0.2f}};
    auto outlined = sets;
    dai::MarkerOutline outline;
    outline.corners = {{{1.f, 2.f}}, {{3.f, 2.f}}, {{3.f, 4.f}}, {{1.f, 4.f}}};
    outlined[0].outlines[43] = {{Socket::CAM_B, outline}, {Socket::CAM_C, outline}};
    outlined[1].outlines[43] = {{Socket::CAM_B, outline}};  // this marker is no scene point in frame set 1
    DclUtils::WorldConstraintsSummary sizedSummary;
    const auto withSizes = DclUtils::buildWorldConstraints(sized, outlined, sensorForSocket, &sizedSummary);
    REQUIRE(sizedSummary.markerSizes == 2);
    REQUIRE(withSizes.markerSizes.size() == 2);
    REQUIRE(withSizes.markerSizes[0].pointIndex == 1);
    REQUIRE(withSizes.markerSizes[0].sensor == sensors[Socket::CAM_B]);
    REQUIRE(withSizes.markerSizes[1].sensor == sensors[Socket::CAM_C]);
    REQUIRE(withSizes.markerSizes[0].type == dcl::MarkerType::ARUCO);
    REQUIRE(withSizes.markerSizes[0].sizeMeters == Catch::Approx(0.2));
    REQUIRE(withSizes.markerSizes[0].detection.center[0] == Catch::Approx(600.0));
    REQUIRE(withSizes.markerSizes[0].detection.corners.size() == 4);
    REQUIRE(withSizes.markerSizes[0].detection.corners[2][1] == Catch::Approx(4.0));
    REQUIRE_FALSE(withSizes.empty());
    // Without outlines there is nothing to measure the size against.
    REQUIRE(DclUtils::buildWorldConstraints(sized, sets, sensorForSocket).markerSizes.empty());

    // Nothing seen by two cameras: no constraints at all.
    std::deque<dai::MarkerObservationSet> single(1);
    single[0].centers[37] = {{Socket::CAM_B, {100.f, 200.f}}};
    REQUIRE(DclUtils::buildWorldConstraints(constraints, single, sensorForSocket).empty());
}

TEST_CASE("WorldConstraints validates marker sizes", "[WorldConstraints]") {
    using Constraints = dai::WorldConstraints;
    const auto sizesOnly = Constraints::fromJsonString(R"({"lengthUnit": "mm", "markerSizes": [{"marker": 3, "size": 100}]})");
    REQUIRE_FALSE(sizesOnly.empty());
    REQUIRE(sizesOnly.markerSizes[0].sizeMeters == Catch::Approx(0.1f));
    REQUIRE(sizesOnly.referencedMarkerIds() == std::vector<int32_t>{3});
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markerSizes": [{"marker": 3, "size": 0}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markerSizes": [{"marker": 3, "size": -0.1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markerSizes": [{"marker": 3}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markerSizes": [{"size": 0.1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markerSizes": [{"marker": -1, "size": 0.1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markers": [1], "markerSizes": [{"marker": 3, "size": 0.1}]})"), std::invalid_argument);
    REQUIRE_THROWS_AS(Constraints::fromJsonString(R"({"markerSizes": [{"marker": 3, "size": 0.1}, {"marker": 3, "size": 0.2}]})"), std::invalid_argument);
}

namespace {
// DICT_4X4_50 marker 37 with its one-cell black border, one character per cell ('1' = white).
constexpr std::array<const char*, 6> kMarker37 = {"000000", "000000", "001000", "011110", "011110", "000000"};

dcl::ImageData whiteImage(int width, int height) {
    dcl::ImageData image;
    image.width = width;
    image.height = height;
    image.format = dcl::DCL_8UC1;
    image.data.assign(static_cast<size_t>(width) * height, 255);
    return image;
}

/// Draws kMarker37 with `cell` pixels per cell, top-left corner at (x, y).
void drawMarker37(dcl::ImageData& image, int x, int y, int cell) {
    for(int row = 0; row < 6; ++row) {
        for(int col = 0; col < 6; ++col) {
            const uint8_t value = kMarker37[row][col] == '1' ? 255 : 0;
            for(int dy = 0; dy < cell; ++dy) {
                for(int dx = 0; dx < cell; ++dx) {
                    image.data[static_cast<size_t>(y + row * cell + dy) * image.width + (x + col * cell + dx)] = value;
                }
            }
        }
    }
}

/// Reference points of the configured markers found in one image, keyed by marker id.
std::map<int32_t, std::array<float, 2>> markerCenters(const dcl::ImageData& image, const dai::WorldConstraints& constraints) {
    auto detected = dai::node::DclUtils::detectConfiguredMarkers(image, constraints);
    REQUIRE(detected.passed());
    std::map<int32_t, std::array<float, 2>> centers;
    for(const auto& [id, detection] : detected.value) {
        centers[id] = {static_cast<float>(detection.center[0]), static_cast<float>(detection.center[1])};
    }
    return centers;
}
}  // namespace

TEST_CASE("DynamicCalibration detects marker centers through DCL", "[WorldConstraints]") {
    using dai::node::DclUtils;
    const auto constraints = dai::WorldConstraints::fromJsonString(kWorldJson);

    // A white image holds no marker.
    REQUIRE(markerCenters(whiteImage(64, 64), constraints).empty());

    // Marker 37 drawn at 16 px per cell with its top-left corner at (40, 40): the center is the middle of the 96 px
    // square, i.e. 40 + 48 - 0.5 in pixel-center coordinates.
    auto image = whiteImage(320, 240);
    drawMarker37(image, 40, 40, 16);
    const auto centers = markerCenters(image, constraints);
    REQUIRE(centers.size() == 1);
    REQUIRE(centers.count(37) == 1);
    REQUIRE(centers.at(37)[0] == Catch::Approx(87.5).margin(0.75));
    REQUIRE(centers.at(37)[1] == Catch::Approx(87.5).margin(0.75));

    // A marker smaller than DclUtils::kMinMarkerSizePx (3 px per cell, 18 px side) is dropped; one at 6 px per cell
    // (36 px side) is used. Each is drawn alone so that it is the only copy of id 37 in the image. (Sub-pixel corners
    // sit on pixel centers, so a square printed exactly 25 px wide measures about 24 px and is not a usable boundary.)
    auto small = whiteImage(320, 240);
    drawMarker37(small, 40, 40, 3);
    REQUIRE(markerCenters(small, constraints).empty());
    std::map<int32_t, double> tooSmall;
    REQUIRE(dai::node::DclUtils::detectConfiguredMarkers(small, constraints, &tooSmall).value.empty());
    REQUIRE(tooSmall.size() == 1);
    REQUIRE(tooSmall.at(37) > 0.0);
    REQUIRE(tooSmall.at(37) < dai::node::DclUtils::kMinMarkerSizePx);
    auto bigEnough = whiteImage(320, 240);
    drawMarker37(bigEnough, 40, 40, 6);
    REQUIRE(markerCenters(bigEnough, constraints).count(37) == 1);

    // A marker id that is not part of the constraints is dropped.
    auto others = constraints;
    others.markerIds = {43};
    others.markerDistances.clear();
    others.socketDistances = {{43, Socket::CAM_A, 1.0f}};
    REQUIRE(markerCenters(image, others).empty());

    // The same id twice in one image is ambiguous: neither copy is used.
    drawMarker37(image, 200, 120, 16);
    REQUIRE(markerCenters(image, constraints).empty());
}

TEST_CASE("DynamicCalibration detects Luxonis Marker centers through DCL", "[WorldConstraints]") {
    using dai::node::DclUtils;
    auto constraints = dai::WorldConstraints::fromJsonString(
        R"({"markerType": "luxonis", "markerDistances": [{"markers": [0, 4], "distance": 0.6}], "socketDistances": [{"marker": 2, "socket": "CAM_A", "distance": 1.5}]})");

    // Three Luxonis Markers (ids 0, 2, 4; outer radius 40 px, 128 px tiles) pasted on a 640x480 grey canvas; the
    // raster centre is the middle of the square tile, i.e. x0 + n / 2 - 0.5 in pixel-center coordinates.
    auto image = whiteImage(640, 480);
    std::fill(image.data.begin(), image.data.end(), static_cast<uint8_t>(190));
    const std::array<std::pair<int, std::pair<int, int>>, 3> placements = {{{0, {30, 40}}, {2, {250, 60}}, {4, {400, 250}}}};
    std::map<int, std::array<float, 2>> truth;
    for(const auto& [id, origin] : placements) {
        const auto tile = dcl::drawLuxonisMarker(id, 40.0);
        for(unsigned y = 0; y < tile.height; ++y) {
            for(unsigned x = 0; x < tile.width; ++x) {
                image.data[(origin.second + y) * image.width + origin.first + x] = tile.data[y * tile.width + x];
            }
        }
        truth[id] = {origin.first + tile.width / 2.0f - 0.5f, origin.second + tile.height / 2.0f - 0.5f};
    }
    const auto centers = markerCenters(image, constraints);
    REQUIRE(centers.size() == 3);
    const auto outline = DclUtils::toMarkerOutline(DclUtils::detectConfiguredMarkers(image, constraints).value.at(0));
    REQUIRE(outline.corners.size() == 4);
    REQUIRE_FALSE(outline.outerEdge.empty());
    REQUIRE_FALSE(outline.innerEdge.empty());
    for(const auto& [id, uv] : truth) {
        REQUIRE(centers.count(id) == 1);
        REQUIRE(centers.at(id)[0] == Catch::Approx(uv[0]).margin(0.3));
        REQUIRE(centers.at(id)[1] == Catch::Approx(uv[1]).margin(0.3));
    }
    // A ring smaller than DclUtils::kMinMarkerSizePx (outer radius 10 px, 20 px diameter) is dropped.
    {
        auto small = whiteImage(640, 480);
        std::fill(small.data.begin(), small.data.end(), static_cast<uint8_t>(190));
        const auto tile = dcl::drawLuxonisMarker(0, 10.0);
        for(unsigned y = 0; y < tile.height; ++y) {
            for(unsigned x = 0; x < tile.width; ++x) {
                small.data[(100 + y) * small.width + 100 + x] = tile.data[y * tile.width + x];
            }
        }
        REQUIRE(markerCenters(small, constraints).empty());
    }
    // Ids outside the constraints are dropped, and an ArUco spec sees no Luxonis Marker.
    constraints.markerIds = {0};
    constraints.socketDistances.clear();
    constraints.markerDistances = {{0, 4, 0.6f}};
    REQUIRE(markerCenters(image, constraints).size() == 2);
    auto aruco = dai::WorldConstraints::fromJsonString(R"({"markerDistances": [{"markers": [0, 4], "distance": 0.6}]})");
    REQUIRE(markerCenters(image, aruco).empty());
}
