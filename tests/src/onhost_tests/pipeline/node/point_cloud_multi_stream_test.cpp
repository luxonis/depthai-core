#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/PointCloud.hpp"
#include "depthai/utility/Serialization.hpp"
#include "spdlog/logger.h"

// Host-only pipelines (no device): the PointCloud node and its Sync subnode both run on the host and are
// fed synthetic depth frames through input queues.

namespace {

constexpr unsigned W = 8, H = 6;
constexpr float FX = 100.f, FY = 100.f, CX = 4.f, CY = 3.f;
constexpr uint16_t DEPTH_MM = 1000;
constexpr auto OUTPUT_TIMEOUT = std::chrono::seconds(5);
constexpr auto NO_OUTPUT_WAIT = std::chrono::milliseconds(500);

const std::array<std::array<float, 3>, 3> INTRINSICS = {{{FX, 0.f, CX}, {0.f, FY, CY}, {0.f, 0.f, 1.f}}};

dai::Extrinsics makeExtrinsics(float tx, float ty, float tz, dai::CameraBoardSocket toSocket, const std::string& toDeviceId) {
    dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {tx, ty, tz}, toSocket, dai::LengthUnit::MILLIMETER);
    extrinsics.toDeviceId = toDeviceId;
    return extrinsics;
}

std::shared_ptr<dai::ImgFrame> makeDepthFrame(
    const dai::Extrinsics& extrinsics, std::chrono::steady_clock::time_point timestamp, uint16_t depthMm = DEPTH_MM, unsigned width = W, unsigned height = H) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(width);
    frame->setHeight(height);
    frame->setType(dai::ImgFrame::Type::RAW16);
    std::vector<uint16_t> depth(width * height, depthMm);
    std::vector<uint8_t> bytes(depth.size() * sizeof(uint16_t));
    std::memcpy(bytes.data(), depth.data(), bytes.size());
    frame->setData(std::move(bytes));
    frame->setTransformation(dai::ImgTransformation(width, height, INTRINSICS, dai::CameraModel::Perspective, {}, extrinsics));
    frame->setTimestamp(timestamp);
    return frame;
}

std::shared_ptr<dai::ImgFrame> makeColorFrame(const dai::Extrinsics& extrinsics,
                                              std::chrono::steady_clock::time_point timestamp,
                                              uint8_t r,
                                              uint8_t g,
                                              uint8_t b,
                                              unsigned width = W,
                                              unsigned height = H) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(width);
    frame->setHeight(height);
    frame->setType(dai::ImgFrame::Type::RGB888i);
    std::vector<uint8_t> data(width * height * 3);
    for(unsigned i = 0; i < width * height; ++i) {
        data[i * 3 + 0] = r;
        data[i * 3 + 1] = g;
        data[i * 3 + 2] = b;
    }
    frame->setData(std::move(data));
    frame->setTransformation(dai::ImgTransformation(width, height, INTRINSICS, dai::CameraModel::Perspective, {}, extrinsics));
    frame->setTimestamp(timestamp);
    return frame;
}

/// Expected deprojected point of pixel (col, row) at DEPTH_MM before any extrinsic transform
dai::Point3f expectedLocalPoint(unsigned col, unsigned row, float depthMm = DEPTH_MM) {
    return {(static_cast<float>(col) - CX) / FX * depthMm, (static_cast<float>(row) - CY) / FY * depthMm, depthMm};
}

void requirePointsClose(const std::vector<dai::Point3f>& points, size_t offset, float dx, float dy, float dz, float depthMm = DEPTH_MM) {
    for(unsigned row = 0; row < H; ++row) {
        for(unsigned col = 0; col < W; ++col) {
            const auto expected = expectedLocalPoint(col, row, depthMm);
            const auto& p = points[offset + row * W + col];
            REQUIRE(p.x == Catch::Approx(expected.x + dx).margin(1e-3f));
            REQUIRE(p.y == Catch::Approx(expected.y + dy).margin(1e-3f));
            REQUIRE(p.z == Catch::Approx(expected.z + dz).margin(1e-3f));
        }
    }
}

struct HostPointCloudFixture {
    dai::Pipeline pipeline{false};
    std::shared_ptr<dai::node::PointCloud> pc;

    HostPointCloudFixture() {
        pc = pipeline.create<dai::node::PointCloud>();
        pc->setRunOnHost(true);
        pc->sync->setRunOnHost(true);
        pc->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    }

    ~HostPointCloudFixture() {
        if(pipeline.isRunning()) pipeline.stop();
    }

    std::shared_ptr<dai::PointCloudData> waitForOutput(dai::MessageQueue& queue) {
        bool timedOut = false;
        auto pcd = queue.get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(pcd != nullptr);
        return pcd;
    }
};

}  // namespace

TEST_CASE("PointCloud stream keys", "[PointCloud][MultiStream]") {
    REQUIRE(dai::node::PointCloud::getDepthInputKey("") == "depth");
    REQUIRE(dai::node::PointCloud::getDepthInputKey("left") == "depth/left");
    REQUIRE(dai::node::PointCloud::getColorInputKey("") == "color");
    REQUIRE(dai::node::PointCloud::getColorInputKey("left") == "color/left");
}

TEST_CASE("Named depth inputs are created on the Sync subnode", "[PointCloud][MultiStream]") {
    dai::Pipeline pipeline(false);
    auto pc = pipeline.create<dai::node::PointCloud>();

    REQUIRE(pc->getDepthInputNames() == std::vector<std::string>{""});
    REQUIRE(&pc->getDepthInput("") == &pc->inputDepth);
    REQUIRE(&pc->getColorInput("") == &pc->getColorInput());

    auto& second = pc->getDepthInput("second");
    REQUIRE(&pc->getDepthInput("second") == &second);
    REQUIRE(pc->syncInputs.has("depth/second"));
    REQUIRE(pc->getDepthInputNames() == std::vector<std::string>{"", "second"});
    REQUIRE_FALSE(second.getBlocking());
    REQUIRE(second.getMaxSize() == 4);

    pc->getColorInput("second");
    REQUIRE(pc->syncInputs.has("color/second"));
}

TEST_CASE_METHOD(HostPointCloudFixture, "Single depth stream output is unchanged", "[PointCloud][MultiStream][Compat]") {
    pc->initialConfig->setOrganized(true);
    auto depthQ = pc->inputDepth.createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    const auto extrinsics = makeExtrinsics(10.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    depthQ->send(makeDepthFrame(extrinsics, std::chrono::steady_clock::now()));

    auto pcd = waitForOutput(*outQ);
    REQUIRE(pcd->isOrganized());
    REQUIRE(pcd->getWidth() == W);
    REQUIRE(pcd->getHeight() == H);
    REQUIRE_FALSE(pcd->isColor());
    auto points = pcd->getPoints();
    REQUIRE(points.size() == W * H);
    // Frame extrinsics (frame -> CAM_A) are applied to the points
    requirePointsClose(points, 0, 10.f, 0.f, 0.f);
    // The source transformation is preserved as metadata
    const auto& transformation = pcd->getTransformation();
    REQUIRE(transformation.getIntrinsicMatrix()[0][0] == Catch::Approx(FX));
    REQUIRE(transformation.getExtrinsics().isEqualExtrinsics(extrinsics));
}

TEST_CASE_METHOD(HostPointCloudFixture, "Two depth streams are merged into one sparse cloud", "[PointCloud][MultiStream]") {
    auto depthA = pc->inputDepth.createInputQueue();
    auto depthB = pc->getDepthInput("second").createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    auto passQ = pc->passthroughDepth.createOutputQueue(8, false);
    pipeline.start();

    // Both streams are expressed relative to CAM_A of the same device; stream B is 100 mm further along X
    const auto now = std::chrono::steady_clock::now();
    const auto extrinsicsA = makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    const auto extrinsicsB = makeExtrinsics(100.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    auto frameA = makeDepthFrame(extrinsicsA, now);
    auto frameB = makeDepthFrame(extrinsicsB, now + std::chrono::milliseconds(2), 2000);
    depthA->send(frameA);
    depthB->send(frameB);

    auto pcd = waitForOutput(*outQ);
    REQUIRE_FALSE(pcd->isOrganized());
    REQUIRE(pcd->getHeight() == 1);
    REQUIRE(pcd->getWidth() == 2 * W * H);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == 2 * W * H);
    // Default stream first, then the named stream (each transformed with its own extrinsics)
    requirePointsClose(points, 0, 0.f, 0.f, 0.f, DEPTH_MM);
    requirePointsClose(points, W * H, 100.f, 0.f, 0.f, 2000.f);

    // Bounding box spans both streams
    REQUIRE(pcd->getMinZ() == Catch::Approx(1000.f));
    REQUIRE(pcd->getMaxZ() == Catch::Approx(2000.f));

    // The merged cloud is expressed in the common coordinate system (identity to dev0 / CAM_A)
    const auto outExtrinsics = pcd->getTransformation().getExtrinsics();
    REQUIRE(outExtrinsics.toCameraSocket == dai::CameraBoardSocket::CAM_A);
    REQUIRE(outExtrinsics.toDeviceId == "dev0");
    const auto outMatrix = outExtrinsics.getTransformationMatrix(false, dai::LengthUnit::MILLIMETER);
    for(int r = 0; r < 4; ++r) {
        for(int c = 0; c < 4; ++c) {
            REQUIRE(outMatrix[r][c] == Catch::Approx(r == c ? 1.f : 0.f).margin(1e-6f));
        }
    }
    REQUIRE(pcd->getTransformation().getSize() == std::make_pair<size_t, size_t>(2 * W * H, 1));
    // Timestamp follows the newest depth frame of the group
    REQUIRE(pcd->getTimestamp() == frameB->getTimestamp());

    // Every depth frame of the group is passed through, in stream order
    bool timedOut = false;
    auto pass1 = passQ->get<dai::ImgFrame>(OUTPUT_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    auto pass2 = passQ->get<dai::ImgFrame>(OUTPUT_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(pass1->getTimestamp() == frameA->getTimestamp());
    REQUIRE(pass2->getTimestamp() == frameB->getTimestamp());
}

TEST_CASE_METHOD(HostPointCloudFixture, "Organized multi-stream output stacks the streams row-wise", "[PointCloud][MultiStream]") {
    pc->initialConfig->setOrganized(true);
    auto depthA = pc->inputDepth.createInputQueue();
    auto depthB = pc->getDepthInput("second").createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    const auto now = std::chrono::steady_clock::now();
    const auto extrinsics = makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    depthA->send(makeDepthFrame(extrinsics, now));
    depthB->send(makeDepthFrame(extrinsics, now, 0));  // all-invalid depth is kept in organized mode

    auto pcd = waitForOutput(*outQ);
    REQUIRE(pcd->isOrganized());
    REQUIRE(pcd->getWidth() == W);
    REQUIRE(pcd->getHeight() == 2 * H);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == 2 * W * H);
    requirePointsClose(points, 0, 0.f, 0.f, 0.f);
    for(size_t i = W * H; i < points.size(); ++i) {
        REQUIRE(points[i].z == 0.f);
    }
}

TEST_CASE_METHOD(HostPointCloudFixture, "Streams with different target coordinate systems are dropped until they agree", "[PointCloud][MultiStream]") {
    auto depthA = pc->inputDepth.createInputQueue();
    auto depthB = pc->getDepthInput("second").createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    // Stream B points at another device's origin: nothing can be merged
    auto now = std::chrono::steady_clock::now();
    depthA->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), now));
    depthB->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev1"), now));
    bool timedOut = false;
    auto dropped = outQ->get<dai::PointCloudData>(NO_OUTPUT_WAIT, timedOut);
    REQUIRE(timedOut);
    REQUIRE(dropped == nullptr);

    // Once stream B is rebased onto dev0 (e.g. a multi-device calibration was applied) the merge resumes
    now = std::chrono::steady_clock::now();
    depthA->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), now));
    depthB->send(makeDepthFrame(makeExtrinsics(0.f, 50.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), now));
    auto pcd = waitForOutput(*outQ);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == 2 * W * H);
    requirePointsClose(points, 0, 0.f, 0.f, 0.f);
    requirePointsClose(points, W * H, 0.f, 50.f, 0.f);
}

TEST_CASE_METHOD(HostPointCloudFixture, "Merged cloud is colorized only when every stream has color", "[PointCloud][MultiStream][Colored]") {
    auto depthA = pc->inputDepth.createInputQueue();
    auto colorA = pc->getColorInput().createInputQueue();
    auto depthB = pc->getDepthInput("second").createInputQueue();
    auto colorB = pc->getColorInput("second").createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    const auto extrinsicsA = makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    const auto extrinsicsB = makeExtrinsics(0.f, 0.f, 100.f, dai::CameraBoardSocket::CAM_A, "dev0");

    SECTION("both streams colored") {
        const auto now = std::chrono::steady_clock::now();
        depthA->send(makeDepthFrame(extrinsicsA, now));
        colorA->send(makeColorFrame(extrinsicsA, now, 10, 20, 30));
        depthB->send(makeDepthFrame(extrinsicsB, now));
        colorB->send(makeColorFrame(extrinsicsB, now, 40, 50, 60));

        auto pcd = waitForOutput(*outQ);
        REQUIRE(pcd->isColor());
        auto points = pcd->getPointsRGB();
        REQUIRE(points.size() == 2 * W * H);
        for(size_t i = 0; i < W * H; ++i) {
            REQUIRE(points[i].r == 10);
            REQUIRE(points[i].g == 20);
            REQUIRE(points[i].b == 30);
            REQUIRE(points[i].z == Catch::Approx(1000.f));
        }
        for(size_t i = W * H; i < 2 * W * H; ++i) {
            REQUIRE(points[i].r == 40);
            REQUIRE(points[i].g == 50);
            REQUIRE(points[i].b == 60);
            REQUIRE(points[i].z == Catch::Approx(1100.f));
        }
    }

    SECTION("one stream without a usable color frame") {
        const auto now = std::chrono::steady_clock::now();
        depthA->send(makeDepthFrame(extrinsicsA, now));
        colorA->send(makeColorFrame(extrinsicsA, now, 10, 20, 30));
        depthB->send(makeDepthFrame(extrinsicsB, now));
        colorB->send(makeColorFrame(extrinsicsB, now, 40, 50, 60, W * 2, H));  // size mismatch

        auto pcd = waitForOutput(*outQ);
        REQUIRE_FALSE(pcd->isColor());
        REQUIRE(pcd->getPoints().size() == 2 * W * H);
    }
}

TEST_CASE_METHOD(HostPointCloudFixture, "Unlinked default depth and color inputs do not stall named streams", "[PointCloud][MultiStream]") {
    // Only named streams are linked; inputDepth and a color input created via getColorInput() stay unlinked
    auto depthA = pc->getDepthInput("a").createInputQueue();
    auto depthB = pc->getDepthInput("b").createInputQueue();
    pc->getColorInput("a");
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    REQUIRE(pc->getDepthInputNames() == std::vector<std::string>{"", "a", "b"});
    pipeline.start();
    REQUIRE(pc->getDepthInputNames() == std::vector<std::string>{"a", "b"});
    REQUIRE_FALSE(pc->syncInputs.has("color/a"));

    const auto now = std::chrono::steady_clock::now();
    const auto extrinsics = makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    depthA->send(makeDepthFrame(extrinsics, now));
    depthB->send(makeDepthFrame(extrinsics, now, 2000));

    auto pcd = waitForOutput(*outQ);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == 2 * W * H);
    requirePointsClose(points, 0, 0.f, 0.f, 0.f, DEPTH_MM);
    requirePointsClose(points, W * H, 0.f, 0.f, 0.f, 2000.f);
}

// ── Target coordinate systems on any device ──

namespace {

const std::vector<std::vector<float>> IDENTITY_ROTATION = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

/// Calibration of one device: CAM_A is the calibration origin, CAM_B is placed at `camBToCamAMm` relative to CAM_A
/// (translation of the CAM_B -> CAM_A extrinsics, in millimeters).
dai::CalibrationHandler makeDeviceCalibration(const std::vector<float>& camBToCamAMm) {
    dai::CalibrationHandler handler;
    const std::vector<std::vector<float>> intrinsics = {{FX, 0.f, CX}, {0.f, FY, CY}, {0.f, 0.f, 1.f}};
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, intrinsics, W, H);
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_B, intrinsics, W, H);
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B,
                                dai::CameraBoardSocket::CAM_A,
                                IDENTITY_ROTATION,
                                {camBToCamAMm[0] / 10.f, camBToCamAMm[1] / 10.f, camBToCamAMm[2] / 10.f},
                                {camBToCamAMm[0] / 10.f, camBToCamAMm[1] / 10.f, camBToCamAMm[2] / 10.f});
    return handler;
}

/// Calibration of one device whose housing origin is CAM_A, displaced by `housingToCamAMm`.
dai::CalibrationHandler makeHousingCalibration(const dai::Point3f& housingToCamAMm) {
    dai::EepromData eeprom;
    eeprom.housingExtrinsics.rotationMatrix = IDENTITY_ROTATION;
    eeprom.housingExtrinsics.translation = {housingToCamAMm.x / 10.f, housingToCamAMm.y / 10.f, housingToCamAMm.z / 10.f};
    eeprom.housingExtrinsics.specTranslation = eeprom.housingExtrinsics.translation;
    eeprom.housingExtrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_A;
    dai::CalibrationHandler handler(eeprom);
    const std::vector<std::vector<float>> intrinsics = {{FX, 0.f, CX}, {0.f, FY, CY}, {0.f, 0.f, 1.f}};
    handler.setCameraIntrinsics(dai::CameraBoardSocket::CAM_A, intrinsics, W, H);
    return handler;
}

/// Multi-device calibration with two devices: dev1/CAM_A -> dev0/CAM_A is a pure translation of `tzMm` along Z.
/// dev0/CAM_A is the common origin (lowest device ID).
std::vector<dai::MultiDeviceExtrinsics> makeTwoDeviceGraph(float tzMm) {
    dai::MultiDeviceExtrinsics edge;
    edge.fromDeviceId = "dev1";
    edge.fromSocket = dai::CameraBoardSocket::CAM_A;
    edge.extrinsics = makeExtrinsics(0.f, 0.f, tzMm, dai::CameraBoardSocket::CAM_A, "dev0");
    return {edge};
}

void requireTranslation(const std::array<std::array<float, 4>, 4>& matrix, float tx, float ty, float tz) {
    for(int r = 0; r < 3; ++r) {
        for(int c = 0; c < 3; ++c) {
            REQUIRE(matrix[r][c] == Catch::Approx(r == c ? 1.f : 0.f).margin(1e-5f));
        }
    }
    REQUIRE(matrix[0][3] == Catch::Approx(tx).margin(1e-3f));
    REQUIRE(matrix[1][3] == Catch::Approx(ty).margin(1e-3f));
    REQUIRE(matrix[2][3] == Catch::Approx(tz).margin(1e-3f));
}

}  // namespace

TEST_CASE("PointCloudConfig target device", "[PointCloud][Config]") {
    dai::PointCloudConfig config;
    REQUIRE(config.getTargetDeviceId().empty());

    config.setTargetCoordinateSystem("dev1", dai::CameraBoardSocket::CAM_B);
    REQUIRE(config.getCoordinateSystemType() == dai::PointCloudConfig::CoordinateSystemType::CAMERA_SOCKET);
    REQUIRE(config.getTargetCameraSocket() == dai::CameraBoardSocket::CAM_B);
    REQUIRE(config.getTargetDeviceId() == "dev1");

    config.setTargetCoordinateSystem("dev2", dai::HousingCoordinateSystem::VESA_A);
    REQUIRE(config.getCoordinateSystemType() == dai::PointCloudConfig::CoordinateSystemType::HOUSING);
    REQUIRE(config.getTargetHousingCS() == dai::HousingCoordinateSystem::VESA_A);
    REQUIRE(config.getTargetDeviceId() == "dev2");

    // The device-less overloads select the device owning the reference camera again
    config.setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_C);
    REQUIRE(config.getTargetCameraSocket() == dai::CameraBoardSocket::CAM_C);
    REQUIRE(config.getTargetDeviceId().empty());
    config.setTargetCoordinateSystem("dev1", dai::CameraBoardSocket::CAM_B);
    config.setTargetCoordinateSystem(dai::HousingCoordinateSystem::IMU);
    REQUIRE(config.getTargetDeviceId().empty());

    // The device ID survives serialization
    config.setTargetCoordinateSystem("dev1", dai::CameraBoardSocket::CAM_B);
    std::vector<std::uint8_t> metadata;
    dai::DatatypeEnum datatype;
    config.serialize(metadata, datatype);
    dai::PointCloudConfig restored;
    REQUIRE(dai::utility::deserialize(metadata, restored));
    REQUIRE(restored.getTargetDeviceId() == "dev1");
    REQUIRE(restored.getTargetCameraSocket() == dai::CameraBoardSocket::CAM_B);
    REQUIRE(restored.getCoordinateSystemType() == dai::PointCloudConfig::CoordinateSystemType::CAMERA_SOCKET);

    // Node setters forward to the initial config
    dai::Pipeline pipeline(false);
    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->setTargetCoordinateSystem("dev3", dai::CameraBoardSocket::CAM_D);
    REQUIRE(pc->initialConfig->getTargetDeviceId() == "dev3");
    REQUIRE(pc->initialConfig->getTargetCameraSocket() == dai::CameraBoardSocket::CAM_D);
    pc->setTargetCoordinateSystem("dev3", dai::HousingCoordinateSystem::VESA_B);
    REQUIRE(pc->initialConfig->getTargetHousingCS() == dai::HousingCoordinateSystem::VESA_B);
}

TEST_CASE_METHOD(HostPointCloudFixture, "Camera socket target without a device uses the reference device", "[PointCloud][Target][Compat]") {
    // dev0: CAM_B -> CAM_A is (0, -100, 0) mm, so CAM_A -> CAM_B is (0, +100, 0) mm
    pc->setDeviceCalibration("dev0", makeDeviceCalibration({0.f, -100.f, 0.f}));
    pc->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_B);
    auto depthQ = pc->inputDepth.createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    // Frame -> CAM_A of dev0 is a 10 mm shift along X
    depthQ->send(makeDepthFrame(makeExtrinsics(10.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), std::chrono::steady_clock::now()));

    auto pcd = waitForOutput(*outQ);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == W * H);
    requirePointsClose(points, 0, 10.f, 100.f, 0.f);

    const auto outExtrinsics = pcd->getTransformation().getExtrinsics();
    REQUIRE(outExtrinsics.toDeviceId == "dev0");
    REQUIRE(outExtrinsics.toCameraSocket == dai::CameraBoardSocket::CAM_B);
    requireTranslation(outExtrinsics.getTransformationMatrix(false, dai::LengthUnit::MILLIMETER), 0.f, 100.f, 0.f);
}

TEST_CASE_METHOD(HostPointCloudFixture, "Camera socket target on another device", "[PointCloud][Target][MultiDevice]") {
    // dev1/CAM_A sits 500 mm in front of dev0/CAM_A; on dev1, CAM_A -> CAM_B is (+200, 0, 0) mm
    pipeline.setMultiDeviceCalibration(makeTwoDeviceGraph(500.f));
    pc->setDeviceCalibration("dev1", makeDeviceCalibration({-200.f, 0.f, 0.f}));
    pc->setTargetCoordinateSystem("dev1", dai::CameraBoardSocket::CAM_B);
    auto depthA = pc->inputDepth.createInputQueue();
    auto depthB = pc->getDepthInput("dev1").createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    // Both frames are expressed relative to the common origin dev0/CAM_A, as the devices rebase them
    const auto now = std::chrono::steady_clock::now();
    depthA->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), now));
    depthB->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 500.f, dai::CameraBoardSocket::CAM_A, "dev0"), now, 2000));

    auto pcd = waitForOutput(*outQ);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == 2 * W * H);
    // dev0 depth: origin -> dev1/CAM_A (0, 0, -500), then CAM_A -> CAM_B (+200, 0, 0)
    requirePointsClose(points, 0, 200.f, 0.f, -500.f, DEPTH_MM);
    // dev1 depth: frame -> origin (+500 z) cancels against origin -> dev1/CAM_A
    requirePointsClose(points, W * H, 200.f, 0.f, 0.f, 2000.f);

    const auto outExtrinsics = pcd->getTransformation().getExtrinsics();
    REQUIRE(outExtrinsics.toDeviceId == "dev1");
    REQUIRE(outExtrinsics.toCameraSocket == dai::CameraBoardSocket::CAM_B);
    requireTranslation(outExtrinsics.getTransformationMatrix(false, dai::LengthUnit::MILLIMETER), 200.f, 0.f, -500.f);
}

TEST_CASE_METHOD(HostPointCloudFixture, "Housing target on another device", "[PointCloud][Target][MultiDevice]") {
    pipeline.setMultiDeviceCalibration(makeTwoDeviceGraph(500.f));
    const auto dev1Calibration = makeHousingCalibration({100.f, 0.f, 0.f});
    pc->setDeviceCalibration("dev1", dev1Calibration);
    pc->setTargetCoordinateSystem("dev1", dai::HousingCoordinateSystem::AUTO);
    auto depthQ = pc->inputDepth.createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    depthQ->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), std::chrono::steady_clock::now()));

    // Expected: origin -> dev1/CAM_A (0, 0, -500), then CAM_A -> housing as the calibration handler resolves it
    const auto camToHousing =
        dev1Calibration.getHousingCalibration(dai::CameraBoardSocket::CAM_A, dai::HousingCoordinateSystem::AUTO, true, dai::LengthUnit::MILLIMETER);
    for(int r = 0; r < 3; ++r) {
        for(int c = 0; c < 3; ++c) REQUIRE(camToHousing[r][c] == Catch::Approx(r == c ? 1.f : 0.f).margin(1e-5f));
    }
    const float tx = camToHousing[0][3], ty = camToHousing[1][3], tz = camToHousing[2][3] - 500.f;
    REQUIRE(tx != 0.f);  // the housing offset has to show up in the result

    auto pcd = waitForOutput(*outQ);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == W * H);
    requirePointsClose(points, 0, tx, ty, tz);

    const auto outExtrinsics = pcd->getTransformation().getExtrinsics();
    REQUIRE(outExtrinsics.toDeviceId == "dev1");
    REQUIRE(outExtrinsics.toCameraSocket == dai::CameraBoardSocket::AUTO);
    requireTranslation(outExtrinsics.getTransformationMatrix(false, dai::LengthUnit::MILLIMETER), tx, ty, tz);
}

TEST_CASE_METHOD(HostPointCloudFixture, "Streams with different reference devices are merged into an explicit target", "[PointCloud][Target][MultiDevice]") {
    // The frames are not rebased (each device still reports its own origin) but the host knows how the devices relate
    pipeline.setMultiDeviceCalibration(makeTwoDeviceGraph(500.f));
    pc->setDeviceCalibration("dev0", makeDeviceCalibration({0.f, -100.f, 0.f}));
    pc->setTargetCoordinateSystem("dev0", dai::CameraBoardSocket::CAM_B);
    auto depthA = pc->inputDepth.createInputQueue();
    auto depthB = pc->getDepthInput("dev1").createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    const auto now = std::chrono::steady_clock::now();
    depthA->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), now));
    depthB->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev1"), now));

    auto pcd = waitForOutput(*outQ);
    auto points = pcd->getPoints();
    REQUIRE(points.size() == 2 * W * H);
    requirePointsClose(points, 0, 0.f, 100.f, 0.f);        // dev0: CAM_A -> CAM_B
    requirePointsClose(points, W * H, 0.f, 100.f, 500.f);  // dev1: dev1/CAM_A -> dev0/CAM_A -> CAM_B
    const auto outExtrinsics = pcd->getTransformation().getExtrinsics();
    REQUIRE(outExtrinsics.toDeviceId == "dev0");
    REQUIRE(outExtrinsics.toCameraSocket == dai::CameraBoardSocket::CAM_B);
}

TEST_CASE_METHOD(HostPointCloudFixture, "Cross-device target waits for the multi-device calibration", "[PointCloud][Target][MultiDevice]") {
    pc->setDeviceCalibration("dev1", makeDeviceCalibration({-200.f, 0.f, 0.f}));
    pc->setTargetCoordinateSystem("dev1", dai::CameraBoardSocket::CAM_B);
    auto depthQ = pc->inputDepth.createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    // Without a multi-device calibration the devices cannot be related: the group is dropped, the node keeps running
    depthQ->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), std::chrono::steady_clock::now()));
    bool timedOut = false;
    auto dropped = outQ->get<dai::PointCloudData>(NO_OUTPUT_WAIT, timedOut);
    REQUIRE(timedOut);
    REQUIRE(dropped == nullptr);
    REQUIRE(pipeline.isRunning());

    // Once the calibration is known the next group is transformed
    pipeline.setMultiDeviceCalibration(makeTwoDeviceGraph(500.f));
    depthQ->send(makeDepthFrame(makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), std::chrono::steady_clock::now()));
    auto pcd = waitForOutput(*outQ);
    requirePointsClose(pcd->getPoints(), 0, 200.f, 0.f, -500.f);
}

// ── Concurrent per-stream computation ──

TEST_CASE_METHOD(HostPointCloudFixture, "Streams computed concurrently keep stream order and are deterministic", "[PointCloud][MultiStream][Parallel]") {
    // Four streams of different sizes and depths; each one carries its own extrinsics so that a mix-up between the
    // per-stream buffers (or their Impls) would show up in the merged points
    constexpr size_t STREAMS = 4;
    const unsigned widths[STREAMS] = {64, 48, 80, 32};
    const unsigned heights[STREAMS] = {40, 30, 50, 20};
    const uint16_t depths[STREAMS] = {1000, 1500, 2000, 2500};
    const float shifts[STREAMS] = {0.f, 100.f, 200.f, 300.f};

    std::vector<std::shared_ptr<dai::InputQueue>> inputs;
    for(size_t i = 0; i < STREAMS; ++i) inputs.push_back(pc->getDepthInput("s" + std::to_string(i)).createInputQueue());
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    std::vector<dai::Point3f> firstPoints;
    for(int round = 0; round < 5; ++round) {
        const auto now = std::chrono::steady_clock::now();
        for(size_t i = 0; i < STREAMS; ++i) {
            inputs[i]->send(makeDepthFrame(makeExtrinsics(shifts[i], 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0"), now, depths[i], widths[i], heights[i]));
        }
        auto pcd = waitForOutput(*outQ);
        auto points = pcd->getPoints();
        size_t expectedTotal = 0;
        for(size_t i = 0; i < STREAMS; ++i) expectedTotal += widths[i] * heights[i];
        REQUIRE(points.size() == expectedTotal);

        // Stream i occupies its block, in stream order, with its own depth and shift
        size_t offset = 0;
        for(size_t i = 0; i < STREAMS; ++i) {
            for(unsigned row = 0; row < heights[i]; ++row) {
                for(unsigned col = 0; col < widths[i]; ++col) {
                    const auto& p = points[offset + row * widths[i] + col];
                    REQUIRE(p.x == Catch::Approx((static_cast<float>(col) - CX) / FX * depths[i] + shifts[i]).margin(1e-3f));
                    REQUIRE(p.y == Catch::Approx((static_cast<float>(row) - CY) / FY * depths[i]).margin(1e-3f));
                    REQUIRE(p.z == Catch::Approx(depths[i]).margin(1e-3f));
                }
            }
            offset += widths[i] * heights[i];
        }

        // Identical inputs give identical outputs, round after round
        if(round == 0) {
            firstPoints = points;
        } else {
            REQUIRE(points.size() == firstPoints.size());
            for(size_t k = 0; k < points.size(); ++k) {
                REQUIRE(points[k].x == firstPoints[k].x);
                REQUIRE(points[k].y == firstPoints[k].y);
                REQUIRE(points[k].z == firstPoints[k].z);
            }
        }
    }
}

TEST_CASE_METHOD(HostPointCloudFixture, "Concurrent streams match the single-stream computation", "[PointCloud][MultiStream][Parallel]") {
    // The same depth frame goes through a merged node with three streams and through a single-stream node;
    // the merged cloud has to be exactly three copies of the single-stream cloud (same Impl code, different threads)
    constexpr unsigned BIG_W = 160, BIG_H = 120;
    auto single = pipeline.create<dai::node::PointCloud>();
    single->setRunOnHost(true);
    single->sync->setRunOnHost(true);
    single->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);

    auto mergedInputs = std::vector<std::shared_ptr<dai::InputQueue>>{
        pc->getDepthInput("a").createInputQueue(), pc->getDepthInput("b").createInputQueue(), pc->getDepthInput("c").createInputQueue()};
    auto singleInput = single->inputDepth.createInputQueue();
    auto mergedOut = pc->outputPointCloud.createOutputQueue(4, false);
    auto singleOut = single->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    // Depth gradient with some invalid pixels so that filtering is exercised too
    auto frame = makeDepthFrame(makeExtrinsics(5.f, -3.f, 7.f, dai::CameraBoardSocket::CAM_A, "dev0"), std::chrono::steady_clock::now(), 0, BIG_W, BIG_H);
    {
        std::vector<uint16_t> depth(BIG_W * BIG_H);
        for(size_t k = 0; k < depth.size(); ++k) depth[k] = (k % 7 == 0) ? 0 : static_cast<uint16_t>(500 + (k % 1000));
        std::vector<uint8_t> bytes(depth.size() * sizeof(uint16_t));
        std::memcpy(bytes.data(), depth.data(), bytes.size());
        frame->setData(std::move(bytes));
    }
    for(auto& input : mergedInputs) input->send(frame);
    singleInput->send(frame);

    auto merged = waitForOutput(*mergedOut)->getPoints();
    auto reference = waitForOutput(*singleOut)->getPoints();
    REQUIRE_FALSE(reference.empty());
    REQUIRE(merged.size() == 3 * reference.size());
    for(size_t copy = 0; copy < 3; ++copy) {
        for(size_t k = 0; k < reference.size(); ++k) {
            const auto& p = merged[copy * reference.size() + k];
            REQUIRE(p.x == reference[k].x);
            REQUIRE(p.y == reference[k].y);
            REQUIRE(p.z == reference[k].z);
        }
    }
}

// ── Platform GPU backend hook ──

namespace {

/// Test backend: reproduces the CPU math on the CPU, counts calls and records what it received
struct FakeGpuBackend : dai::node::PointCloudGpuBackend {
    std::atomic<int> denseCalls{0};
    std::atomic<int> coloredCalls{0};
    std::atomic<int> memorySeen{0};
    std::atomic<int> transformsSeen{0};

    template <typename PointT>
    void deproject(const Geometry& g, const std::uint8_t* depthData, PointT* points) {
        for(size_t i = 0; i < static_cast<size_t>(g.width) * g.height; ++i) {
            uint16_t depth;
            std::memcpy(&depth, depthData + i * 2, 2);
            const float z = static_cast<float>(depth) * g.depthScale;
            float x = 0.f, y = 0.f;
            if(z > 0.f) {
                x = g.rays[i].x * z;
                y = g.rays[i].y * z;
            }
            points[i].x = x;
            points[i].y = y;
            points[i].z = z;
            if(g.hasTransform && z > 0.f) {
                const auto& T = g.transform;
                const float tx = T[0][0] * x + T[0][1] * y + T[0][2] * z + T[0][3];
                const float ty = T[1][0] * x + T[1][1] * y + T[1][2] * z + T[1][3];
                const float tz = T[2][0] * x + T[2][1] * y + T[2][2] * z + T[2][3];
                points[i].x = tx;
                points[i].y = ty;
                points[i].z = tz;
            }
        }
    }

    std::vector<dai::Point3f> dense;
    std::vector<dai::Point3fRGBA> denseColored;

    const dai::Point3f* computeDense(const Geometry& g, const std::uint8_t* depthData, const std::shared_ptr<dai::Memory>& depthMemory) override {
        ++denseCalls;
        if(depthMemory) ++memorySeen;
        if(g.hasTransform) ++transformsSeen;
        dense.resize(static_cast<size_t>(g.width) * g.height);
        deproject(g, depthData, dense.data());
        return dense.data();
    }

    const dai::Point3fRGBA* computeDenseColored(const Geometry& g,
                                                const std::uint8_t* depthData,
                                                const std::shared_ptr<dai::Memory>& depthMemory,
                                                const std::uint8_t* colorData,
                                                const std::shared_ptr<dai::Memory>& colorMemory) override {
        ++coloredCalls;
        if(depthMemory && colorMemory) ++memorySeen;
        denseColored.resize(static_cast<size_t>(g.width) * g.height);
        deproject(g, depthData, denseColored.data());
        for(size_t i = 0; i < denseColored.size(); ++i) {
            denseColored[i].r = colorData[i * 3];
            denseColored[i].g = colorData[i * 3 + 1];
            denseColored[i].b = colorData[i * 3 + 2];
            denseColored[i].a = 255;
        }
        return denseColored.data();
    }
};

struct GpuBackendFactoryGuard {
    ~GpuBackendFactoryGuard() {
        dai::node::PointCloud::Impl::setGpuBackendFactory(nullptr);
    }
};

}  // namespace

TEST_CASE_METHOD(HostPointCloudFixture, "A registered GPU backend computes the points and the transformation once", "[PointCloud][GPU]") {
    GpuBackendFactoryGuard guard;
    auto backend = std::make_shared<FakeGpuBackend>();
    int factoryCalls = 0;
    dai::node::PointCloud::Impl::setGpuBackendFactory([&](std::uint32_t, std::shared_ptr<spdlog::logger>) {
        ++factoryCalls;
        return backend;
    });

    // Reference node on the CPU, GPU node through the backend; both get the same frames
    auto cpu = pipeline.create<dai::node::PointCloud>();
    cpu->setRunOnHost(true);
    cpu->sync->setRunOnHost(true);
    cpu->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    pc->useGPU(0);

    auto cpuDepth = cpu->inputDepth.createInputQueue();
    auto cpuColor = cpu->getColorInput().createInputQueue();
    auto gpuDepth = pc->inputDepth.createInputQueue();
    auto gpuColor = pc->getColorInput().createInputQueue();
    auto cpuOut = cpu->outputPointCloud.createOutputQueue(4, false);
    auto gpuOut = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();
    REQUIRE(factoryCalls == 1);

    // Frame -> CAM_A is a translation, so the extrinsics have to be applied exactly once
    const auto extrinsics = makeExtrinsics(10.f, -20.f, 30.f, dai::CameraBoardSocket::CAM_A, "dev0");
    const auto now = std::chrono::steady_clock::now();
    auto depth = makeDepthFrame(extrinsics, now, 1500);
    auto color = makeColorFrame(extrinsics, now, 1, 2, 3);
    cpuDepth->send(depth);
    cpuColor->send(color);
    gpuDepth->send(depth);
    gpuColor->send(color);

    auto reference = waitForOutput(*cpuOut);
    auto result = waitForOutput(*gpuOut);
    REQUIRE(backend->coloredCalls == 1);
    REQUIRE(backend->memorySeen == 1);
    REQUIRE(result->isColor());
    auto refPoints = reference->getPointsRGB();
    auto gpuPoints = result->getPointsRGB();
    REQUIRE(gpuPoints.size() == refPoints.size());
    REQUIRE(gpuPoints.size() == W * H);
    for(size_t i = 0; i < refPoints.size(); ++i) {
        REQUIRE(gpuPoints[i].x == Catch::Approx(refPoints[i].x).margin(1e-4f));
        REQUIRE(gpuPoints[i].y == Catch::Approx(refPoints[i].y).margin(1e-4f));
        REQUIRE(gpuPoints[i].z == Catch::Approx(refPoints[i].z).margin(1e-4f));
        REQUIRE(gpuPoints[i].r == refPoints[i].r);
        REQUIRE(gpuPoints[i].g == refPoints[i].g);
        REQUIRE(gpuPoints[i].b == refPoints[i].b);
    }
    // The translation shows up exactly once
    requirePointsClose(reference->getPoints(), 0, 10.f, -20.f, 30.f, 1500.f);
    requirePointsClose(result->getPoints(), 0, 10.f, -20.f, 30.f, 1500.f);
}

TEST_CASE_METHOD(HostPointCloudFixture, "GPU backend handles depth-only frames with distortion and a target socket", "[PointCloud][GPU]") {
    GpuBackendFactoryGuard guard;
    auto backend = std::make_shared<FakeGpuBackend>();
    dai::node::PointCloud::Impl::setGpuBackendFactory([&](std::uint32_t, std::shared_ptr<spdlog::logger>) { return backend; });

    auto cpu = pipeline.create<dai::node::PointCloud>();
    cpu->setRunOnHost(true);
    cpu->sync->setRunOnHost(true);
    cpu->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    for(auto* node : {cpu.get(), pc.get()}) {
        node->setDeviceCalibration("dev0", makeDeviceCalibration({0.f, -100.f, 0.f}));
        node->setTargetCoordinateSystem(dai::CameraBoardSocket::CAM_B);
    }
    pc->useGPU();

    auto cpuDepth = cpu->inputDepth.createInputQueue();
    auto gpuDepth = pc->inputDepth.createInputQueue();
    auto cpuOut = cpu->outputPointCloud.createOutputQueue(4, false);
    auto gpuOut = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();

    // Distorted frame with invalid pixels: the ray table carries the undistortion, the compaction drops the zeros
    const auto extrinsics = makeExtrinsics(0.f, 0.f, 0.f, dai::CameraBoardSocket::CAM_A, "dev0");
    auto frame = makeDepthFrame(extrinsics, std::chrono::steady_clock::now(), 0, 32, 24);
    {
        std::vector<uint16_t> depth(32 * 24);
        for(size_t k = 0; k < depth.size(); ++k) depth[k] = (k % 5 == 0) ? 0 : static_cast<uint16_t>(800 + k);
        std::vector<uint8_t> bytes(depth.size() * 2);
        std::memcpy(bytes.data(), depth.data(), bytes.size());
        frame->setData(std::move(bytes));
        frame->setTransformation(
            dai::ImgTransformation(32, 24, {{{FX, 0.f, 16.f}, {0.f, FY, 12.f}, {0.f, 0.f, 1.f}}}, dai::CameraModel::Perspective, {0.1f}, extrinsics));
    }
    cpuDepth->send(frame);
    gpuDepth->send(frame);

    auto refPoints = waitForOutput(*cpuOut)->getPoints();
    auto gpuPoints = waitForOutput(*gpuOut)->getPoints();
    REQUIRE(backend->denseCalls == 1);
    REQUIRE(backend->transformsSeen == 1);
    REQUIRE(refPoints.size() == 32 * 24 - (32 * 24 + 4) / 5);
    REQUIRE(gpuPoints.size() == refPoints.size());
    for(size_t i = 0; i < refPoints.size(); ++i) {
        REQUIRE(gpuPoints[i].x == Catch::Approx(refPoints[i].x).margin(1e-3f));
        REQUIRE(gpuPoints[i].y == Catch::Approx(refPoints[i].y).margin(1e-3f));
        REQUIRE(gpuPoints[i].z == Catch::Approx(refPoints[i].z).margin(1e-3f));
    }
}

TEST_CASE_METHOD(HostPointCloudFixture, "GPU request without a GPU falls back to the CPU", "[PointCloud][GPU]") {
    GpuBackendFactoryGuard guard;
    dai::node::PointCloud::Impl::setGpuBackendFactory(
        [](std::uint32_t, std::shared_ptr<spdlog::logger>) -> std::shared_ptr<dai::node::PointCloudGpuBackend> { return nullptr; });
    pc->useGPU();
    auto depthQ = pc->inputDepth.createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);
    pipeline.start();
    depthQ->send(makeDepthFrame(makeExtrinsics(1.f, 2.f, 3.f, dai::CameraBoardSocket::CAM_A, "dev0"), std::chrono::steady_clock::now()));
    auto pcd = waitForOutput(*outQ);
    requirePointsClose(pcd->getPoints(), 0, 1.f, 2.f, 3.f);
}
