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
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/PointCloud.hpp"

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
