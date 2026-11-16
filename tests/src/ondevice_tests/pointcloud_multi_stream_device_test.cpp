#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"

// The multi-stream PointCloud node runs on an RVC4 device together with its Sync subnode. Only named depth streams are
// linked (through input queues); the default inputDepth and a color input exist but are never linked. The device-side
// Sync has to leave them out instead of waiting for them, and the unlinked inputs stay valid objects.

namespace {

constexpr unsigned W = 320, H = 200;
constexpr auto OUTPUT_TIMEOUT = std::chrono::seconds(10);

std::shared_ptr<dai::ImgFrame> makeDepthFrame(std::chrono::steady_clock::time_point timestamp, unsigned seed) {
    const std::array<std::array<float, 3>, 3> intrinsics = {{{300.f, 0.f, W / 2.f}, {0.f, 300.f, H / 2.f}, {0.f, 0.f, 1.f}}};
    dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0.f, 0.f, 0.f}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    dai::ImgTransformation transformation(W, H, intrinsics, dai::CameraModel::Perspective, {}, extrinsics);

    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(W);
    frame->setHeight(H);
    frame->setType(dai::ImgFrame::Type::RAW16);
    std::vector<uint16_t> depth(W * H);
    for(size_t i = 0; i < depth.size(); ++i) depth[i] = (i % 7 == 0) ? 0 : static_cast<uint16_t>(600 + ((i * 13 + seed * 101) % 2400));
    std::vector<uint8_t> bytes(depth.size() * sizeof(uint16_t));
    std::memcpy(bytes.data(), depth.data(), bytes.size());
    frame->setData(std::move(bytes));
    frame->setTransformation(transformation);
    frame->setTimestamp(timestamp);
    frame->setSequenceNum(seed);
    return frame;
}

}  // namespace

TEST_CASE("On-device PointCloud leaves never-linked inputs out of the synchronization", "[PointCloud][MultiStream]") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) {
        WARN("Skipping: the PointCloud node runs on the device on RVC4 only");
        return;
    }
    auto node = pipeline.create<dai::node::PointCloud>();
    node->setRunOnHost(false);
    node->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);
    node->initialConfig->setOrganized(true);
    auto depthA = node->getDepthInput("a").createInputQueue();
    auto depthB = node->getDepthInput("b").createInputQueue();
    node->getColorInput("a");  // created, never linked
    auto out = node->outputPointCloud.createOutputQueue(4, false);
    REQUIRE(node->getDepthInputNames() == std::vector<std::string>{"", "a", "b"});

    pipeline.start();
    // The unlinked inputs are still there after the build (no dangling references)
    REQUIRE(node->getDepthInputNames() == std::vector<std::string>{"", "a", "b"});
    REQUIRE(node->inputDepth.getName() == "depth");
    REQUIRE_FALSE(node->inputDepth.isConnected());
    REQUIRE(node->syncInputs.has("color/a"));

    for(unsigned seed = 0; seed < 5; ++seed) {
        const auto now = std::chrono::steady_clock::now();
        depthA->send(makeDepthFrame(now, seed));
        depthB->send(makeDepthFrame(now, seed + 50));
        bool timedOut = false;
        auto cloud = out->get<dai::PointCloudData>(OUTPUT_TIMEOUT, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(cloud != nullptr);
        // Organized output stacks the two streams row-wise
        REQUIRE(cloud->getWidth() == W);
        REQUIRE(cloud->getHeight() == 2 * H);
        const auto points = cloud->getPoints();
        REQUIRE(points.size() == 2 * W * H);
        // Invalid pixels (every 7th) stay at z = 0 in both halves, valid ones carry the depth ramp
        REQUIRE(points[0].z == 0.f);
        REQUIRE(points[W * H].z == 0.f);
        REQUIRE(points[1].z > 0.f);
        REQUIRE(points[W * H + 1].z > 0.f);
        REQUIRE(points[1].z != points[W * H + 1].z);  // different seeds, different frames
    }
    pipeline.stop();
}
