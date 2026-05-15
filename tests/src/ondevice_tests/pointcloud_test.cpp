#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"

constexpr auto WIDTH = 640;
constexpr auto HEIGHT = 400;
std::shared_ptr<dai::MessageQueue> configurePipeline(bool organized, dai::Pipeline& pipeline) {
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();

    if(organized) {
        pointcloud->initialConfig->setOrganized(true);
    }

    monoLeft->requestOutput(std::make_pair(WIDTH, HEIGHT))->link(stereo->left);
    monoRight->requestOutput(std::make_pair(WIDTH, HEIGHT))->link(stereo->right);
    stereo->depth.link(pointcloud->inputDepth);
    return pointcloud->outputPointCloud.createOutputQueue();
}

TEST_CASE("organized pointcloud") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        WARN("Skipping organized pointcloud test: PointCloud node is not supported on RVC2.");
        return;
    }
    auto outQ = configurePipeline(true, pipeline);
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == WIDTH);
        REQUIRE(pcl->getHeight() == HEIGHT);
        REQUIRE(pcl->getPoints().size() == WIDTH * HEIGHT);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}

TEST_CASE("sparse pointcloud") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        WARN("Skipping sparse pointcloud test: PointCloud node is not supported on RVC2.");
        return;
    }
    auto outQ = configurePipeline(false, pipeline);
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getHeight() == 1);
        REQUIRE(pcl->getWidth() == pcl->getPoints().size());
        REQUIRE(pcl->getPoints().size() <= WIDTH * HEIGHT);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}

// ============================================================================
// Colorization proceeds with mismatched extrinsics (warn only, no skip)
// ============================================================================
TEST_CASE("Colorization proceeds despite mismatched frame extrinsics") {
    // This test verifies that when depth and color frames have different
    // toCameraSocket extrinsics, the PointCloud node still produces a
    // colorized point cloud (with a warning) rather than falling back to
    // depth-only output.

    dai::Pipeline pipeline;

    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->initialConfig->setLengthUnit(dai::LengthUnit::MILLIMETER);

    auto depthInQ = pc->inputDepth.createInputQueue();
    auto colorInQ = pc->getColorInput().createInputQueue();
    auto outQ = pc->outputPointCloud.createOutputQueue(4, false);

    pipeline.start();

    constexpr unsigned W = 4, H = 4;
    std::array<std::array<float, 3>, 3> intrinsics = {{{100.f, 0.f, 2.f}, {0.f, 100.f, 2.f}, {0.f, 0.f, 1.f}}};

    // Depth extrinsics → CAM_B
    dai::Extrinsics depthExt({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_B);
    dai::ImgTransformation depthTransform(W, H, intrinsics, dai::CameraModel::Perspective, {}, depthExt);

    // Color extrinsics → CAM_A (mismatched!)
    dai::Extrinsics colorExt({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A);
    dai::ImgTransformation colorTransform(W, H, intrinsics, dai::CameraModel::Perspective, {}, colorExt);

    // Create synthetic depth frame (RAW16)
    auto depthFrame = std::make_shared<dai::ImgFrame>();
    depthFrame->setWidth(W);
    depthFrame->setHeight(H);
    depthFrame->setType(dai::ImgFrame::Type::RAW16);
    std::vector<uint16_t> depthData(W * H, 1000);
    std::vector<uint8_t> depthBytes(depthData.size() * sizeof(uint16_t));
    std::memcpy(depthBytes.data(), depthData.data(), depthBytes.size());
    depthFrame->setData(std::move(depthBytes));
    depthFrame->setTransformation(depthTransform);

    // Create synthetic color frame (RGB888i) — same size
    auto colorFrame = std::make_shared<dai::ImgFrame>();
    colorFrame->setWidth(W);
    colorFrame->setHeight(H);
    colorFrame->setType(dai::ImgFrame::Type::RGB888i);
    std::vector<uint8_t> colorData(W * H * 3, 0);
    for(unsigned i = 0; i < W * H; ++i) {
        colorData[i * 3 + 0] = 100;  // R
        colorData[i * 3 + 1] = 150;  // G
        colorData[i * 3 + 2] = 200;  // B
    }
    colorFrame->setData(std::move(colorData));
    colorFrame->setTransformation(colorTransform);

    // Send frames
    depthInQ->send(depthFrame);
    colorInQ->send(colorFrame);

    // Get output — should be colorized despite extrinsics mismatch
    auto pcd = outQ->get<dai::PointCloudData>();
    REQUIRE(pcd != nullptr);
    REQUIRE(pcd->isColor());
    REQUIRE(pcd->getWidth() > 0);

    // Verify color data is present
    auto points = pcd->getPointsRGB();
    REQUIRE(!points.empty());
    for(const auto& p : points) {
        if(p.z > 0.f) {
            REQUIRE(p.r == 100);
            REQUIRE(p.g == 150);
            REQUIRE(p.b == 200);
        }
    }

    pipeline.stop();
}
