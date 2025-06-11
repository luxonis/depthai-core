#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"

constexpr auto WIDTH = 640;
constexpr auto HEIGHT = 400;
std::shared_ptr<dai::MessageQueue> configurePipeline(bool sparse, dai::Pipeline& pipeline) {
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();

    pointcloud->initialConfig->setSparse(sparse);

    monoLeft->requestOutput(std::make_pair(WIDTH, HEIGHT))->link(stereo->left);
    monoRight->requestOutput(std::make_pair(WIDTH, HEIGHT))->link(stereo->right);
    stereo->depth.link(pointcloud->inputDepth);
    return pointcloud->outputPointCloud.createOutputQueue();
}

TEST_CASE("dense pointcloud") {
    dai::Pipeline pipeline;
    auto outQ = configurePipeline(false, pipeline);
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
    auto outQ = configurePipeline(true, pipeline);
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == WIDTH);
        REQUIRE(pcl->getHeight() == HEIGHT);
        REQUIRE(pcl->getPoints().size() < WIDTH * HEIGHT);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}
