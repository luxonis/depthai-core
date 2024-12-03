#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/PointCloudData.hpp"

dai::Pipeline getPipeline(bool sparse) {
    dai::Pipeline pipeline;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    monoLeft->setCamera("left");
    monoRight->setCamera("right");

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setOutputSize(1280, 720);

    xout->setStreamName("out");

    pointcloud->initialConfig.setSparse(sparse);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(pointcloud->inputDepth);
    pointcloud->outputPointCloud.link(xout->input);

    return pipeline;
}

TEST_CASE("dense pointcloud") {
    dai::Pipeline pipeline;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();

    monoLeft->setCamera("left");
    monoRight->setCamera("right");

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setOutputSize(1280, 720);

    pointcloud->initialConfig.setSparse(false);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(pointcloud->inputDepth);

    auto outQ = pointcloud->outputPointCloud.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == 1280);
        REQUIRE(pcl->getHeight() == 720);
        REQUIRE(pcl->getPoints().size() == 1280UL * 720UL);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}

TEST_CASE("sparse pointcloud") {
    dai::Pipeline pipeline;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();

    monoLeft->setCamera("left");
    monoRight->setCamera("right");

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setOutputSize(1280, 720);

    pointcloud->initialConfig.setSparse(true);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(pointcloud->inputDepth);

    auto outQ = pointcloud->outputPointCloud.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == 1280);
        REQUIRE(pcl->getHeight() == 720);
        REQUIRE(pcl->getPoints().size() < 1280UL * 720UL);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}
