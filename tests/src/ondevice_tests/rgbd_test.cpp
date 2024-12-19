#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

TEST_CASE("basic rgbd") {
    // Create pipeline
    dai::Pipeline pipeline;
    // Define sources and outputs
    int fps = 30;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto color = pipeline.create<dai::node::Camera>();
    stereo->setExtendedDisparity(false);
    color->build();

    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    right->setCamera("right");
    right->setFps(fps);
    stereo->setSubpixel(true);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);

    auto *out = color->requestOutput(std::pair<int, int>(1280, 720), dai::ImgFrame::Type::RGB888i);
    out->link(stereo->inputAlignTo);
    left->out.link(stereo->left);
    right->out.link(stereo->right);

    stereo->depth.link(rgbd->inDepth);
    out->link(rgbd->inColor);

    auto outQ = rgbd->pcl.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == 1280);
        REQUIRE(pcl->getHeight() == 720);
        REQUIRE(pcl->getPoints().size() == 1280UL * 720UL);
        REQUIRE(pcl->isColor() == true);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
    }
}
