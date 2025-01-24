#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("basic rgbd") {
    // Create pipeline
    dai::Pipeline pipeline;
    auto platform = pipeline.getDefaultDevice()->getPlatform();
    auto boardName = pipeline.getDefaultDevice()->readCalibration().getEepromData().boardName;
    auto stereoPairs = pipeline.getDefaultDevice()->getAvailableStereoPairs();
    std::cout << "Detected board: " << boardName << std::endl;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto color = pipeline.create<dai::node::Camera>();
    std::shared_ptr<dai::node::ImageAlign> align = nullptr;
    if(platform == dai::Platform::RVC4) {
        align = pipeline.create<dai::node::ImageAlign>();
    }
    
    color->build();

    left->build(dai::CameraBoardSocket::CAM_B);
    right->build(dai::CameraBoardSocket::CAM_C);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);

    auto* out = color->requestOutput(std::pair<int, int>(1280, 720), dai::ImgFrame::Type::RGB888i);
    left->requestOutput(std::pair<int, int>(1280, 720))->link(stereo->left);
    right->requestOutput(std::pair<int, int>(1280, 720))->link(stereo->right);

    if(platform == dai::Platform::RVC4) {
        stereo->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->outputAligned.link(rgbd->inDepth);
    } else {
        out->link(stereo->inputAlignTo);
        stereo->depth.link(rgbd->inDepth);
    }
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
TEST_CASE("rgbd autocreate") {
    // Create pipeline
    dai::Pipeline pipeline;
    auto rgbd = pipeline.create<dai::node::RGBD>()->build(true);

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
