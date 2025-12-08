#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("basic rgbd") {
    // Create pipeline
    dai::Pipeline pipeline;
    auto platform = pipeline.getDefaultDevice()->getPlatform();
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

    auto* out = color->requestOutput(std::pair<int, int>(1280, 800), dai::ImgFrame::Type::RGB888i);
    left->requestOutput(std::pair<int, int>(1280, 800))->link(stereo->left);
    right->requestOutput(std::pair<int, int>(1280, 800))->link(stereo->right);

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
    auto rgbdQ = rgbd->rgbd.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == 1280);
        REQUIRE(pcl->getHeight() == 800);
        REQUIRE(pcl->getPoints().size() == 1280UL * 800UL);
        REQUIRE(pcl->isColor() == true);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
        auto rgbdData = rgbdQ->get<dai::RGBDData>();
        auto rgbFrame = rgbdData->getRGBFrame();
        REQUIRE(rgbFrame.has_value());
        REQUIRE(std::get<std::shared_ptr<dai::ImgFrame>>(rgbFrame.value())->getData().size() == 1280UL * 800UL * 3UL);
        auto depthFrame = rgbdData->getDepthFrame();
        REQUIRE(depthFrame.has_value());
        REQUIRE(std::get<std::shared_ptr<dai::ImgFrame>>(depthFrame.value())->getData().size() == 1280UL * 800UL * 2UL);
    }
}
TEST_CASE("rgbd autocreate") {
    // Create pipeline
    dai::Pipeline pipeline;
    auto rgbd = pipeline.create<dai::node::RGBD>()->build(true);

    auto outQ = rgbd->pcl.createOutputQueue();
    auto rgbdQ = rgbd->rgbd.createOutputQueue();
    pipeline.start();
    for(int i = 0; i < 10; ++i) {
        auto pcl = outQ->get<dai::PointCloudData>();
        REQUIRE(pcl != nullptr);
        REQUIRE(pcl->getWidth() == 640);
        REQUIRE(pcl->getHeight() == 400);
        REQUIRE(pcl->getPoints().size() == 640UL * 400UL);
        REQUIRE(pcl->isColor() == true);
        REQUIRE(pcl->getMinX() <= pcl->getMaxX());
        REQUIRE(pcl->getMinY() <= pcl->getMaxY());
        REQUIRE(pcl->getMinZ() <= pcl->getMaxZ());
        auto rgbdData = rgbdQ->get<dai::RGBDData>();
        auto rgbFrame = rgbdData->getRGBFrame();
        REQUIRE(rgbFrame.has_value());
        REQUIRE(std::get<std::shared_ptr<dai::ImgFrame>>(rgbFrame.value())->getData().size() == 640UL * 400UL * 3UL);
        auto depthFrame = rgbdData->getDepthFrame();
        REQUIRE(depthFrame.has_value());
        REQUIRE(std::get<std::shared_ptr<dai::ImgFrame>>(depthFrame.value())->getData().size() == 640UL * 400UL * 2UL);
    }
}
