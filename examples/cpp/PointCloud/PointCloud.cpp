// Minimal PointCloud example: colorized point cloud from stereo depth + RGB.
#include <iostream>
#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    // Cameras
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto color = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    // Stereo depth
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    left->requestFullResolutionOutput()->link(stereo->left);
    right->requestFullResolutionOutput()->link(stereo->right);

    // Color output aligned to depth
    auto colorOut = color->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::RGB888i,
                                         dai::ImgResizeMode::CROP, std::nullopt, true);

    // Point cloud
    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->initialConfig->setLengthUnit(dai::LengthUnit::METER);

    // Align depth to color on RVC4, or color to depth on RVC2/3
    auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform == dai::Platform::RVC4) {
        auto imageAlign = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(imageAlign->input);
        colorOut->link(imageAlign->inputAlignTo);
        imageAlign->outputAligned.link(pc->inputDepth);
    } else {
        colorOut->link(stereo->inputAlignTo);
        stereo->depth.link(pc->inputDepth);
    }

    colorOut->link(pc->getColorInput());

    auto q = pc->outputPointCloud.createOutputQueue(4, false);

    pipeline.start();
    while(pipeline.isRunning()) {
        auto pcd = q->get<dai::PointCloudData>();
        if(pcd->isColor()) {
            auto points = pcd->getPointsRGB();
            std::cout << "Points: " << points.size();
        } else {
            auto points = pcd->getPoints();
            std::cout << "Points: " << points.size();
        }
        std::cout << ", " << pcd->getWidth() << "x" << pcd->getHeight()
                  << ", color=" << (pcd->isColor() ? "yes" : "no")
                  << ", Z=[" << pcd->getMinZ() << ", " << pcd->getMaxZ() << "]"
                  << std::endl;
    }
    pipeline.stop();
    return 0;
}
