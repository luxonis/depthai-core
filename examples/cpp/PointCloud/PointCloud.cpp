// Minimal PointCloud example: colorized point cloud from stereo depth + RGB.
#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    // Color camera
    auto color = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    // Color output aligned to depth
    auto colorOut = color->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);

    // Unified Depth node. It manages its own stereo cameras and backend, and aligns
    // depth to the color output internally via setAlignTo (no ImageAlign node needed).
    auto depth = pipeline.create<dai::node::Depth>();
    depth->setAlignTo(*colorOut);

    // Point cloud
    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->initialConfig->setLengthUnit(dai::LengthUnit::METER);

    depth->depth().link(pc->inputDepth);
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
        std::cout << ", " << pcd->getWidth() << "x" << pcd->getHeight() << ", color=" << (pcd->isColor() ? "yes" : "no") << ", Z=[" << pcd->getMinZ() << ", "
                  << pcd->getMaxZ() << "]" << std::endl;
    }
    pipeline.stop();
    return 0;
}
