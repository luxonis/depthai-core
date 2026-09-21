// Minimal PointCloud example: colorized point cloud from stereo depth + RGB.
#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    // Cameras
    auto colorSockets = pipeline.getDefaultDevice()->getConnectedCameras(dai::CameraSensorType::COLOR);
    auto colorSocket = colorSockets.empty() ? dai::CameraBoardSocket::CAM_A : colorSockets.front();
    auto color = pipeline.create<dai::node::Camera>()->build(colorSocket);

    // Color output aligned to depth
    auto colorOut = color->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);

    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, std::make_pair(640u, 400u));
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
