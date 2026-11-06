#include <iostream>
#include <optional>
#include <utility>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto frameQueue = camera->requestOutput(std::make_pair(640, 400))->createOutputQueue();

    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, std::make_pair(640u, 400u));
    auto pointCloud = pipeline.create<dai::node::PointCloud>();
    depth->depth().link(pointCloud->inputDepth);
    auto pointCloudQueue = pointCloud->outputPointCloud.createOutputQueue();

    pipeline.start();
    const auto capturedFrame = frameQueue->get<dai::ImgFrame>();
    const auto capturedPointCloud = pointCloudQueue->get<dai::PointCloudData>();
    if(!capturedFrame || !capturedPointCloud) {
        std::cerr << "Failed to capture a frame or point cloud\n";
        return 1;
    }
    capturedFrame->save("./frame.dai");
    capturedPointCloud->save("./pointcloud.dai");
    pipeline.stop();

    dai::ImgFrame frame;
    frame.load("./frame.dai");
    std::cout << "Loaded frame: " << frame.getWidth() << 'x' << frame.getHeight() << '\n';

    dai::PointCloudData loadedPointCloud;
    loadedPointCloud.load("./pointcloud.dai");
    std::cout << "Loaded point cloud: " << loadedPointCloud.getPoints().size() << " points\n";
}
