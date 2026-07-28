#include <algorithm>
#include <csignal>
#include <iostream>
#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"
static volatile std::sig_atomic_t isRunning{1};
void signalHandler(int) {
    isRunning = 0;
}

int main() {
    std::signal(SIGINT, signalHandler);
    std::cout << "PointCloud Visualizer\n"
              << "=====================\n"
              << "Connecting to device...\n";
    dai::RemoteConnection remoteConnector;
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    std::cout << "Device: " << device->getDeviceName() << "  (ID: " << device->getDeviceId() << ")\n\n";
    const auto size = std::make_pair(640, 400);


    auto colorSocket = dai::CameraBoardSocket::CAM_A;
    for(const auto& features : device->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    auto color = pipeline.create<dai::node::Camera>();
    color->build(colorSocket);
    auto colorOut = color->requestOutput(size, dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);
    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, std::make_pair(640u, 400u));
    depth->setAlignTo(*colorOut);

    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->setRunOnHost(true);
    depth->depth().link(pc->inputDepth);
    colorOut->link(pc->getColorInput());
    remoteConnector.addTopic("pcl", pc->outputPointCloud);

    pipeline.start();
    remoteConnector.registerPipeline(pipeline);
    device->setIrLaserDotProjectorIntensity(0.7);
    std::cout << "Pipeline started.\n"
              << "Open the visualizer at http://localhost:8082 to see the point cloud.\n"
              << "Press 'q' in the viewer or Ctrl-C to quit.\n";

    while(isRunning != 0 && pipeline.isRunning()) {
        int key = remoteConnector.waitKey(1);
        if(key == 'q') {
            std::cout << "Got 'q' key from the remote connection.\n";
            break;
        }
    }
    std::cout << "Done.\n";
    return 0;
}
