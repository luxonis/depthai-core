
#include <algorithm>
#include <csignal>
#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>

#include "depthai/depthai.hpp"

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    isRunning = false;
}

int main() {
    using namespace std;
    // Default port values
    int webSocketPort = 8765;
    int httpPort = 8082;

    // Register signal handler
    std::signal(SIGINT, signalHandler);

    // Create RemoteConnection
    dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);
    // Create pipeline
    dai::Pipeline pipeline;
    // The Depth node manages its own stereo cameras and backend internally, and
    // RGBD aligns its depth to the color camera internally.
    // Pick a COLOR-capable socket for the color camera so it does not take the
    // socket the Depth node needs (e.g. the ToF sensor on ToF-only devices, where
    // the default AUTO socket would otherwise grab the only ToF-capable sensor).
    auto colorSocket = dai::CameraBoardSocket::CAM_A;
    for(const auto& features : pipeline.getDefaultDevice()->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    auto color = pipeline.create<dai::node::Camera>();
    color->build(colorSocket);
    // The (640, 400) size keeps the depth resolution the same as the RGBD frame
    // size instead of computing depth at the full stereo sensor resolution.
    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, std::make_pair(640u, 400u));
    auto rgbd = pipeline.create<dai::node::RGBD>()->build(color, depth);

    remoteConnector.addTopic("pcl", rgbd->pcl);
    pipeline.start();
    remoteConnector.registerPipeline(pipeline);
    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);
    // Main loop
    while(isRunning && pipeline.isRunning()) {
        int key = remoteConnector.waitKey(1);
        if(key == 'q') {
            std::cout << "Got 'q' key from the remote connection!" << std::endl;
            break;
        }
    }

    std::cout << "Pipeline stopped." << std::endl;
    return 0;
}
