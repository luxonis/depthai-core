
#include <csignal>
#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>

#include "depthai/depthai.hpp"

// NOTE: Using autocreate takes over the cameras cannot be used in complex pipelines,
// where cameras would be used in other nodes as well yet.

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
    auto rgbd = pipeline.create<dai::node::RGBD>()->build(true, dai::node::StereoDepth::PresetMode::DEFAULT);

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
