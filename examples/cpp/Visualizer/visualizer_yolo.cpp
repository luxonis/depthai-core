#include <csignal>
#include <depthai/depthai.hpp>
#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>

#include "depthai/modelzoo/Zoo.hpp"

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    isRunning = false;
}

int main(int argc, char** argv) {
    // Default port values
    int webSocketPort = 8765;
    int httpPort = 8082;

    // Register signal handler
    std::signal(SIGINT, signalHandler);

    // Create RemoteConnection
    dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);

    // Create Pipeline
    dai::Pipeline pipeline;
    auto cameraNode = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    // Create and configure Detection Network
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>()->build(cameraNode, dai::NNModelDescription{"yolov6-nano"});

    // Configure output for visualization
    auto* cameraOutputVisualize = cameraNode->requestOutput(std::make_pair(512, 288), dai::ImgFrame::Type::NV12);
    if(cameraOutputVisualize == nullptr) throw std::runtime_error("Error creating camera output");
    // Set up topics for remote connection
    remoteConnector.addTopic("detections", detectionNetwork->out);
    remoteConnector.addTopic("images", *cameraOutputVisualize);
    pipeline.start();

    remoteConnector.registerPipeline(pipeline);
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
