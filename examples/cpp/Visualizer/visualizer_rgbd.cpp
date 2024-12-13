#include "depthai/depthai.hpp"

#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>
#include <csignal>

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    isRunning = false;
}

int main() {
    using namespace std;
    // Default port values
    int webSocketPort = 8765;
    int httpPort = 8080;

    // Register signal handler
    std::signal(SIGINT, signalHandler);

    // Create RemoteConnection
    dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);
    // Create pipeline
    dai::Pipeline pipeline;
    // Define sources and outputs
    int fps = 30;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto color = pipeline.create<dai::node::Camera>();
    stereo->setExtendedDisparity(false);
    color->build();

    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    right->setCamera("right");
    right->setFps(fps);
    stereo->setSubpixel(true);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);

    rgbd->setGPUDevice(1);
    rgbd->useGPU();
    auto *out = color->requestOutput(std::pair<int, int>(1280, 720), dai::ImgFrame::Type::RGB888i);
    left->out.link(stereo->left);
    right->out.link(stereo->right);

    stereo->depth.link(rgbd->inDepth);
    out->link(rgbd->inColor);

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
