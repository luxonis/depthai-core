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
    // Currently visualizer doesn't display colors for PCL
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
    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto color = pipeline.create<dai::node::Camera>();
    std::shared_ptr<dai::node::ImageAlign> align;
    color->build();

    left->build(dai::CameraBoardSocket::CAM_B);
    right->build(dai::CameraBoardSocket::CAM_C);
    stereo->setSubpixel(true);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);

    auto* out = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i);

    left->requestOutput(std::pair<int, int>(640, 400))->link(stereo->left);
    right->requestOutput(std::pair<int, int>(640, 400))->link(stereo->right);

    auto platform = pipeline.getDefaultDevice()->getPlatform();
    if(platform == dai::Platform::RVC4) {
        align = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->outputAligned.link(rgbd->inDepth);
    } else {
        out->link(stereo->inputAlignTo);
        stereo->depth.link(rgbd->inDepth);
    }

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
