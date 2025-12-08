#include <argparse/argparse.hpp>
#include <csignal>
#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>

#include "depthai/depthai.hpp"

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    (void)signum;
    isRunning = false;
}

int main(int argc, char** argv) {
    // Initialize argument parser
    argparse::ArgumentParser program("visualizer_rgbd", "1.0.0");
    program.add_description("RGBD point cloud visualizer with configurable depth source");
    program.add_argument("--webSocketPort").default_value(8765).scan<'i', int>().help("WebSocket port for remote connection");
    program.add_argument("--httpPort").default_value(8082).scan<'i', int>().help("HTTP port for remote connection");
    program.add_argument("--depthSource").default_value(std::string("stereo")).help("Depth source: stereo, neural, tof");

    // Parse arguments
    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return EXIT_FAILURE;
    }

    // Get arguments
    int webSocketPort = program.get<int>("--webSocketPort");
    int httpPort = program.get<int>("--httpPort");
    std::string depthSourceArg = program.get<std::string>("--depthSource");

    // Validate depth source argument
    if(depthSourceArg != "stereo" && depthSourceArg != "neural" && depthSourceArg != "tof") {
        std::cerr << "Invalid depth source: " << depthSourceArg << std::endl;
        std::cerr << "Valid options are: stereo, neural, tof" << std::endl;
        return EXIT_FAILURE;
    }

    // Register signal handler
    std::signal(SIGINT, signalHandler);

    try {
        // Create RemoteConnection
        dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);

        // Create pipeline
        dai::Pipeline pipeline;

        constexpr float FPS = 30.0f;
        const std::pair<int, int> size = std::make_pair(640, 400);

        // Create color camera
        auto color = pipeline.create<dai::node::Camera>();
        color->build();

        // Create depth source based on argument
        dai::node::DepthSource depthSource;

        if(depthSourceArg == "stereo") {
            auto left = pipeline.create<dai::node::Camera>();
            auto right = pipeline.create<dai::node::Camera>();
            auto stereo = pipeline.create<dai::node::StereoDepth>();

            left->build(dai::CameraBoardSocket::CAM_B);
            right->build(dai::CameraBoardSocket::CAM_C);

            stereo->setSubpixel(true);
            stereo->setExtendedDisparity(false);
            stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
            stereo->setLeftRightCheck(true);
            stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
            stereo->enableDistortionCorrection(true);
            stereo->initialConfig->setLeftRightCheckThreshold(10);

            left->requestOutput(size, std::nullopt, dai::ImgResizeMode::CROP, FPS)->link(stereo->left);
            right->requestOutput(size, std::nullopt, dai::ImgResizeMode::CROP, FPS)->link(stereo->right);

            depthSource = stereo;
        } else if(depthSourceArg == "neural") {
            auto left = pipeline.create<dai::node::Camera>();
            auto right = pipeline.create<dai::node::Camera>();

            left->build(dai::CameraBoardSocket::CAM_B);
            right->build(dai::CameraBoardSocket::CAM_C);

            auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
            neuralDepth->build(*left->requestFullResolutionOutput(), *right->requestFullResolutionOutput(), dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);

            depthSource = neuralDepth;
        } else if(depthSourceArg == "tof") {
            auto tof = pipeline.create<dai::node::ToF>();
            depthSource = tof;
        }

        // Create RGBD node using the unified build method with DepthSource variant
        auto rgbd = pipeline.create<dai::node::RGBD>();
        rgbd->build(color, depthSource, size, FPS);

        remoteConnector.addTopic("pcl", rgbd->pcl);
        pipeline.start();
        remoteConnector.registerPipeline(pipeline);

        auto device = pipeline.getDefaultDevice();
        device->setIrLaserDotProjectorIntensity(0.7);

        std::cout << "Pipeline started with depth source: " << depthSourceArg << std::endl;

        // Main loop
        while(isRunning && pipeline.isRunning()) {
            int key = remoteConnector.waitKey(1);
            if(key == 'q') {
                std::cout << "Got 'q' key from the remote connection!" << std::endl;
                break;
            }
        }

        std::cout << "Pipeline stopped." << std::endl;

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
