#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

// Global flag to allow for a graceful shutdown on Ctrl+C
static std::atomic<bool> isRunning(true);

void signalHandler(int signum) {
    isRunning = false;
}

int main(int argc, char** argv) {
    // Default port values
    int webSocketPort = 8765;
    int httpPort = 8080;

    // A simple argument parser
    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if(arg == "--webSocketPort" && i + 1 < argc) {
            webSocketPort = std::stoi(argv[++i]);
        } else if(arg == "--httpPort" && i + 1 < argc) {
            httpPort = std::stoi(argv[++i]);
        }
    }

    // Register a signal handler for Ctrl+C
    std::signal(SIGINT, signalHandler);

    constexpr float FPS = 10.0f;

    try {
        // Create a remote connection for visualization
        dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);

        // Create the DepthAI pipeline
        dai::Pipeline pipeline;

        // --- Define pipeline nodes ---

        // Color camera
        auto color = pipeline.create<dai::node::Camera>();
        color->build(dai::CameraBoardSocket::CAM_A, std::nullopt, FPS);

        // Left and right mono cameras for the stereo pair
        auto left = pipeline.create<dai::node::Camera>();
        left->build(dai::CameraBoardSocket::CAM_B, std::nullopt, FPS);

        auto right = pipeline.create<dai::node::Camera>();
        right->build(dai::CameraBoardSocket::CAM_C, std::nullopt, FPS);

        // NeuralDepth node for stereo processing
        auto stereo = pipeline.create<dai::node::NeuralDepth>();

        // RGBD node to generate a point cloud
        auto rgbd = pipeline.create<dai::node::RGBD>();
        rgbd->build();

        // ImageAlign node to align depth to the color camera's perspective
        auto align = pipeline.create<dai::node::ImageAlign>();

        // --- Link pipeline nodes ---

        // Build the NeuralDepth node using full-resolution outputs from mono cameras
        auto* leftOut = left->requestFullResolutionOutput();
        auto* rightOut = right->requestFullResolutionOutput();
        stereo->build(*leftOut, *rightOut, dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);

        // Request an output from the color camera
        auto* colorOut = color->requestOutput(std::make_pair(1280, 800), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);
        if(colorOut == nullptr) {
            throw std::runtime_error("Failed to create color camera output.");
        }

        // Connect the nodes to form the data flow
        stereo->depth.link(align->input);
        colorOut->link(align->inputAlignTo);
        align->outputAligned.link(rgbd->inDepth);
        colorOut->link(rgbd->inColor);

        // Add the point cloud output as a topic for the remote visualizer
        remoteConnector.addTopic("pcl", rgbd->pcl, "common");

        // Start the pipeline
        pipeline.start();

        // Register the running pipeline with the remote connector
        remoteConnector.registerPipeline(pipeline);

        std::cout << "Pipeline started. Connect to the visualizer to see the point cloud." << std::endl;

        // Main loop to keep the application running
        while(isRunning && pipeline.isRunning()) {
            // Check for a 'q' key press from the remote connection to quit
            int key = remoteConnector.waitKey(1);
            if(key == 'q') {
                std::cout << "Received 'q' key from the remote connection. Stopping." << std::endl;
                break;
            }
        }

    } catch(const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Application finished." << std::endl;
    return 0;
}