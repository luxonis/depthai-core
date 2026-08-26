#include <atomic>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

// Global flag to allow for a graceful shutdown
std::atomic<bool> quitEvent(false);

void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    // Set up signal handlers for clean exit on Ctrl+C
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    constexpr float FPS = 25.0f;

    // Create the DepthAI pipeline
    dai::Pipeline pipeline;

    // Define camera sources for the stereo pair
    auto cameraLeft = pipeline.create<dai::node::Camera>();
    cameraLeft->build(dai::CameraBoardSocket::CAM_B, std::nullopt, FPS);

    auto cameraRight = pipeline.create<dai::node::Camera>();
    cameraRight->build(dai::CameraBoardSocket::CAM_C, std::nullopt, FPS);

    // Request full resolution output from each camera
    auto* leftOutput = cameraLeft->requestFullResolutionOutput();
    auto* rightOutput = cameraRight->requestFullResolutionOutput();

    // Create and build the NeuralDepth node, linking the camera outputs to it
    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);

    // Create an output queue to get the depth frames from the node
    auto depthQueue = neuralDepth->depth.createOutputQueue();

    // Start the pipeline
    pipeline.start();

    while(!quitEvent && pipeline.isRunning()) {
        auto depthData = depthQueue->get<dai::ImgFrame>();
        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depthData).getCvFrame());

        // Check for keyboard input to quit
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    // The pipeline is stopped automatically when the 'pipeline' object goes out of scope
    // at the end of the main function.
    return 0;
}
