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

    constexpr float FPS = 10.0f;

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

    // Create an output queue to get the disparity frames from the node
    auto disparityQueue = neuralDepth->disparity.createOutputQueue();

    // Start the pipeline
    pipeline.start();

    // Variables for visualization
    double maxDisparity = 1.0;
    cv::Mat colorMap;

    // Pre-generate the color map for efficiency
    cv::Mat gray(256, 1, CV_8UC1);
    for(int i = 0; i < 256; i++) {
        gray.at<uchar>(i) = i;
    }
    cv::applyColorMap(gray, colorMap, cv::COLORMAP_JET);
    // Set the color for zero-disparity pixels to black, as in the Python example
    colorMap.at<cv::Vec3b>(0) = cv::Vec3b(0, 0, 0);

    while(!quitEvent && pipeline.isRunning()) {
        // Get the disparity data from the queue
        auto disparityData = disparityQueue->get<dai::ImgFrame>();
        cv::Mat npDisparity = disparityData->getFrame();

        // Find the current maximum disparity value to keep the visualization normalized
        double minVal, currentMax;
        cv::minMaxLoc(npDisparity, &minVal, &currentMax);
        if(currentMax > 0) {
            maxDisparity = std::max(maxDisparity, currentMax);
        }

        // Normalize the disparity image to a 0-255 scale for color mapping
        cv::Mat normalized;
        npDisparity.convertTo(normalized, CV_8UC1, 255.0 / maxDisparity);

        // Apply the color map to create a visual representation
        cv::Mat colorizedDisparity;
        cv::applyColorMap(normalized, colorizedDisparity, colorMap);

        // Display the colorized disparity map
        cv::imshow("disparity", colorizedDisparity);

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