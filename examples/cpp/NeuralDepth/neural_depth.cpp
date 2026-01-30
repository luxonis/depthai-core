#include <atomic>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    // Set up signal handlers for clean exit
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    constexpr float FPS = 10.0f;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources
    auto cameraLeft = pipeline.create<dai::node::Camera>();
    cameraLeft->build(dai::CameraBoardSocket::CAM_B, std::nullopt, FPS);

    auto cameraRight = pipeline.create<dai::node::Camera>();
    cameraRight->build(dai::CameraBoardSocket::CAM_C, std::nullopt, FPS);

    // Get full resolution outputs
    auto leftOutput = cameraLeft->requestFullResolutionOutput();
    auto rightOutput = cameraRight->requestFullResolutionOutput();

    // Create and build NeuralDepth node
    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);

    // Create output queues
    auto confidenceQueue = neuralDepth->confidence.createOutputQueue();
    auto edgeQueue = neuralDepth->edge.createOutputQueue();
    auto disparityQueue = neuralDepth->disparity.createOutputQueue();

    // Create input queue for runtime configuration
    auto inputConfigQueue = neuralDepth->inputConfig.createInputQueue();

    // Start the pipeline
    pipeline.start();

    // Variables for visualization
    double maxDisparity = 1.0;
    cv::Mat colorMap;
    cv::Mat gray(256, 1, CV_8UC1);
    for(int i = 0; i < 256; i++) {
        gray.at<uchar>(i) = i;
    }
    cv::applyColorMap(gray, colorMap, cv::COLORMAP_JET);
    colorMap.at<cv::Vec3b>(0) = cv::Vec3b(0, 0, 0);  // Set zero-disparity pixels to black

    // Get the initial configuration
    auto currentConfig = neuralDepth->initialConfig;

    std::cout << "For adjusting thresholds, use keys:" << std::endl;
    std::cout << " - 'w': Increase confidence threshold" << std::endl;
    std::cout << " - 's': Decrease confidence threshold" << std::endl;
    std::cout << " - 'd': Increase edge threshold" << std::endl;
    std::cout << " - 'a': Decrease edge threshold" << std::endl;
    std::cout << " - 't': Toggle temporal filtering" << std::endl;

    while(!quitEvent && pipeline.isRunning()) {
        // Get confidence data and display it
        auto confidenceData = confidenceQueue->get<dai::ImgFrame>();
        cv::Mat npConfidence = confidenceData->getFrame();
        cv::Mat colorizedConfidence;
        cv::applyColorMap(npConfidence, colorizedConfidence, colorMap);
        cv::imshow("confidence", colorizedConfidence);

        // Get edge data and display it
        auto edgeData = edgeQueue->get<dai::ImgFrame>();
        cv::Mat npEdge = edgeData->getFrame();
        cv::Mat colorizedEdge;
        cv::applyColorMap(npEdge, colorizedEdge, colorMap);
        cv::imshow("edge", colorizedEdge);

        // Get disparity data, normalize, and display it
        auto disparityData = disparityQueue->get<dai::ImgFrame>();
        cv::Mat npDisparity = disparityData->getFrame();

        double minVal, curMax;
        cv::minMaxLoc(npDisparity, &minVal, &curMax);
        maxDisparity = std::max(maxDisparity, curMax);

        cv::Mat normalized;
        npDisparity.convertTo(normalized, CV_8UC1, 255.0 / maxDisparity);

        cv::Mat colorizedDisparity;
        cv::applyColorMap(normalized, colorizedDisparity, colorMap);
        cv::imshow("disparity", colorizedDisparity);

        // Check for keyboard input
        int key = cv::waitKey(1);
        bool configChanged = false;
        if(key == 'q') {
            break;
        } else if(key == 'w') {  // Increase confidence threshold
            uint8_t currentThreshold = currentConfig->getConfidenceThreshold();
            currentConfig->setConfidenceThreshold((currentThreshold + 5) % 255);
            std::cout << "Setting confidence threshold to: " << (int)currentConfig->getConfidenceThreshold() << std::endl;
            configChanged = true;
        } else if(key == 's') {  // Decrease confidence threshold
            uint8_t currentThreshold = currentConfig->getConfidenceThreshold();
            currentConfig->setConfidenceThreshold(currentThreshold - 5);  // uint8_t will wrap around on underflow
            std::cout << "Setting confidence threshold to: " << (int)currentConfig->getConfidenceThreshold() << std::endl;
            configChanged = true;
        } else if(key == 'd') {  // Increase edge threshold
            uint8_t currentThreshold = currentConfig->getEdgeThreshold();
            currentConfig->setEdgeThreshold((currentThreshold + 1) % 255);
            std::cout << "Setting edge threshold to: " << (int)currentConfig->getEdgeThreshold() << std::endl;
            configChanged = true;
        } else if(key == 'a') {  // Decrease edge threshold
            uint8_t currentThreshold = currentConfig->getEdgeThreshold();
            currentConfig->setEdgeThreshold(currentThreshold - 1);  // uint8_t will wrap around on underflow
            std::cout << "Setting edge threshold to: " << (int)currentConfig->getEdgeThreshold() << std::endl;
            configChanged = true;
        } else if(key == 't') {
            bool enable = !currentConfig->postProcessing.temporalFilter.enable;
            currentConfig->postProcessing.temporalFilter.enable = enable;
            std::cout << "Temporal filtering: " << (enable ? "on" : "off") << std::endl;
            configChanged = true;
        }

        // Send the updated configuration to the device
        if(configChanged) {
            inputConfigQueue->send(currentConfig);
        }
    }

    // Pipeline is stopped automatically when the 'pipeline' object goes out of scope.
    return 0;
}
