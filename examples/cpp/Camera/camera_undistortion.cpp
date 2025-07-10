#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create pipeline
        dai::Pipeline pipeline;

        // Define source and outputs
        auto cam = pipeline.create<dai::node::Camera>();
        cam->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30);

        // Create output queues with different resize modes
        auto croppedQueue = cam->requestOutput(std::make_pair(300, 300), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 20, 20)->createOutputQueue();

        auto stretchedQueue = cam->requestOutput(std::make_pair(300, 300), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH, 20, 20)->createOutputQueue();

        auto letterBoxedQueue =
            cam->requestOutput(std::make_pair(300, 300), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::LETTERBOX, 20, 20)->createOutputQueue();

        // Start pipeline
        pipeline.start();

        // Main loop
        while(pipeline.isRunning() && !quitEvent) {
            // Get and show cropped undistorted frame
            auto croppedIn = croppedQueue->get<dai::ImgFrame>();
            cv::imshow("cropped undistorted", croppedIn->getCvFrame());

            // Get and show stretched undistorted frame
            auto stretchedIn = stretchedQueue->get<dai::ImgFrame>();
            cv::imshow("stretched undistorted", stretchedIn->getCvFrame());

            // Get and show letterboxed undistorted frame
            auto letterBoxedIn = letterBoxedQueue->get<dai::ImgFrame>();
            cv::imshow("letterboxed undistorted", letterBoxedIn->getCvFrame());

            // Check for quit key
            if(cv::waitKey(1) == 'q') {
                break;
            }
        }

        // Cleanup
        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}