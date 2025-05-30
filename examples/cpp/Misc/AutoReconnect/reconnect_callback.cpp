#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    quitEvent = true;
}

// Reconnection callback function
void reconnectionCallback(dai::Device::ReconnectionStatus status) {
    std::cout << "Reconnecting state " << static_cast<int>(status) << std::endl;
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create device with reconnection settings
        auto device = std::make_shared<dai::Device>();
        device->setMaxReconnectionAttempts(3, reconnectionCallback);

        // Create pipeline
        dai::Pipeline pipeline(device);

        // Define source and output
        auto cam = pipeline.create<dai::node::Camera>()->build();
        auto output = cam->requestOutput(std::make_pair(640, 400));
        auto videoQueue = output->createOutputQueue();

        // Start pipeline
        pipeline.start();
        std::cout << "Pipeline started. Try unplugging the camera to see reconnection in action." << std::endl;
        std::cout << "Press 'q' to quit." << std::endl;

        while(pipeline.isRunning() && !quitEvent) {
            auto videoIn = videoQueue->get<dai::ImgFrame>();
            if(videoIn == nullptr) continue;

            cv::imshow("video", videoIn->getCvFrame());

            int key = cv::waitKey(1);
            if(key == 'q') {
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