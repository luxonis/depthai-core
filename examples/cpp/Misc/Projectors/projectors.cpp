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

// Constants for intensity control
constexpr float DOT_STEP = 0.1f;
constexpr float FLOOD_STEP = 0.1f;

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create device
        auto device = std::make_shared<dai::Device>();

        // Create pipeline
        dai::Pipeline pipeline(device);

        // Create camera nodes
        auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

        // Configure outputs
        auto monoLeftOut = monoLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
        auto monoRightOut = monoRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

        // Create output queues
        auto leftQueue = monoLeftOut->createOutputQueue();
        auto rightQueue = monoRightOut->createOutputQueue();

        // Start pipeline
        pipeline.start();

        // Initialize intensities
        float dot_intensity = 1.0f;
        float flood_intensity = 1.0f;

        // Set initial intensities
        device->setIrLaserDotProjectorIntensity(dot_intensity);
        device->setIrFloodLightIntensity(flood_intensity);

        std::cout << "Controls:" << std::endl;
        std::cout << "  W/S: Increase/decrease dot projector intensity" << std::endl;
        std::cout << "  A/D: Increase/decrease flood light intensity" << std::endl;
        std::cout << "  Q: Quit" << std::endl;

        while(pipeline.isRunning() && !quitEvent) {
            auto leftSynced = leftQueue->get<dai::ImgFrame>();
            auto rightSynced = rightQueue->get<dai::ImgFrame>();

            if(leftSynced == nullptr || rightSynced == nullptr) continue;

            cv::imshow("left", leftSynced->getCvFrame());
            cv::imshow("right", rightSynced->getCvFrame());

            int key = cv::waitKey(1);
            if(key == 'q') {
                break;
            } else if(key == 'w') {
                dot_intensity += DOT_STEP;
                if(dot_intensity > 1.0f) dot_intensity = 1.0f;
                device->setIrLaserDotProjectorIntensity(dot_intensity);
                std::cout << "Dot intensity: " << dot_intensity << std::endl;
            } else if(key == 's') {
                dot_intensity -= DOT_STEP;
                if(dot_intensity < 0.0f) dot_intensity = 0.0f;
                device->setIrLaserDotProjectorIntensity(dot_intensity);
                std::cout << "Dot intensity: " << dot_intensity << std::endl;
            } else if(key == 'a') {
                flood_intensity += FLOOD_STEP;
                if(flood_intensity > 1.0f) flood_intensity = 1.0f;
                device->setIrFloodLightIntensity(flood_intensity);
                std::cout << "Flood intensity: " << flood_intensity << std::endl;
            } else if(key == 'd') {
                flood_intensity -= FLOOD_STEP;
                if(flood_intensity < 0.0f) flood_intensity = 0.0f;
                device->setIrFloodLightIntensity(flood_intensity);
                std::cout << "Flood intensity: " << flood_intensity << std::endl;
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