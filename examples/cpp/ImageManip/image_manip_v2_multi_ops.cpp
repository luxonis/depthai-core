#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <opencv2/opencv.hpp>
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

        // Create camera node
        auto camRgb = pipeline.create<dai::node::Camera>();
        camRgb->build(dai::CameraBoardSocket::CAM_A);

        // Create image manipulator node
        auto manip = pipeline.create<dai::node::ImageManipV2>();

        // Configure image manipulator with multiple operations
        manip->initialConfig.setOutputSize(1270, 710, dai::ImageManipConfigV2::ResizeMode::LETTERBOX);
        manip->initialConfig.addCrop(50, 100, 500, 500);
        manip->initialConfig.addFlipVertical();
        manip->initialConfig.setFrameType(dai::ImgFrame::Type::NV12);
        manip->setMaxOutputFrameSize(2709360);

        // Link camera output to manipulator input
        camRgb->requestOutput(std::make_pair(1920, 1080), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::LETTERBOX, 20, 20)->link(manip->inputImage);

        // Create output queue
        auto out = manip->out.createOutputQueue();

        // Start pipeline
        pipeline.start();

        // Main loop
        while(!quitEvent) {
            auto inFrame = out->get<dai::ImgFrame>();
            if(inFrame != nullptr) {
                cv::imshow("Show frame", inFrame->getCvFrame());
                
                // Check for quit key
                if(cv::waitKey(1) == 'q') {
                    break;
                }
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
