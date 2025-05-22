#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

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

        // Define dimensions
        int width = 1280;
        int height = 800;

        // Create camera node
        auto camRgb = pipeline.create<dai::node::Camera>();
        camRgb->build(dai::CameraBoardSocket::CAM_A);

        // Get platform and set image type
        auto platform = pipeline.getDefaultDevice()->getPlatform();
        auto imgType = (platform == dai::Platform::RVC2) ? dai::ImgFrame::Type::BGR888p : dai::ImgFrame::Type::NV12;

        // Create camera output
        auto cameraOutput = camRgb->requestOutput(std::make_pair(width, height), imgType);
        auto originalFrameQueue = cameraOutput->createOutputQueue();

        // Define 3x3 warp mesh points
        // Each point tells the warp node from which source coordinate to sample pixels
        dai::Point2f p0_3x3(0, 0);
        dai::Point2f p1_3x3(static_cast<float>(width) / 2, 200);  // Identity would be (width / 2, 0)
        dai::Point2f p2_3x3(static_cast<float>(width), 0);

        dai::Point2f p3_3x3(300, static_cast<float>(height) / 2);  // Identity would be (0, height / 2)
        dai::Point2f p4_3x3(static_cast<float>(width) / 2, static_cast<float>(height) / 2);
        dai::Point2f p5_3x3(static_cast<float>(width) - 300, static_cast<float>(height) / 2);  // Identity would be (width, height / 2)

        dai::Point2f p6_3x3(0, static_cast<float>(height));
        dai::Point2f p7_3x3(static_cast<float>(width) / 2, static_cast<float>(height) - 200);  // Identity would be (width / 2, height)
        dai::Point2f p8_3x3(static_cast<float>(width), static_cast<float>(height));

        // Create and configure warp node
        auto warp = pipeline.create<dai::node::Warp>();
        warp->setWarpMesh({p0_3x3, p1_3x3, p2_3x3, p3_3x3, p4_3x3, p5_3x3, p6_3x3, p7_3x3, p8_3x3}, 3, 3);

        // Set output size and frame limits
        std::pair<int, int> warpOutputSize(640, 480);
        warp->setOutputSize(warpOutputSize);
        warp->setMaxOutputFrameSize(warpOutputSize.first * warpOutputSize.second * 3);
        warp->setInterpolation(dai::Interpolation::BILINEAR);

        // Linking
        cameraOutput->link(warp->inputImage);
        auto warpQueue = warp->out.createOutputQueue();

        // Start pipeline
        pipeline.start();

        // Main loop
        while(!quitEvent) {
            // Get and show original frame
            auto originalFrame = originalFrameQueue->get<dai::ImgFrame>();
            cv::imshow("Original", originalFrame->getCvFrame());

            // Get and show warped frame
            auto warpedFrame = warpQueue->get<dai::ImgFrame>();
            if(platform == dai::Platform::RVC4) {
                warpedFrame->setType(dai::ImgFrame::Type::GRAY8);  // Chroma plane warping not supported on RVC4
            }
            cv::imshow("Warped", warpedFrame->getCvFrame());

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