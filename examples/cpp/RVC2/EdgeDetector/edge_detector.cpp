#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int signum) {
    quitEvent = true;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define cameras
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    // Request outputs
    auto rgbOut = camRgb->requestOutput(std::make_pair(1920, 1080), dai::ImgFrame::Type::GRAY8);
    auto leftOut = monoLeft->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::GRAY8);
    auto rightOut = monoRight->requestOutput(std::make_pair(640, 400), dai::ImgFrame::Type::GRAY8);

    // Define edge detectors
    auto edgeDetectorLeft = pipeline.create<dai::node::EdgeDetector>();
    auto edgeDetectorRight = pipeline.create<dai::node::EdgeDetector>();
    auto edgeDetectorRgb = pipeline.create<dai::node::EdgeDetector>();
    edgeDetectorRgb->setMaxOutputFrameSize(1920 * 1080);

    // Create input queues
    auto edgeCfgLeftQueue = edgeDetectorLeft->inputConfig.createInputQueue();
    auto edgeCfgRightQueue = edgeDetectorRight->inputConfig.createInputQueue();
    auto edgeCfgRgbQueue = edgeDetectorRgb->inputConfig.createInputQueue();

    // Link camera outputs to edge detectors
    leftOut->link(edgeDetectorLeft->inputImage);
    rightOut->link(edgeDetectorRight->inputImage);
    rgbOut->link(edgeDetectorRgb->inputImage);

    // Create output queues
    auto edgeLeftQueue = edgeDetectorLeft->outputImage.createOutputQueue();
    auto edgeRightQueue = edgeDetectorRight->outputImage.createOutputQueue();
    auto edgeRgbQueue = edgeDetectorRgb->outputImage.createOutputQueue();

    // Start pipeline
    pipeline.start();
    std::cout << "Switch between sobel filter kernels using keys '1' and '2'" << std::endl;

    while(pipeline.isRunning() && !quitEvent) {
        auto edgeLeft = edgeLeftQueue->get<dai::ImgFrame>();
        auto edgeRight = edgeRightQueue->get<dai::ImgFrame>();
        auto edgeRgb = edgeRgbQueue->get<dai::ImgFrame>();

        if(edgeLeft == nullptr || edgeRight == nullptr || edgeRgb == nullptr) continue;

        // Convert to OpenCV format and display
        cv::imshow("edge left", edgeLeft->getCvFrame());
        cv::imshow("edge right", edgeRight->getCvFrame());
        cv::imshow("edge rgb", edgeRgb->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }

        if(key == '1') {
            std::cout << "Switching sobel filter kernel." << std::endl;
            auto cfg = std::make_shared<dai::EdgeDetectorConfig>();
            std::vector<std::vector<int>> sobelHorizontalKernel = {{1, 0, -1}, {2, 0, -2}, {1, 0, -1}};
            std::vector<std::vector<int>> sobelVerticalKernel = {{1, 2, 1}, {0, 0, 0}, {-1, -2, -1}};
            cfg->setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel);
            edgeCfgLeftQueue->send(cfg);
            edgeCfgRightQueue->send(cfg);
            edgeCfgRgbQueue->send(cfg);
        }

        if(key == '2') {
            std::cout << "Switching sobel filter kernel." << std::endl;
            auto cfg = std::make_shared<dai::EdgeDetectorConfig>();
            std::vector<std::vector<int>> sobelHorizontalKernel = {{3, 0, -3}, {10, 0, -10}, {3, 0, -3}};
            std::vector<std::vector<int>> sobelVerticalKernel = {{3, 10, 3}, {0, 0, 0}, {-3, -10, -3}};
            cfg->setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel);
            edgeCfgLeftQueue->send(cfg);
            edgeCfgRightQueue->send(cfg);
            edgeCfgRgbQueue->send(cfg);
        }
    }

    // Cleanup
    pipeline.stop();
    pipeline.wait();

    return 0;
}