
#include <depthai/depthai.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

int main() {
    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline;

    auto camLeft  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    // Dynamic-recalibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();

    // Full-resolution NV12 outputs
    auto* leftOut  = camLeft ->requestFullResolutionOutput();
    auto* rightOut = camRight->requestFullResolutionOutput();

    auto leftQueue = leftOut->createOutputQueue();
    auto rightQueue = rightOut->createOutputQueue();

    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    pipeline.start();
    while(pipeline.isRunning()) {
        auto leftFrameQueue = leftQueue->get<dai::ImgFrame>();
        auto rightFrameQueue = rightQueue->get<dai::ImgFrame>();
        if(leftFrameQueue == nullptr || rightFrameQueue == nullptr ) continue;

        cv::imshow("left", leftFrameQueue->getCvFrame());
        cv::imshow("right", rightFrameQueue->getCvFrame());
        auto key = cv::waitKey(1);
        auto qualityResult = dynCalib->getCalibQuality();

        if(key == 'q') {
            break;
        }
        else if(key == 'c') {
            dynCalib->startCalibQualityCheck();
        }
        else if(key == 'r') {
            dynCalib->startRecalibration();
        }
    }
    return 0;
}