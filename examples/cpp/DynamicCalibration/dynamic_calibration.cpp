
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
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto* leftOut  = camLeft ->requestFullResolutionOutput();
    auto* rightOut = camRight->requestFullResolutionOutput();

    leftOut->link(stereo->left);
    rightOut->link(stereo->right);

    // Dynamic-recalibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    // Full-resolution NV12 outputs

    auto leftQueue = leftOut->createOutputQueue();
    auto rightQueue = rightOut->createOutputQueue();

    auto q = stereo->disparity.createOutputQueue();

    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    pipeline.start();
    while(pipeline.isRunning()) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto leftFrameQueue = leftQueue->get<dai::ImgFrame>();
        auto rightFrameQueue = rightQueue->get<dai::ImgFrame>();
        auto frame = inDepth->getFrame();
        frame.convertTo(frame, CV_8UC1, 255 / stereo->initialConfig->getMaxDisparity());
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);

        if(leftFrameQueue == nullptr || rightFrameQueue == nullptr ) continue;

        cv::imshow("left", leftFrameQueue->getCvFrame());
        cv::imshow("right", rightFrameQueue->getCvFrame());
        cv::imshow("disparity_color", frame);
        auto key = cv::waitKey(1);
        auto qualityResult = dynCalib->getCalibQuality();
        auto qualityCalibration = dynCalib->getNewCalibration();
        auto calibration = qualityCalibration.calibration; 

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