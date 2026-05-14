/**
 * Minimal GPUStereo example — disparity from a stereo camera pair (RVC4 only).
 */
#include <depthai/depthai.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    dai::Device device;

    dai::Pipeline pipeline(device);
    auto camLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto gpu = pipeline.create<dai::node::GPUStereo>();
    camLeft->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8, 30)->link(gpu->left);
    camRight->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8, 30)->link(gpu->right);
    gpu->setRectification(true);
    gpu->setConfidenceThreshold(25);

    auto dispQ = gpu->disparity.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto frame = dispQ->get<dai::ImgFrame>();
        cv::Mat disp(frame->getHeight(), frame->getWidth(), CV_16UC1, frame->getData().data());
        cv::Mat disp8;
        disp.convertTo(disp8, CV_8UC1, 255.0 / 96.0);
        cv::applyColorMap(disp8, disp8, cv::COLORMAP_JET);
        cv::imshow("GPUStereo Disparity", disp8);
        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
