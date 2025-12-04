#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    pipeline.enablePipelineDebugging();

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto monoLeftOut = monoLeft->requestFullResolutionOutput();
    auto monoRightOut = monoRight->requestFullResolutionOutput();

    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);

    auto disparityQueue = stereo->disparity.createOutputQueue();
    auto monoLeftEventQueue = monoLeft->pipelineEventOutput.createOutputQueue(1, false);

    double maxDisparity = 1.0;
    pipeline.start();
    while(pipeline.isRunning()) {
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        auto latestNodeEvent = monoLeftEventQueue->tryGet<dai::PipelineEvent>();

        cv::Mat npDisparity = disparity->getFrame();

        double minVal, curMax;
        cv::minMaxLoc(npDisparity, &minVal, &curMax);
        maxDisparity = std::max(maxDisparity, curMax);

        // Normalize the disparity image to an 8-bit scale.
        cv::Mat normalized;
        npDisparity.convertTo(normalized, CV_8UC1, 255.0 / maxDisparity);

        cv::Mat colorizedDisparity;
        cv::applyColorMap(normalized, colorizedDisparity, cv::COLORMAP_JET);

        // Set pixels with zero disparity to black.
        colorizedDisparity.setTo(cv::Scalar(0, 0, 0), normalized == 0);

        cv::imshow("disparity", colorizedDisparity);

        std::cout << "Latest event from MonoLeft camera node: " << (latestNodeEvent ? latestNodeEvent->str() : "No event");

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();

    return 0;
}
