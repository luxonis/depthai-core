#include "depthai/depthai.hpp"
#include <opencv2/opencv.hpp>

int main() {
    dai::Pipeline pipeline;

    auto monoLeft  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto monoLeftOut  = monoLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto monoRightOut = monoRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);

    auto syncedLeftQueue  = stereo->syncedLeft.createOutputQueue();
    auto syncedRightQueue = stereo->syncedRight.createOutputQueue();
    auto disparityQueue   = stereo->disparity.createOutputQueue();

    double maxDisparity = 1.0;
    pipeline.start();
    while(true) {
        auto leftSynced  = syncedLeftQueue->get<dai::ImgFrame>();
        auto rightSynced = syncedRightQueue->get<dai::ImgFrame>();
        auto disparity   = disparityQueue->get<dai::ImgFrame>();

        cv::imshow("left", leftSynced->getCvFrame());
        cv::imshow("right", rightSynced->getCvFrame());

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

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();
    return 0;
}