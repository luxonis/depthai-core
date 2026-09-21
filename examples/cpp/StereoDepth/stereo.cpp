#include <atomic>
#include <csignal>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;

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

    auto depthQueue = stereo->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning() && !quitEvent) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::Mat colorizedDepth = dai::utility::colorizeDepthFrame(*depth).getCvFrame();
        cv::imshow("depth", colorizedDepth);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();
    pipeline.wait();

    return 0;
}
