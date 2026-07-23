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

    auto depthQueue = stereo->depth.createOutputQueue();
    auto monoLeftEventQueue = monoLeft->pipelineEventOutput.createOutputQueue(1, false);

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        auto latestNodeEvent = monoLeftEventQueue->tryGet<dai::PipelineEvent>();
        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depth, 500.0f, 12000.0f, cv::COLORMAP_JET, true).getCvFrame());

        std::cout << "Latest event from MonoLeft camera node: " << (latestNodeEvent ? latestNodeEvent->str() : "No event");

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    pipeline.stop();

    return 0;
}
