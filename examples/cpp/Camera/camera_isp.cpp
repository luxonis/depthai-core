#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    using namespace dai;
    using namespace std;

    dai::Pipeline pipeline;

    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);

    // Request outputs
    auto videoOut = cam->requestOutput(std::make_pair(800, 400), std::nullopt, ImgResizeMode::CROP, 30.0f, std::nullopt);
    auto ispOut = cam->requestIspOutput(2.0f);

    // Create output queues
    auto videoQueue = videoOut->createOutputQueue();
    auto ispQueue = ispOut->createOutputQueue();

    pipeline.start();

    // Get first frames and print resolutions
    auto videoIn = videoQueue->get<dai::ImgFrame>();
    auto videoInIsp = ispQueue->get<dai::ImgFrame>();

    cout << "Standard output resolution = " << videoIn->getCvFrame().cols << " x " << videoIn->getCvFrame().rows << endl;

    cout << "Isp output resolution = " << videoInIsp->getCvFrame().cols << " x " << videoInIsp->getCvFrame().rows << endl;

    // Main loop
    while(pipeline.isRunning()) {
        auto videoIn = videoQueue->tryGet<dai::ImgFrame>();
        auto videoInIsp = ispQueue->tryGet<dai::ImgFrame>();

        if(videoIn) {
            cv::imshow("video", videoIn->getCvFrame());
        }
        if(videoInIsp) {
            cv::imshow("videoIsp", videoInIsp->getCvFrame());
        }

        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
