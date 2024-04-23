#include "depthai/pipeline/node/host/HostCamera.hpp"

#include <opencv2/opencv.hpp>
#include <thread>

namespace dai {
namespace node {

void HostCamera::run() {
    // Open a opencv camera
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        throw std::runtime_error("Couldn't open camera");
    }
    int64_t seqNum = 0;
    while(isRunning()) {
        cv::Mat frame;
        auto success = cap.read(frame);
        if(frame.empty() || !success) {
            throw std::runtime_error("Couldn't capture frame");
        }
        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(640, 480));
        auto imgFrame = std::make_shared<ImgFrame>();
        imgFrame->setFrame(frameResized);
        imgFrame->setTimestamp(std::chrono::steady_clock::now());
        imgFrame->setSequenceNum(seqNum++);
        imgFrame->setType(ImgFrame::Type::RGB888i);
        imgFrame->setWidth(frameResized.cols);
        imgFrame->setHeight(frameResized.rows);

        cv::imshow("HostCameraPreview", imgFrame->getCvFrame());
        // Send message
        out.send(imgFrame);
    }
}

}  // namespace node
}  // namespace dai