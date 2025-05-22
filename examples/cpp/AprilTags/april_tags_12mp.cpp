#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Constants
    const cv::Size FULL_RES(4000, 3000);      // 12MP
    const cv::Size PREVIEW_SIZE(1332, 1000);  // 1/3 of 12MP

    // Create nodes
    auto hostCamera = pipeline.create<dai::node::Camera>()->build();
    auto aprilTagNode = pipeline.create<dai::node::AprilTag>();
    auto manip = pipeline.create<dai::node::ImageManip>();

    // Configure nodes
    auto outputCam = hostCamera->requestOutput(std::make_pair(FULL_RES.width, FULL_RES.height));
    outputCam->link(aprilTagNode->inputImage);

    // Configure ImageManip
    manip->initialConfig->setOutputSize(PREVIEW_SIZE.width, PREVIEW_SIZE.height, dai::ImageManipConfig::ResizeMode::STRETCH);
    manip->setMaxOutputFrameSize(2162688);
    outputCam->link(manip->inputImage);

    // Create output queues
    auto outQueue = aprilTagNode->out.createOutputQueue();
    auto frameQ = manip->out.createOutputQueue();

    // Start pipeline
    pipeline.start();

    // Variables for FPS calculation
    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    float fps = 0.0f;
    const cv::Scalar color(0, 255, 0);

    while(true) {
        auto aprilTagMessage = outQueue->get<dai::AprilTags>();
        if(aprilTagMessage == nullptr) continue;

        // FPS calculation
        counter++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        if(elapsed > 1000) {
            fps = counter * 1000.0f / elapsed;
            counter = 0;
            startTime = currentTime;
        }

        // Helper function to rescale points
        auto rescale = [&FULL_RES, &PREVIEW_SIZE](const dai::Point2f& p) {
            return cv::Point(static_cast<int>(p.x / FULL_RES.width * PREVIEW_SIZE.width), static_cast<int>(p.y / FULL_RES.height * PREVIEW_SIZE.height));
        };

        auto frame = frameQ->get<dai::ImgFrame>();
        if(frame == nullptr) continue;

        cv::Mat cvFrame = frame->getCvFrame();
        for(const auto& tag : aprilTagMessage->aprilTags) {
            auto topLeft = rescale(tag.topLeft);
            auto topRight = rescale(tag.topRight);
            auto bottomRight = rescale(tag.bottomRight);
            auto bottomLeft = rescale(tag.bottomLeft);

            cv::Point center((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);

            // Draw tag boundaries
            cv::line(cvFrame, topLeft, topRight, color, 2, cv::LINE_AA, 0);
            cv::line(cvFrame, topRight, bottomRight, color, 2, cv::LINE_AA, 0);
            cv::line(cvFrame, bottomRight, bottomLeft, color, 2, cv::LINE_AA, 0);
            cv::line(cvFrame, bottomLeft, topLeft, color, 2, cv::LINE_AA, 0);

            // Draw tag ID
            std::string idStr = "ID: " + std::to_string(tag.id);
            cv::putText(cvFrame, idStr, center, cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            // Draw FPS
            cv::putText(cvFrame, "fps: " + std::to_string(fps).substr(0, 4), cv::Point(200, 20), cv::FONT_HERSHEY_TRIPLEX, 1, color);
        }

        cv::imshow("detections", cvFrame);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}