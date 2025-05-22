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

    // Create nodes
    auto cameraNode = pipeline.create<dai::node::Camera>()->build();
    auto aprilTagNode = pipeline.create<dai::node::AprilTag>();

    // Configure nodes
    auto outputCam = cameraNode->requestOutput(std::make_pair(1280, 720));
    outputCam->link(aprilTagNode->inputImage);

    // Create output queues
    auto passthroughOutputQueue = aprilTagNode->passthroughInputImage.createOutputQueue();
    auto outQueue = aprilTagNode->out.createOutputQueue();

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

        auto frame = passthroughOutputQueue->get<dai::ImgFrame>();
        if(frame == nullptr) continue;

        cv::Mat cvFrame = frame->getCvFrame();

        // Helper function to convert points to integers
        auto to_int = [](const dai::Point2f& p) { return cv::Point(static_cast<int>(p.x), static_cast<int>(p.y)); };

        for(const auto& tag : aprilTagMessage->aprilTags) {
            auto topLeft = to_int(tag.topLeft);
            auto topRight = to_int(tag.topRight);
            auto bottomRight = to_int(tag.bottomRight);
            auto bottomLeft = to_int(tag.bottomLeft);

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
            cv::putText(cvFrame, "fps: " + std::to_string(fps).substr(0, 4), cv::Point(200, 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }

        cv::imshow("detections", cvFrame);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}