#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Helper function to normalize frame coordinates
cv::Rect frameNorm(const cv::Mat& frame, const dai::Point2f& topLeft, const dai::Point2f& bottomRight) {
    float width = frame.cols, height = frame.rows;
    return cv::Rect(cv::Point(topLeft.x * width, topLeft.y * height), cv::Point(bottomRight.x * width, bottomRight.y * height));
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create and configure camera node
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    // Create and configure detection network node
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();

    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(cameraNode, modelDescription);
    auto labelMap = detectionNetwork->getClasses();

    // Create output queues
    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qDet = detectionNetwork->out.createOutputQueue();

    cv::Mat frame;
    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    cv::Scalar color(255, 0, 0);
    cv::Scalar textColor(255, 255, 255);

    pipeline.start();
    while(pipeline.isRunning()) {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();

        if(inRgb != nullptr) {
            frame = inRgb->getCvFrame();

            // Add FPS text
            auto currentTime = std::chrono::steady_clock::now();
            float fps = counter / std::chrono::duration<float>(currentTime - startTime).count();
            cv::putText(frame, "NN fps: " + std::to_string(fps), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, textColor);
        }

        if(inDet != nullptr) {
            counter++;
        }

        if(!frame.empty()) {
            // Display detections
            for(const auto& detection : inDet->detections) {
                auto bbox = frameNorm(frame, dai::Point2f(detection.xmin, detection.ymin), dai::Point2f(detection.xmax, detection.ymax));

                // Draw label
                cv::putText(frame, labelMap.value()[detection.label], cv::Point(bbox.x + 10, bbox.y + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, textColor);

                // Draw confidence
                cv::putText(frame,
                            std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                            cv::Point(bbox.x + 10, bbox.y + 40),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.5,
                            textColor);

                // Draw rectangle
                cv::rectangle(frame, bbox, color, 2);
            }

            // Show the frame
            cv::imshow("rgb", frame);

            auto currentTime = std::chrono::steady_clock::now();
            float fps = counter / std::chrono::duration<float>(currentTime - startTime).count();
            std::cout << "FPS: " << fps << std::endl;
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}