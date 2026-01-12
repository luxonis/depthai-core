#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Helper function to normalize frame coordinates
cv::Rect frameNorm(const cv::Mat& frame, const dai::Point2f& topLeft, const dai::Point2f& bottomRight) {
    float width = frame.cols, height = frame.rows;
    return cv::Rect(cv::Point(topLeft.x * width, topLeft.y * height), cv::Point(bottomRight.x * width, bottomRight.y * height));
}

int main() {
    // Create pipeline
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline{device};

    // Create and configure camera node
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    // Create and configure detection network node
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();

    dai::NNModelDescription modelDescription;
    modelDescription.model = "luxonis/yolov8-large-pose-estimation:coco-640x352";
    if(device->getPlatform() == dai::Platform::RVC2) modelDescription.model = "luxonis/yolov8-nano-pose-estimation:coco-512x288";
    detectionNetwork->build(cameraNode, modelDescription);

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
        int frameWidth = frame.cols;
        int frameHeight = frame.rows;

        if(!frame.empty()) {
            // Display detections
            for(const auto& detection : inDet->detections) {
                auto bbox = frameNorm(frame, dai::Point2f(detection.xmin, detection.ymin), dai::Point2f(detection.xmax, detection.ymax));

                // Draw label
                cv::putText(frame, detection.labelName, cv::Point(bbox.x + 10, bbox.y + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, textColor);

                // Draw confidence
                cv::putText(frame,
                            std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                            cv::Point(bbox.x + 10, bbox.y + 40),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.5,
                            textColor);

                // Draw rectangle
                cv::rectangle(frame, bbox, color, 2);

                for(auto kp : detection.getKeypoints()) {
                    auto keypointPos = cv::Point(static_cast<int>(kp.imageCoordinates.x * frameWidth), static_cast<int>(kp.imageCoordinates.y * frameHeight));
                    cv::circle(frame, keypointPos, 3, cv::Scalar(0, 255, 0), -1);
                    if(kp.labelName != "") {
                        cv::putText(frame, kp.labelName, cv::Point(keypointPos.x + 5, keypointPos.y + 5), cv::FONT_HERSHEY_TRIPLEX, 0.4, cv::Scalar(0, 255, 0));
                    }
                }
                auto keypoints = detection.keypoints->getPoints2f();
                for(auto edge : detection.getEdges()) {
                    auto kp1 = keypoints.at(edge[0]);
                    auto kp2 = keypoints.at(edge[1]);
                    auto pt1 = cv::Point(static_cast<int>(kp1.x * frameWidth), static_cast<int>(kp1.y * frameHeight));
                    auto pt2 = cv::Point(static_cast<int>(kp2.x * frameWidth), static_cast<int>(kp2.y * frameHeight));
                    cv::line(frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
                }
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