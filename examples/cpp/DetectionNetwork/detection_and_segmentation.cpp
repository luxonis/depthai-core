#include <opencv2/core/hal/interface.h>

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

// Helper function to normalize frame coordinates
cv::Rect frameNorm(const cv::Mat& frame, const dai::Point2f& topLeft, const dai::Point2f& bottomRight) {
    float width = frame.cols, height = frame.rows;
    return cv::Rect(cv::Point(topLeft.x * width, topLeft.y * height), cv::Point(bottomRight.x * width, bottomRight.y * height));
}

int main() {
    std::string modelName = "luxonis/yolov8-instance-segmentation-large:coco-640x352";
    bool setRunOnHost = false;
    auto device = std::make_shared<dai::Device>();

    if(device->getPlatformAsString() == "RVC2") {
        modelName = "luxonis/yolov8-instance-segmentation-nano:coco-512x288";
        setRunOnHost = true;
    }
    // Create pipeline
    dai::Pipeline pipeline{device};

    // Create and configure camera node
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    // Create and configure detection network node
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();

    dai::NNModelDescription modelDescription;

    modelDescription.model = modelName;
    detectionNetwork->build(cameraNode, modelDescription);
    detectionNetwork->detectionParser->setRunOnHost(setRunOnHost);
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
    int filteredLabel = -1;
    while(pipeline.isRunning()) {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();

        auto key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }

        if(inRgb != nullptr) {
            frame = inRgb->getCvFrame();

            // Add FPS text
            auto currentTime = std::chrono::steady_clock::now();
            float fps = counter / std::chrono::duration<float>(currentTime - startTime).count();
            cv::putText(frame, "NN fps: " + std::to_string(fps), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, textColor);
        }
        cv::Mat sidePanel(frame.rows, 400, CV_8UC3, cv::Scalar(255, 255, 255));

        if(inDet != nullptr) {
            counter++;

            // get all labels as sorted list

            auto labels = std::set<int>();
            for(const auto& detection : inDet->detections) {
                labels.insert(detection.label);
            }
            std::list<int> labelsList(labels.begin(), labels.end());
            labelsList.sort();
            std::vector<int> labelsVector(labelsList.begin(), labelsList.end());

            std::vector<std::string> labelMaps;
            for(const auto& label : labelsList) {
                labelMaps.push_back(labelMap->at(label));
            };
            cv::putText(sidePanel, "Press index to filter by class:", cv::Point(10, 20), cv::FONT_HERSHEY_TRIPLEX, 0.7, cv::Scalar(0, 0, 0), 1);

            for(size_t i = 0; i < labelMaps.size(); i++) {
                cv::putText(sidePanel,
                            std::to_string(i) + " - " + labelMaps[i],
                            cv::Point(10, 40 + static_cast<int>(i) * 20),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.7,
                            cv::Scalar(0, 0, 0),
                            1);
            }

            cv::putText(sidePanel, "0 - Show all", cv::Point(10, frame.rows - 20), cv::FONT_HERSHEY_TRIPLEX, 0.7, cv::Scalar(0, 0, 0), 1);

            if(key == '0') {
                std::printf("Showing all labels\n");
                filteredLabel = -1;
            } else if(key >= '1' && key <= '9') {
                int index = key - '1';
                if(index < static_cast<int>(labelsList.size())) {
                    std::printf("Filtering by label: %s\n", labelMaps[index].c_str());
                    filteredLabel = labelsVector[index];
                }
            }

            if(!frame.empty()) {
                auto detections = inDet->detections;

                int segWidth = inDet->getSegmentationMaskWidth();
                int segHeight = inDet->getSegmentationMaskHeight();
                std::optional<cv::Mat> segmentationMask;

                if(filteredLabel == -1) {
                    segmentationMask = inDet->getSegmentationMask();
                } else {
                    segmentationMask = inDet->getCvSegmentationMaskByClass(filteredLabel);
                    detections.erase(
                        std::remove_if(
                            detections.begin(), detections.end(), [filteredLabel](const dai::ImgDetection& det) { return det.label != filteredLabel; }),
                        detections.end());
                }
                if(segmentationMask) {
                    cv::Mat lut(1, 256, CV_8U);
                    for(int i = 0; i < 256; ++i) lut.at<uchar>(i) = (i >= 255) ? 255 : cv::saturate_cast<uchar>(i * 25);

                    cv::Mat scaledMask;
                    cv::LUT(*segmentationMask, lut, scaledMask);

                    cv::Mat coloredMask;
                    cv::applyColorMap(scaledMask, coloredMask, cv::COLORMAP_JET);
                    frame.copyTo(coloredMask, (scaledMask == 255));
                    cv::addWeighted(frame, 0.7, coloredMask, 0.3, 0, frame);
                }

                // Display detections
                for(const auto& detection : detections) {
                    auto bbox = frameNorm(frame, dai::Point2f(detection.xmin, detection.ymin), dai::Point2f(detection.xmax, detection.ymax));

                    // Draw label
                    cv::putText(frame, labelMap.value()[detection.label], cv::Point(bbox.x + 10, bbox.y + 20), cv::FONT_HERSHEY_TRIPLEX, 0.7, textColor);

                    // Draw confidence
                    cv::putText(frame,
                                std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                                cv::Point(bbox.x + 10, bbox.y + 40),
                                cv::FONT_HERSHEY_TRIPLEX,
                                0.7,
                                textColor);

                    // Draw rectangle
                    cv::rectangle(frame, bbox, color, 2);
                }
                // print sizes
                cv::hconcat(frame, sidePanel, frame);
                // cv::imshow("side panel", sidePanel);
                // Show the frame
                cv::imshow("rgb", frame);

                auto currentTime = std::chrono::steady_clock::now();
            }
        }
    }

    return 0;
}