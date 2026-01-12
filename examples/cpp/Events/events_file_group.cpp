#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai/utility/EventsManager.hpp"

// Helper function to normalize frame coordinates
cv::Rect frameNorm(const cv::Mat& frame, const dai::Point2f& topLeft, const dai::Point2f& bottomRight) {
    float width = frame.cols, height = frame.rows;
    return cv::Rect(cv::Point(topLeft.x * width, topLeft.y * height), cv::Point(bottomRight.x * width, bottomRight.y * height));
}

// Callback functions
void uploadSuccessCallback(dai::utility::SendSnapCallbackResult sendSnapResult) {
    std::cout << "Successfully uploaded Snap: (" << sendSnapResult.snapName << ", " << sendSnapResult.snapTimestamp << ", "
              << sendSnapResult.snapHubID.value_or("") << ") to the hub." << std::endl;
}

void uploadFailureCallback(dai::utility::SendSnapCallbackResult sendSnapResult) {
    std::cout << "Upload of Snap: (" << sendSnapResult.snapName << ", " << sendSnapResult.snapTimestamp << ", " << sendSnapResult.snapLocalID
              << ") to the hub has failed." << std::endl;

    switch(sendSnapResult.uploadStatus) {
        case dai::utility::SendSnapCallbackStatus::FILE_BATCH_PREPARATION_FAILED:
            std::cout << "File batch preparation failed!" << std::endl;
            break;
        case dai::utility::SendSnapCallbackStatus::GROUP_CONTAINS_REJECTED_FILES:
            std::cout << "Snap's file group contains rejected files!" << std::endl;
            break;
        case dai::utility::SendSnapCallbackStatus::FILE_UPLOAD_FAILED:
            std::cout << "File upload was unsuccessful!" << std::endl;
            break;
        case dai::utility::SendSnapCallbackStatus::SEND_EVENT_FAILED:
            std::cout << "Snap could not been sent to the hub, following successful file upload!" << std::endl;
            break;
        default:
            break;
    }
}

int main() {
    dai::Pipeline pipeline(true);

    // Set your Hub team's api-key using the environment variable DEPTHAI_HUB_API_KEY. Or use the EventsManager setToken() method.
    auto eventsManager = std::make_shared<dai::utility::EventsManager>();

    auto camRgb = pipeline.create<dai::node::Camera>()->build();
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();

    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(camRgb, modelDescription);
    auto labelMap = detectionNetwork->getClasses();

    // Create output queues
    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qDet = detectionNetwork->out.createOutputQueue();

    pipeline.start();

    int counter = 0;
    while(pipeline.isRunning()) {
        if(cv::waitKey(1) != -1) {
            break;
        }

        auto inRgb = qRgb->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();
        if(inRgb == nullptr || inDet == nullptr) {
            continue;
        }

        // Display the video stream and detections
        cv::Mat frame = inRgb->getCvFrame();
        if(!frame.empty()) {
            // Display detections
            for(const auto& detection : inDet->detections) {
                auto bbox = frameNorm(frame, dai::Point2f(detection.xmin, detection.ymin), dai::Point2f(detection.xmax, detection.ymax));

                // Draw label
                cv::putText(
                    frame, labelMap.value()[detection.label], cv::Point(bbox.x + 10, bbox.y + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, cv::Scalar(255, 255, 255));

                // Draw confidence
                cv::putText(frame,
                            std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
                            cv::Point(bbox.x + 10, bbox.y + 40),
                            cv::FONT_HERSHEY_TRIPLEX,
                            0.5,
                            cv::Scalar(255, 255, 255));

                // Draw rectangle
                cv::rectangle(frame, bbox, cv::Scalar(255, 0, 0), 2);
            }

            // Show the frame
            cv::imshow("rgb", frame);
        }

        // Suppose we are only interested in the detections with confidence between 50% and 60%
        auto borderDetections = std::make_shared<dai::ImgDetections>();
        for(const auto& detection : inDet->detections) {
            if(detection.confidence > 0.5f && detection.confidence < 0.6f) {
                borderDetections->detections.emplace_back(detection);
            }
        }

        // Are there any border detections
        if(borderDetections->detections.size() > 0) {
            std::string fileName = "ImageDetection_";
            std::stringstream ss;
            ss << fileName << counter;

            auto fileGroup = std::make_shared<dai::utility::FileGroup>();
            fileGroup->addImageDetectionsPair(ss.str(), inRgb, borderDetections);
            auto localSnapID = eventsManager->sendSnap("LowConfidenceDetection",
                                                       fileGroup,
                                                       {"EventsExample", "C++"},
                                                       {{"key_0", "value_0"}, {"key_1", "value_1"}},
                                                       uploadSuccessCallback,
                                                       uploadFailureCallback);

            if(localSnapID.has_value()) {
                std::cout << "Snap with a localID: " << localSnapID.value() << " has been successfully added to the EventsManager" << std::endl;
            } else {
                std::cout << "Snap was not successfully added to the EventsManager" << std::endl;
            }

            counter++;
        }
    }

    return EXIT_SUCCESS;
}
