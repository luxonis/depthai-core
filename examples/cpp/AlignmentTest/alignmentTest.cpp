#include <chrono>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <utility>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Keypoint.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

dai::ImgDetections transformDetections(dai::ImgDetections dets, dai::ImgTransformation to) {
    dai::ImgDetections transformed;
    transformed.transformation = to;
    transformed.detections.reserve(dets.detections.size());
    dai::ImgTransformation from = *dets.transformation;

    for(int i = 0; i < dets.detections.size(); i++) {
        dai::ImgDetection det = dets.detections[i];

        dai::ImgDetection transformedDet;

        // std::cout << "BBox is normalized: " << std::boolalpha << det.getBoundingBox().isNormalized() << std::endl; // true
        std::cout << "BBox coordinates BEFORE transformation: :\n";
        for(auto point : det.getBoundingBox().getPoints()) {
            std::cout << "(" << point.x << ", " << point.y << ")\n";
        }
        // dai::RotatedRect rectTransformed = from.remapRectTo(to, det.getBoundingBox());
        auto pts = det.getBoundingBox().getPoints();
        std::array<dai::Point2f, 4> rectTransformedPts;
        for(int i = 0; i < pts.size(); i++) {
            auto q = from.remapPointTo(to, pts[i]);
            rectTransformedPts[i] = q;
        }

        dai::RotatedRect rectTransformed{dai::Rect{rectTransformedPts[0], rectTransformedPts[2]}};
        transformedDet.setBoundingBox(rectTransformed);

        std::vector<dai::Point2f> transformedKps;
        transformedKps.reserve(det.getKeypoints().size());

        for(dai::Point2f kp : det.getKeypoints2f()) {
            dai::Point2f transformedKp = from.remapPointTo(to, kp);
            transformedKps.push_back(transformedKp);
        }
        transformedDet.setKeypoints(transformedKps);
        std::cout << "BBox coordinates after transformation: :\n";
        for(auto point : transformedDet.getBoundingBox().getPoints()) {
            std::cout << "(" << point.x << ", " << point.y << ")\n";
        }
        transformed.detections.push_back(transformedDet);
    }

    return transformed;
}
cv::Rect frameNorm(const cv::Mat& frame, const dai::Point2f& topLeft, const dai::Point2f& bottomRight) {
    float width = frame.cols, height = frame.rows;
    return cv::Rect(cv::Point(topLeft.x * width, topLeft.y * height), cv::Point(bottomRight.x * width, bottomRight.y * height));
}

void showDetections(dai::ImgDetections& dets, dai::ImgFrame inRgb, std::string name = "rgb") {
    cv::Mat frame = inRgb.getCvFrame();
    cv::Scalar color(255, 0, 0);
    cv::Scalar textColor(255, 255, 255);
    int frameWidth = frame.cols;
    int frameHeight = frame.rows;
    // Display detections
    for(const auto& detection : dets.detections) {
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
    cv::imshow(name, frame);
}

// int main() {
//     // Create pipeline
//     std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
//     dai::Pipeline pipeline{device};

//     // Create and configure camera node
//     auto cameraNode = pipeline.create<dai::node::Camera>();
//     cameraNode->build();

//     // Create and configure detection network node
//     auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();

//     dai::NNModelDescription modelDescription;
//     modelDescription.model = "luxonis/yolov8-large-pose-estimation:coco-640x352";
//     detectionNetwork->build(cameraNode, modelDescription);

//     auto cameraOutput = cameraNode->requestOutput({1920, 1080}, std::nullopt, dai::ImgResizeMode::CROP, 30, true);

//     // Create output queues
//     auto qRgb = detectionNetwork->passthrough.createOutputQueue();
//     auto qDet = detectionNetwork->out.createOutputQueue();
//     auto fullResOutput = cameraOutput->createOutputQueue();

//     cv::Mat frame;
//     auto startTime = std::chrono::steady_clock::now();
//     int counter = 0;
//     cv::Scalar color(255, 0, 0);
//     cv::Scalar textColor(255, 255, 255);

//     pipeline.start();
//     while(pipeline.isRunning()) {
//         auto inRgb = qRgb->get<dai::ImgFrame>();
//         auto inDet = qDet->get<dai::ImgDetections>();
//         auto inFullRes = fullResOutput->get<dai::ImgFrame>();

//         if(inRgb != nullptr) {
//             frame = inRgb->getCvFrame();

//             // Add FPS text
//             auto currentTime = std::chrono::steady_clock::now();
//             float fps = counter / std::chrono::duration<float>(currentTime - startTime).count();
//             cv::putText(frame, "NN fps: " + std::to_string(fps), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, textColor);
//         }

//         if(inDet != nullptr) {
//             counter++;
//         }
//         int frameWidth = frame.cols;
//         int frameHeight = frame.rows;

//         if(!frame.empty()) {
//             showDetections(*inDet, *inRgb);

//             dai::ImgDetections transformedDets = transformDetections(*inDet, (*inFullRes).getTransformation());
//             showDetections(transformedDets, *inFullRes, "full");
//         }

//         if(cv::waitKey(1) == 'q') {
//             break;
//         }
//     }

//     return 0;
// }

dai::Point2f normalizePoint2f(const dai::Point2f& point, unsigned int width, unsigned int height) {
    return {point.x / width, point.y / height, true};
}

dai::ImgDetections transformAprilTagsToImgDetections(const dai::AprilTags& aprilTags) {
    dai::ImgDetections dets;
    auto w = aprilTags.transformation->getSize().first;
    auto h = aprilTags.transformation->getSize().second;

    for(const auto& aprilTag : aprilTags.aprilTags) {
        dai::ImgDetection det;

        dai::Point2f tl = normalizePoint2f(aprilTag.topLeft, w, h);
        dai::Point2f br = normalizePoint2f(aprilTag.bottomRight, w, h);
        dai::Point2f tr = normalizePoint2f(aprilTag.topRight, w, h);
        dai::Point2f bl = normalizePoint2f(aprilTag.bottomLeft, w, h);

        det.setBoundingBox(dai::RotatedRect(dai::Rect(tl, br)));
        std::vector<dai::Point2f> keypoints = {tl, tr, br, bl};
        det.setKeypoints(keypoints);
        dets.detections.push_back(det);
    }
    dets.transformation = aprilTags.transformation;
    return dets;
}

int main() {
    // Create pipeline
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline{device};

    // Create and configure camera node
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    auto aprilTagNode = pipeline.create<dai::node::AprilTag>();

    auto outputCam = cameraNode->requestOutput(std::make_pair(1280, 720));
    outputCam->link(aprilTagNode->inputImage);

    auto cameraOutput = cameraNode->requestOutput({1280, 720}, std::nullopt, dai::ImgResizeMode::CROP, 30, true);
    // Create output queues

    auto qRgb = aprilTagNode->passthroughInputImage.createOutputQueue();
    auto qDet = aprilTagNode->out.createOutputQueue();
    auto fullResOutput = cameraOutput->createOutputQueue();

    cv::Mat frame;
    auto startTime = std::chrono::steady_clock::now();
    int counter = 0;
    cv::Scalar color(255, 0, 0);
    cv::Scalar textColor(255, 255, 255);

    pipeline.start();
    while(pipeline.isRunning()) {
        auto inRgb = qRgb->get<dai::ImgFrame>();
        // auto inDet = qDet->get<dai::ImgDetections>();
        auto aprilTagMessage = qDet->get<dai::AprilTags>();

        auto inFullRes = fullResOutput->get<dai::ImgFrame>();

        if(inRgb != nullptr) {
            frame = inRgb->getCvFrame();

            // Add FPS text
            auto currentTime = std::chrono::steady_clock::now();
            float fps = counter / std::chrono::duration<float>(currentTime - startTime).count();
            cv::putText(frame, "NN fps: " + std::to_string(fps), cv::Point(2, frame.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, textColor);
        }

        if(aprilTagMessage != nullptr) {
            counter++;
        }
        // dai::ImgDetections dets = *inDet;
        dai::ImgDetections dets = transformAprilTagsToImgDetections(*aprilTagMessage);
        int frameWidth = frame.cols;
        int frameHeight = frame.rows;

        if(!frame.empty()) {
            showDetections(dets, *inRgb);

            dai::ImgDetections transformedDets = transformDetections(dets, (*inFullRes).getTransformation());
            showDetections(transformedDets, *inFullRes, "undistorted");
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}