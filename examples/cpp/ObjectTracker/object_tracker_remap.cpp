#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

// Helper function to display frames with detections
void displayFrame(const std::string& name,
                  std::shared_ptr<dai::ImgFrame> frame,
                  std::shared_ptr<dai::Tracklets> tracklets,
                  const std::vector<std::string>& labelMap) {
    cv::Scalar color(0, 255, 0);
    cv::Mat cvFrame;

    if(frame->getType() == dai::ImgFrame::Type::RAW16) {
        cvFrame = dai::utility::colorizeDepthFrame(*frame).getCvFrame();
    } else {
        cvFrame = frame->getCvFrame();
    }

    if(!tracklets) {
        // std::cout << "No detections or transformation data for " << name << std::endl;
        cv::imshow(name, cvFrame);
        return;
    }

    if(!tracklets->getTransformation().has_value()) {
        return;
    }

    const auto sourceTransform = tracklets->getTransformation().value();
    const auto& targetTransform = frame->transformation;

    for(const auto& tracklet : tracklets->tracklets) {
        auto normShape = sourceTransform.getSize();

        dai::Rect rect = tracklet.roi;
        rect = rect.denormalize(static_cast<float>(normShape.first), static_cast<float>(normShape.second));
        dai::RotatedRect rotRect(rect, 0);

        auto remapped = sourceTransform.remapRectTo(targetTransform, rotRect);
        auto bbox = remapped.getOuterRect();

        cv::putText(cvFrame,
                    labelMap[tracklet.label],
                    cv::Point(static_cast<int>(bbox[0]) + 10, static_cast<int>(bbox[1]) + 20),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    cv::Scalar(255, 255, 255));
        cv::putText(cvFrame,
                    std::to_string(static_cast<int>(tracklet.srcImgDetection.confidence * 100)) + "%",
                    cv::Point(static_cast<int>(bbox[0]) + 10, static_cast<int>(bbox[1]) + 40),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    cv::Scalar(255, 255, 255));
        cv::rectangle(cvFrame,
                      cv::Point(static_cast<int>(bbox[0]), static_cast<int>(bbox[1])),
                      cv::Point(static_cast<int>(bbox[2]), static_cast<int>(bbox[3])),
                      color,
                      2);
    }
    cv::imshow(name, cvFrame);
}

int main() {
    dai::Pipeline pipeline;

    auto colorSockets = pipeline.getDefaultDevice()->getConnectedCameras(dai::CameraSensorType::COLOR);
    auto colorSocket = colorSockets.empty() ? dai::CameraBoardSocket::CAM_A : colorSockets.front();
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build(colorSocket);

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(cameraNode, modelDescription);
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    auto labelMap = detectionNetwork->getClasses().value_or(std::vector<std::string>{});

    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO);

    detectionNetwork->out.link(objectTracker->inputDetections);
    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);

    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qTrack = objectTracker->out.createOutputQueue();
    auto qDepth = depth->depth().createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning()) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        auto inTrack = qTrack->tryGet<dai::Tracklets>();
        auto inDepth = qDepth->tryGet<dai::ImgFrame>();

        bool hasRgb = inRgb != nullptr;
        bool hasDepth = inDepth != nullptr;
        bool hasTrack = inTrack != nullptr;

        if(hasRgb && hasTrack) {
            displayFrame("rgb", inRgb, inTrack, labelMap);
        }
        if(hasDepth && hasTrack) {
            displayFrame("depth", inDepth, inTrack, labelMap);
        }

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    return 0;
}
