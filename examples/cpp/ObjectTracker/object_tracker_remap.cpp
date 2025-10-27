#include <algorithm>  // Required for std::sort and std::unique
#include <cmath>      // Required for std::log, std::isnan, std::isinf
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "xtensor/containers/xadapt.hpp"
#include "xtensor/core/xmath.hpp"

cv::Mat colorizeDepth(cv::Mat frameDepth) {
    cv::Mat invalidMask = frameDepth == 0;
    cv::Mat depthFrameColor;

    try {
        cv::Mat frameDepthFloat;
        frameDepth.convertTo(frameDepthFloat, CV_32F);
        xt::xtensor<float, 2> depth =
            xt::adapt((float*)frameDepthFloat.data, {static_cast<size_t>(frameDepthFloat.rows), static_cast<size_t>(frameDepthFloat.cols)});

        // Get valid depth values (non-zero)
        std::vector<float> validDepth;
        validDepth.reserve(depth.size());
        std::copy_if(depth.begin(), depth.end(), std::back_inserter(validDepth), [](float x) { return x != 0; });

        if(validDepth.size() == 0) {
            return cv::Mat::zeros(frameDepth.rows, frameDepth.cols, CV_8UC3);
        }

        // Calculate percentiles
        std::sort(validDepth.begin(), validDepth.end());
        float minDepth = validDepth[static_cast<size_t>(validDepth.size() * 0.03)];
        float maxDepth = validDepth[static_cast<size_t>(validDepth.size() * 0.95)];

        // Take log of depth values
        auto logDepth = xt::eval(xt::log(depth));
        float logMinDepth = std::log(minDepth);
        float logMaxDepth = std::log(maxDepth);

        // Replace invalid values with logMinDepth using a naive implementation
        auto logDepthData = logDepth.data();
        auto depthData = depth.data();
        const size_t size = depth.size();
        for(size_t i = 0; i < size; i++) {
            if(std::isnan(logDepthData[i]) || std::isinf(logDepthData[i]) || depthData[i] == 0.0f) {
                logDepthData[i] = logMinDepth;
            }
        }

        // Clip values
        logDepth = xt::clip(logDepth, logMinDepth, logMaxDepth);

        // Normalize to 0-255 range
        auto normalizedDepth = (logDepth - logMinDepth) / (logMaxDepth - logMinDepth) * 255.0f;

        // Convert to CV_8UC1
        cv::Mat depthMat(frameDepth.rows, frameDepth.cols, CV_8UC1);
        std::transform(normalizedDepth.begin(), normalizedDepth.end(), depthMat.data, [](float x) { return static_cast<uchar>(x); });

        // Apply colormap
        cv::applyColorMap(depthMat, depthFrameColor, cv::COLORMAP_JET);

        // Set invalid pixels to black
        depthFrameColor.setTo(cv::Scalar(0, 0, 0), invalidMask);

    } catch(const std::exception& e) {
        std::cerr << "Error in colorizeDepth: " << e.what() << std::endl;
        return cv::Mat::zeros(frameDepth.rows, frameDepth.cols, CV_8UC3);
    }

    return depthFrameColor;
}

// Helper function to display frames with detections
void displayFrame(const std::string& name,
                  std::shared_ptr<dai::ImgFrame> frame,
                  std::shared_ptr<dai::Tracklets> tracklets,
                  const std::vector<std::string>& labelMap) {
    cv::Scalar color(0, 255, 0);
    cv::Mat cvFrame;

    if(frame->getType() == dai::ImgFrame::Type::RAW16) {
        cvFrame = colorizeDepth(frame->getFrame());
    } else {
        cvFrame = frame->getCvFrame();
    }

    if(!tracklets) {
        // std::cout << "No detections or transformation data for " << name << std::endl;
        cv::imshow(name, cvFrame);
        return;
    }

    const auto& sourceTransform = tracklets->transformation;
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

    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build();

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(cameraNode, modelDescription);
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    auto labelMap = detectionNetwork->getClasses().value_or(std::vector<std::string>{});

    auto monoLeft = pipeline.create<dai::node::Camera>();
    monoLeft->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>();
    monoRight->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    // Linking
    auto monoLeftOut = monoLeft->requestOutput(std::make_pair(1280, 720));
    auto monoRightOut = monoRight->requestOutput(std::make_pair(1280, 720));
    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    detectionNetwork->out.link(objectTracker->inputDetections);
    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);

    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(true);

    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qTrack = objectTracker->out.createOutputQueue();
    auto qDepth = stereo->disparity.createOutputQueue();

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
