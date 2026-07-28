#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"
#include "xtensor/containers/xadapt.hpp"
#include "xtensor/core/xmath.hpp"

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

cv::Mat colorizeDepth(cv::Mat frameDepth) {
    cv::Mat invalidMask = frameDepth == 0;
    cv::Mat depthFrameColor;

    try {
        cv::Mat frameDepthFloat;
        frameDepth.convertTo(frameDepthFloat, CV_32F);
        xt::xtensor<float, 2> depth =
            xt::adapt((float*)frameDepthFloat.data, {static_cast<size_t>(frameDepthFloat.rows), static_cast<size_t>(frameDepthFloat.cols)});

        std::vector<float> validDepth;
        validDepth.reserve(depth.size());
        std::copy_if(depth.begin(), depth.end(), std::back_inserter(validDepth), [](float x) { return x != 0; });

        if(validDepth.size() == 0) {
            return cv::Mat::zeros(frameDepth.rows, frameDepth.cols, CV_8UC3);
        }

        std::sort(validDepth.begin(), validDepth.end());
        float minDepth = validDepth[static_cast<size_t>(validDepth.size() * 0.03)];
        float maxDepth = validDepth[static_cast<size_t>(validDepth.size() * 0.95)];

        auto logDepth = xt::eval(xt::log(depth));
        float logMinDepth = std::log(minDepth);
        float logMaxDepth = std::log(maxDepth);

        auto logDepthData = logDepth.data();
        auto depthData = depth.data();
        const size_t size = depth.size();
        for(size_t i = 0; i < size; i++) {
            if(std::isnan(logDepthData[i]) || std::isinf(logDepthData[i]) || depthData[i] == 0.0f) {
                logDepthData[i] = logMinDepth;
            }
        }

        logDepth = xt::clip(logDepth, logMinDepth, logMaxDepth);

        auto normalizedDepth = (logDepth - logMinDepth) / (logMaxDepth - logMinDepth) * 255.0f;

        cv::Mat depthMat(frameDepth.rows, frameDepth.cols, CV_8UC1);
        std::transform(normalizedDepth.begin(), normalizedDepth.end(), depthMat.data, [](float x) { return static_cast<uchar>(x); });

        cv::applyColorMap(depthMat, depthFrameColor, cv::COLORMAP_JET);

        depthFrameColor.setTo(cv::Scalar(0, 0, 0), invalidMask);

    } catch(const std::exception& e) {
        std::cerr << "Error in colorizeDepth: " << e.what() << std::endl;
        return cv::Mat::zeros(frameDepth.rows, frameDepth.cols, CV_8UC3);
    }

    return depthFrameColor;
}

void displayFrame(const std::string& name,
                  std::shared_ptr<dai::ImgFrame> frame,
                  std::shared_ptr<dai::ImgDetections> imgDetections,
                  const std::vector<std::string>& labelMap) {
    cv::Scalar color(0, 255, 0);
    cv::Mat cvFrame;

    if(frame->getType() == dai::ImgFrame::Type::RAW16) {
        cvFrame = colorizeDepth(frame->getFrame());
    } else {
        cvFrame = frame->getCvFrame();
    }

    if(!imgDetections || !imgDetections->transformation.has_value()) {
        cv::imshow(name, cvFrame);
        return;
    }

    const auto& sourceTransform = *(imgDetections->transformation);
    const auto& targetTransform = frame->transformation;

    for(const auto& detection : imgDetections->detections) {
        auto normShape = sourceTransform.getSize();

        dai::Rect rect(dai::Point2f(detection.xmin, detection.ymin), dai::Point2f(detection.xmax, detection.ymax));
        rect = rect.denormalize(static_cast<float>(normShape.first), static_cast<float>(normShape.second));
        dai::RotatedRect rotRect(rect, 0);

        auto remapped = sourceTransform.remapRectTo(targetTransform, rotRect);
        auto bbox = remapped.getOuterRect();

        cv::putText(cvFrame,
                    labelMap[detection.label],
                    cv::Point(static_cast<int>(bbox[0]) + 10, static_cast<int>(bbox[1]) + 20),
                    cv::FONT_HERSHEY_TRIPLEX,
                    0.5,
                    cv::Scalar(255, 255, 255));
        cv::putText(cvFrame,
                    std::to_string(static_cast<int>(detection.confidence * 100)) + "%",
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
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;

    auto colorSocket = dai::CameraBoardSocket::CAM_A;
    for(const auto& features : pipeline.getDefaultDevice()->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    auto cameraNode = pipeline.create<dai::node::Camera>();
    cameraNode->build(colorSocket);

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(cameraNode, modelDescription);
    auto labelMap = detectionNetwork->getClasses().value_or(std::vector<std::string>{});

    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, std::make_pair(1280u, 720u));

    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qDet = detectionNetwork->out.createOutputQueue();
    auto qDepth = depth->depth().createOutputQueue();

    pipeline.start();

    while(pipeline.isRunning() && !quitEvent) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        auto inDet = qDet->tryGet<dai::ImgDetections>();
        auto inDepth = qDepth->tryGet<dai::ImgFrame>();

        bool hasRgb = inRgb != nullptr;
        bool hasDepth = inDepth != nullptr;
        bool hasDet = inDet != nullptr;

        if(hasRgb && hasDet) {
            displayFrame("rgb", inRgb, inDet, labelMap);
        }
        if(hasDepth && hasDet) {
            displayFrame("depth", inDepth, inDet, labelMap);
        }

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    return 0;
}