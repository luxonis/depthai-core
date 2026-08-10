#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

const std::string SOURCE_WINDOW = "Source window CAM_B";
const std::string RGB_WINDOW = "RGB window CAM_A";

std::atomic<bool> quitEvent(false);
std::optional<cv::Point> selectedPoint = std::nullopt;

void signalHandler(int) {
    quitEvent = true;
}

void onLeftClick(int event, int x, int y, int flags, void* param) {
    (void)flags;
    (void)param;

    if(event == cv::EVENT_LBUTTONDOWN) {
        selectedPoint = cv::Point(x, y);
    }
}

cv::Mat toColorFrame(const cv::Mat& frame) {
    if(frame.channels() == 3) {
        return frame;
    }

    cv::Mat colorFrame;
    cv::cvtColor(frame, colorFrame, cv::COLOR_GRAY2BGR);
    return colorFrame;
}

void drawPoint(cv::Mat& frame, const std::optional<dai::Point2f>& point, const std::string& label, const cv::Scalar& color) {
    if(point.has_value()) {
        const cv::Point intPoint(static_cast<int>(std::lround(point->x)), static_cast<int>(std::lround(point->y)));
        cv::drawMarker(frame, intPoint, color, cv::MARKER_CROSS, 16, 2);
        cv::circle(frame, intPoint, 6, color, 1);
        cv::putText(frame, label, cv::Point(intPoint.x + 10, std::max(20, intPoint.y - 10)), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
    } else {
        cv::putText(frame, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv::LINE_AA);
    }
}

std::pair<std::optional<float>, std::string> sampleDepth(const std::optional<cv::Point>& point,
                                                         const std::shared_ptr<dai::ImgFrame>& depthFrame,
                                                         int patchRadius = 2) {
    if(!point.has_value()) {
        return {std::nullopt, "Left click to select a point"};
    }

    cv::Mat depthData = depthFrame->getFrame();
    const int height = depthData.rows;
    const int width = depthData.cols;
    const int x = point->x;
    const int y = point->y;

    if(x < 0 || x >= width || y < 0 || y >= height) {
        return {std::nullopt, "Selected point is outside the depth frame"};
    }

    const int xStart = std::max(0, x - patchRadius);
    const int xEnd = std::min(width, x + patchRadius + 1);
    const int yStart = std::max(0, y - patchRadius);
    const int yEnd = std::min(height, y + patchRadius + 1);

    std::vector<float> validDepth;
    validDepth.reserve(static_cast<size_t>((xEnd - xStart) * (yEnd - yStart)));
    for(int yy = yStart; yy < yEnd; ++yy) {
        for(int xx = xStart; xx < xEnd; ++xx) {
            const uint16_t depth = depthData.at<uint16_t>(yy, xx);
            if(depth > 0) {
                validDepth.push_back(static_cast<float>(depth));
            }
        }
    }

    if(validDepth.empty()) {
        return {std::nullopt, "No valid depth at selected point"};
    }

    std::sort(validDepth.begin(), validDepth.end());
    float depthMm = validDepth[validDepth.size() / 2];
    if(validDepth.size() % 2 == 0) {
        depthMm = (validDepth[validDepth.size() / 2 - 1] + validDepth[validDepth.size() / 2]) * 0.5f;
    }

    std::ostringstream status;
    status << "z=" << std::fixed << std::setprecision(0) << depthMm << "mm";
    return {depthMm, status.str()};
}

std::string formatPointStatus(const std::string& prefix, const dai::Point2f& point, float depthMm) {
    std::ostringstream status;
    status << std::fixed << std::setprecision(1) << prefix << "=(" << point.x << ", " << point.y << ") ";
    status << std::setprecision(0) << "z=" << depthMm << "mm";
    return status.str();
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;

    auto rgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto monoLeftOut = monoLeft->requestFullResolutionOutput();
    auto monoRightOut = monoRight->requestFullResolutionOutput();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    auto rgbOut = rgb->requestOutput({720, 480}, std::nullopt, dai::ImgResizeMode::CROP, std::nullopt, false);
    auto rgbQueue = rgbOut->createOutputQueue();
    auto depthQueue = stereo->depth.createOutputQueue();
    auto rectifiedLeftQueue = stereo->rectifiedLeft.createOutputQueue();

    cv::namedWindow(SOURCE_WINDOW);
    cv::namedWindow(RGB_WINDOW);
    cv::setMouseCallback(SOURCE_WINDOW, onLeftClick);

    pipeline.start();
    while(pipeline.isRunning() && !quitEvent) {
        auto rgbFrame = rgbQueue->get<dai::ImgFrame>();
        auto depthFrame = depthQueue->get<dai::ImgFrame>();
        auto rectifiedLeft = rectifiedLeftQueue->get<dai::ImgFrame>();

        if(rgbFrame == nullptr || depthFrame == nullptr || rectifiedLeft == nullptr) {
            continue;
        }

        if(!rgbFrame->validateTransformations() || !depthFrame->validateTransformations() || !rectifiedLeft->validateTransformations()) {
            std::cerr << "Invalid transformations!" << std::endl;
            continue;
        }

        auto& sourceTransformation = rectifiedLeft->getTransformation();
        auto& rgbTransformation = rgbFrame->getTransformation();
        auto& depthTransformation = depthFrame->getTransformation();

        cv::Mat leftFrame = toColorFrame(rectifiedLeft->getCvFrame());
        cv::Mat rgbDisplay = rgbFrame->getCvFrame();
        auto [depthMm, sourceStatus] = sampleDepth(selectedPoint, depthFrame);

        std::optional<dai::Point2f> remappedRgbPoint = std::nullopt;
        std::optional<dai::Point2f> remappedDepthPoint = std::nullopt;
        std::string rgbStatus;
        std::string depthStatus;

        std::optional<dai::Point2f> originalPoint = std::nullopt;
        if(selectedPoint.has_value() && depthMm.has_value()) {
            dai::Point2f sourcePoint(static_cast<float>(selectedPoint->x), static_cast<float>(selectedPoint->y));
            originalPoint = sourcePoint;

            try {
                remappedRgbPoint = sourceTransformation.projectPointTo(rgbTransformation, sourcePoint, *depthMm);
                remappedDepthPoint = sourceTransformation.projectPointTo(depthTransformation, sourcePoint, *depthMm);
                rgbStatus = formatPointStatus("RGB", *remappedRgbPoint, *depthMm);
                sourceStatus = formatPointStatus("Source", sourcePoint, *depthMm);
                depthStatus = formatPointStatus("Depth", *remappedDepthPoint, *depthMm);
            } catch(const std::runtime_error& exc) {
                rgbStatus = std::string("RGB projection failed: ") + exc.what();
                depthStatus = std::string("Depth projection failed: ") + exc.what();
            }
        }

        cv::Mat depthScaled;
        cv::convertScaleAbs(depthFrame->getFrame(), depthScaled, 0.05);

        cv::Mat depthColor;
        cv::applyColorMap(depthScaled, depthColor, cv::COLORMAP_JET);

        drawPoint(leftFrame, originalPoint, sourceStatus, cv::Scalar(0, 255, 0));
        drawPoint(rgbDisplay, remappedRgbPoint, rgbStatus, cv::Scalar(255, 255, 0));
        drawPoint(depthColor, remappedDepthPoint, depthStatus, cv::Scalar(0, 0, 255));

        cv::imshow("Depth", depthColor);
        cv::imshow(SOURCE_WINDOW, leftFrame);
        cv::imshow(RGB_WINDOW, rgbDisplay);

        const int key = cv::waitKey(1);
        if(key == 'q') {
            pipeline.stop();
            break;
        }
        if(key == 'c') {
            selectedPoint = std::nullopt;
        }
    }

    pipeline.stop();
    pipeline.wait();
    return 0;
}
