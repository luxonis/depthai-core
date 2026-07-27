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

const std::string DEPTH_WINDOW = "Depth window (source)";
const std::string RGB_WINDOW = "RGB window (remapped)";

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
        return {std::nullopt, "Left click a point on the depth image"};
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

    // Pick a COLOR-capable socket for the RGB camera so it does not take the
    // socket the Depth node needs (e.g. the ToF sensor on ToF-capable devices,
    // where the default AUTO socket could otherwise grab the only ToF-capable
    // sensor).
    auto colorSocket = dai::CameraBoardSocket::CAM_A;
    for(const auto& features : pipeline.getDefaultDevice()->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    auto rgb = pipeline.create<dai::node::Camera>()->build(colorSocket);

    // The Depth node manages its own cameras and backend (stereo / ToF / neural)
    // internally, so this example stays device-agnostic. We click a point on the
    // depth image (its own sensor frame), read the depth there, and remap that 3D
    // point into the RGB sensor via projectPointTo. The (640, 400) size keeps a
    // stereo backend within its 1280-wide input limit (full sensor resolution
    // would exceed it on e.g. OAK-D-LR).
    auto depth = pipeline.create<dai::node::Depth>();
    depth->build(dai::node::Depth::Algorithm::AUTO, std::nullopt, std::make_pair(640u, 400u));

    auto rgbOut = rgb->requestOutput({720, 480}, std::nullopt, dai::ImgResizeMode::CROP, std::nullopt, false);
    auto rgbQueue = rgbOut->createOutputQueue();
    auto depthQueue = depth->depth().createOutputQueue();

    cv::namedWindow(DEPTH_WINDOW);
    cv::namedWindow(RGB_WINDOW);
    cv::setMouseCallback(DEPTH_WINDOW, onLeftClick);

    pipeline.start();
    while(pipeline.isRunning() && !quitEvent) {
        auto rgbFrame = rgbQueue->get<dai::ImgFrame>();
        auto depthFrame = depthQueue->get<dai::ImgFrame>();

        if(rgbFrame == nullptr || depthFrame == nullptr) {
            continue;
        }

        if(!rgbFrame->validateTransformations() || !depthFrame->validateTransformations()) {
            std::cerr << "Invalid transformations!" << std::endl;
            continue;
        }

        auto& depthTransformation = depthFrame->getTransformation();
        auto& rgbTransformation = rgbFrame->getTransformation();

        cv::Mat rgbDisplay = rgbFrame->getCvFrame();
        auto [depthMm, depthStatus] = sampleDepth(selectedPoint, depthFrame);

        std::optional<dai::Point2f> remappedRgbPoint = std::nullopt;
        std::string rgbStatus;

        std::optional<dai::Point2f> sourcePoint = std::nullopt;
        if(selectedPoint.has_value() && depthMm.has_value()) {
            dai::Point2f source(static_cast<float>(selectedPoint->x), static_cast<float>(selectedPoint->y));
            sourcePoint = source;

            try {
                remappedRgbPoint = depthTransformation.projectPointTo(rgbTransformation, source, *depthMm);
                depthStatus = formatPointStatus("Depth", source, *depthMm);
                rgbStatus = formatPointStatus("RGB", *remappedRgbPoint, *depthMm);
            } catch(const std::runtime_error& exc) {
                rgbStatus = std::string("RGB projection failed: ") + exc.what();
            }
        }

        cv::Mat depthScaled;
        cv::convertScaleAbs(depthFrame->getFrame(), depthScaled, 0.05);

        cv::Mat depthColor;
        cv::applyColorMap(depthScaled, depthColor, cv::COLORMAP_JET);

        drawPoint(depthColor, sourcePoint, depthStatus, cv::Scalar(0, 255, 0));
        drawPoint(rgbDisplay, remappedRgbPoint, rgbStatus, cv::Scalar(255, 255, 0));

        cv::imshow(DEPTH_WINDOW, depthColor);
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
