#include <atomic>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

const std::string DISTORTED_WINDOW = "CAM_A distorted 640x480";
const std::string UNDISTORTED_WINDOW = "CAM_A undistorted 1000x400";
const std::pair<uint32_t, uint32_t> DISTORTED_SIZE = {640, 480};
const std::pair<uint32_t, uint32_t> UNDISTORTED_SIZE = {1000, 400};

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

void addStatusLines(cv::Mat& frame, const std::vector<std::string>& lines, const cv::Scalar& color = cv::Scalar(255, 0, 255)) {
    for(size_t index = 0; index < lines.size(); ++index) {
        cv::putText(frame,
                    lines[index],
                    cv::Point(10, 28 + static_cast<int>(index) * 24),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.65,
                    color,
                    2,
                    cv::LINE_AA);
    }
}

bool isInsideFrame(const dai::Point2f& point, const cv::Mat& frame) {
    return 0 <= point.x && point.x < frame.cols && 0 <= point.y && point.y < frame.rows;
}

void drawPoint(cv::Mat& frame, const std::optional<dai::Point2f>& point, const cv::Scalar& color) {
    if(!point.has_value() || !isInsideFrame(*point, frame)) {
        return;
    }

    const cv::Point intPoint(static_cast<int>(std::lround(point->x)), static_cast<int>(std::lround(point->y)));
    if(intPoint.x < 0 || intPoint.x >= frame.cols || intPoint.y < 0 || intPoint.y >= frame.rows) {
        return;
    }

    cv::drawMarker(frame, intPoint, color, cv::MARKER_CROSS, 16, 2);
    cv::circle(frame, intPoint, 6, color, 1);
}

std::string formatPointStatus(const std::string& prefix, const dai::Point2f& point) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << prefix << ": (" << point.x << ", " << point.y << ")";
    return stream.str();
}

int main() {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    dai::Pipeline pipeline;

    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* distorted = camera->requestOutput(DISTORTED_SIZE, std::nullopt, dai::ImgResizeMode::CROP, std::nullopt, false);
    auto* undistorted = camera->requestOutput(UNDISTORTED_SIZE, std::nullopt, dai::ImgResizeMode::CROP, std::nullopt, true);

    auto distQueue = distorted->createOutputQueue();
    auto undistQueue = undistorted->createOutputQueue();

    cv::namedWindow(DISTORTED_WINDOW);
    cv::namedWindow(UNDISTORTED_WINDOW);
    cv::setMouseCallback(DISTORTED_WINDOW, onLeftClick);

    std::cout << "Left click in the distorted CAM_A window to remap a point to the undistorted output." << std::endl;
    std::cout << "Press 'c' to clear the point and 'q' to quit." << std::endl;

    pipeline.start();
    while(pipeline.isRunning() && !quitEvent) {
        auto distortedFrame = distQueue->get<dai::ImgFrame>();
        auto undistortedFrame = undistQueue->get<dai::ImgFrame>();

        if(distortedFrame == nullptr || undistortedFrame == nullptr) {
            continue;
        }

        if(!distortedFrame->validateTransformations() || !undistortedFrame->validateTransformations()) {
            std::cerr << "Invalid transformations!" << std::endl;
            continue;
        }

        auto& distortedTransform = distortedFrame->getTransformation();
        auto& undistortedTransform = undistortedFrame->getTransformation();

        cv::Mat distortedView = toColorFrame(distortedFrame->getCvFrame());
        cv::Mat undistortedView = toColorFrame(undistortedFrame->getCvFrame());

        std::optional<dai::Point2f> sourcePoint = std::nullopt;
        std::optional<dai::Point2f> remappedPoint = std::nullopt;
        std::string sourceStatus = "Click in this window to select a point";
        std::string targetStatus = "Waiting for a selected point";

        if(selectedPoint.has_value()) {
            sourcePoint = dai::Point2f(static_cast<float>(selectedPoint->x), static_cast<float>(selectedPoint->y));
            remappedPoint = distortedTransform.remapPointTo(undistortedTransform, *sourcePoint);
            sourceStatus = formatPointStatus("Source", *sourcePoint);
            targetStatus = formatPointStatus("Remapped", *remappedPoint);
            if(!isInsideFrame(*remappedPoint, undistortedView)) {
                targetStatus += " outside target frame";
            }
        }

        drawPoint(distortedView, sourcePoint, cv::Scalar(0, 255, 0));
        drawPoint(undistortedView, remappedPoint, cv::Scalar(0, 255, 255));

        addStatusLines(distortedView, {"Distorted CAM_A output", sourceStatus, "Press c to clear, q to quit"});
        addStatusLines(undistortedView,
                       {"Undistorted CAM_A output",
                        targetStatus,
                        "Target size: " + std::to_string(UNDISTORTED_SIZE.first) + "x" + std::to_string(UNDISTORTED_SIZE.second)});

        cv::imshow(DISTORTED_WINDOW, distortedView);
        cv::imshow(UNDISTORTED_WINDOW, undistortedView);

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
