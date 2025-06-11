#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "depthai/depthai.hpp"

// Global variables for ROI selection
std::vector<cv::Point> startPoints;
cv::Rect roiRect;
std::pair<float, float> scaleFactors;

// Mouse callback function for ROI selection
void selectRoi(int event, int x, int y, int flags, void* userdata) {
    auto* camQIn = static_cast<std::shared_ptr<dai::InputQueue>*>(userdata);

    if(event == cv::EVENT_LBUTTONDOWN) {
        roiRect = cv::Rect();
        startPoints.clear();
        startPoints.push_back(cv::Point(x, y));
    } else if(event == cv::EVENT_MOUSEMOVE && !startPoints.empty()) {
        cv::Point start = startPoints[0];
        roiRect = cv::Rect(std::min(start.x, x), std::min(start.y, y), std::abs(x - start.x), std::abs(y - start.y));
    } else if(event == cv::EVENT_LBUTTONUP && !startPoints.empty()) {
        cv::Point start = startPoints[0];
        roiRect = cv::Rect(std::min(start.x, x), std::min(start.y, y), std::abs(x - start.x), std::abs(y - start.y));

        // Scale ROI to original resolution
        cv::Rect roiRectScaled(static_cast<int>(roiRect.x * scaleFactors.first),
                               static_cast<int>(roiRect.y * scaleFactors.second),
                               static_cast<int>(roiRect.width * scaleFactors.first),
                               static_cast<int>(roiRect.height * scaleFactors.second));

        std::cout << "ROI selected: " << roiRect << std::endl;
        std::cout << "Scaled ROI selected: " << roiRectScaled << ". Setting exposure and focus to this region." << std::endl;

        // Create and send control message
        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setAutoExposureRegion(roiRectScaled.x, roiRectScaled.y, roiRectScaled.width, roiRectScaled.height);
        ctrl->setAutoFocusRegion(roiRectScaled.x, roiRectScaled.y, roiRectScaled.width, roiRectScaled.height);
        (*camQIn)->send(ctrl);

        startPoints.clear();
    }
}

int main() {
    // Create device
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    // Create pipeline
    dai::Pipeline pipeline(device);

    // Create nodes
    auto cam = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto camQIn = cam->inputControl.createInputQueue();
    auto streamQ = cam->requestOutput(std::make_pair(1920, 1080))->createOutputQueue();

    // Start pipeline
    pipeline.start();

    // Create window and set mouse callback
    cv::namedWindow("video");
    cv::setMouseCallback("video", selectRoi, &camQIn);

    while(true) {
        auto imgHd = streamQ->get<dai::ImgFrame>();
        if(imgHd == nullptr) continue;

        // Calculate scale factors if not set
        if(scaleFactors.first == 0.0f) {
            auto sourceSize = imgHd->transformation.getSourceSize();
            auto targetSize = imgHd->transformation.getSize();
            scaleFactors = std::make_pair(static_cast<float>(sourceSize.first) / targetSize.first, static_cast<float>(sourceSize.second) / targetSize.second);
            std::cout << "Source size: " << sourceSize.first << "x" << sourceSize.second << std::endl;
            std::cout << "Target size: " << targetSize.first << "x" << targetSize.second << std::endl;
        }

        cv::Mat frame = imgHd->getCvFrame();

        // Draw ROI rectangle if it exists
        if(roiRect.width > 0 && roiRect.height > 0) {
            cv::rectangle(frame, roiRect, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("video", frame);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}