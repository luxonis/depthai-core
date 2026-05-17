#include <opencv2/opencv.hpp>

#include <iostream>

#include "depthai/depthai.hpp"

namespace {

cv::Mat colorizeDepthMm(const cv::Mat& frame) {
    if(frame.empty() || frame.channels() != 1) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }

    const cv::Mat validMask = frame > 0;
    if(cv::countNonZero(validMask) == 0) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }

    double maxValue = 0.0;
    cv::minMaxLoc(frame, nullptr, &maxValue, nullptr, nullptr, validMask);

    cv::Mat normalized(frame.size(), CV_8U, cv::Scalar(0));
    if(maxValue > 0.0) {
        cv::Mat scaled;
        frame.convertTo(scaled, CV_32F, 255.0 / maxValue);
        scaled.convertTo(normalized, CV_8U);
    }

    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
    colorized.setTo(cv::Scalar::all(0), ~validMask);
    return colorized;
}

cv::Mat colorizeConfidence(const cv::Mat& frame) {
    if(frame.empty() || frame.channels() != 1) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }

    cv::Mat normalized;
    if(frame.depth() == CV_16U) {
        double maxValue = 0.0;
        cv::minMaxLoc(frame, nullptr, &maxValue);
        if(maxValue <= 0.0) {
            return cv::Mat::zeros(frame.size(), CV_8UC3);
        }
        cv::Mat scaled;
        frame.convertTo(scaled, CV_32F, 255.0 / maxValue);
        scaled.convertTo(normalized, CV_8U);
    } else {
        frame.convertTo(normalized, CV_8U);
    }

    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
    return colorized;
}

}  // namespace

int main() {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        std::cerr << "Connect a device (host-only pipeline cannot use Depth)." << std::endl;
        return 1;
    }

    const auto stereoPairs = device->getStereoPairs();
    if(stereoPairs.empty()) {
        std::cerr << "This device has no stereo pair; Depth cannot run." << std::endl;
        return 1;
    }

    const auto stereoPair = stereoPairs[0];

    pipeline.create<dai::node::Camera>()->build(stereoPair.left, std::make_pair(1280u, 800u), 30.0f);
    pipeline.create<dai::node::Camera>()->build(stereoPair.right, std::make_pair(1280u, 800u), 30.0f);

    auto depthNode = pipeline.create<dai::node::Depth>();

    auto depthQueue = depthNode->depth().createOutputQueue();
    auto confidenceQueue = depthNode->confidence().createOutputQueue();

    pipeline.build();
    pipeline.start();

    while(pipeline.isRunning()) {
        auto depthFrame = depthQueue->get<dai::ImgFrame>();
        auto confidenceFrame = confidenceQueue->get<dai::ImgFrame>();

        if(depthFrame != nullptr) {
            cv::imshow("depth (user cameras)", colorizeDepthMm(depthFrame->getFrame()));
        }
        if(confidenceFrame != nullptr) {
            cv::imshow("confidence", colorizeConfidence(confidenceFrame->getFrame()));
        }

        const int key = cv::waitKey(1);
        if(key == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
