#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

constexpr float FPS = 25.0f;

const dai::CameraBoardSocket RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
const dai::CameraBoardSocket LEFT_SOCKET = dai::CameraBoardSocket::CAM_B;
const dai::CameraBoardSocket RIGHT_SOCKET = dai::CameraBoardSocket::CAM_C;

// FPS Counter class
class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        while(frameTimes.size() > 10) {
            frameTimes.pop_front();
        }
    }

    float getFps() {
        if(frameTimes.size() <= 1) return 0.0f;
        auto duration = std::chrono::duration_cast<std::chrono::duration<float>>(frameTimes.back() - frameTimes.front()).count();
        return (frameTimes.size() - 1) / duration;
    }

   private:
    std::deque<std::chrono::steady_clock::time_point> frameTimes;
};

// Depth colorization function from detection_network_remap.cpp
cv::Mat colorizeDepth(cv::Mat frameDepth) {
    try {
        // Early exit if no valid pixels
        if(cv::countNonZero(frameDepth) == 0) {
            return cv::Mat::zeros(frameDepth.rows, frameDepth.cols, CV_8UC3);
        }

        // Convert to float once
        cv::Mat frameDepthFloat;
        frameDepth.convertTo(frameDepthFloat, CV_32F);

        double minVal, maxVal;
        cv::minMaxLoc(frameDepthFloat, &minVal, &maxVal, nullptr, nullptr, frameDepthFloat > 0);

        // Take log in-place
        cv::log(frameDepthFloat, frameDepthFloat);
        float logMinDepth = std::log(minVal);
        float logMaxDepth = std::log(maxVal);

        frameDepthFloat = (frameDepthFloat - logMinDepth) * (255.0f / (logMaxDepth - logMinDepth));

        cv::Mat normalizedDepth;
        frameDepthFloat.convertTo(normalizedDepth, CV_8UC1);

        cv::Mat depthFrameColor;
        cv::applyColorMap(normalizedDepth, depthFrameColor, cv::COLORMAP_JET);

        // Mask invalid pixels
        depthFrameColor.setTo(0, frameDepth == 0);

        return depthFrameColor;

    } catch(const std::exception& e) {
        std::cerr << "Error in colorizeDepth: " << e.what() << std::endl;
        return cv::Mat::zeros(frameDepth.rows, frameDepth.cols, CV_8UC3);
    }
}

// Global blend weights
float rgbWeight = 0.4f;
float depthWeight = 0.6f;

// Trackbar callback
void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<float>(percentRgb) / 100.0f;
    depthWeight = 1.0f - rgbWeight;
}

int main() {
    dai::Pipeline pipeline;

    // Create and configure nodes
    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->build(RGB_SOCKET);
    auto left = pipeline.create<dai::node::Camera>();
    left->build(LEFT_SOCKET);
    auto right = pipeline.create<dai::node::Camera>();
    right->build(RIGHT_SOCKET);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto sync = pipeline.create<dai::node::Sync>();

    // Check if platform is RVC4 and create ImageAlign node if needed
    auto platform = pipeline.getDefaultDevice()->getPlatform();
    std::shared_ptr<dai::node::ImageAlign> align;
    if(platform == dai::Platform::RVC4) {
        align = pipeline.create<dai::node::ImageAlign>();
    }

    stereo->setExtendedDisparity(true);
    sync->setSyncThreshold(std::chrono::duration<int64_t, std::nano>(static_cast<int64_t>(1e9 / (2.0 * FPS))));

    // Configure outputs
    auto rgbOut = camRgb->requestOutput(std::make_pair(1280, 960), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS, true);
    auto leftOut = left->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS);
    auto rightOut = right->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS);

    // Link nodes
    rgbOut->link(sync->inputs["rgb"]);
    leftOut->link(stereo->left);
    rightOut->link(stereo->right);

    if(platform == dai::Platform::RVC4) {
        stereo->depth.link(align->input);
        rgbOut->link(align->inputAlignTo);
        align->outputAligned.link(sync->inputs["depth_aligned"]);
    } else {
        stereo->depth.link(sync->inputs["depth_aligned"]);
        rgbOut->link(stereo->inputAlignTo);
    }

    // Create output queue
    auto queue = sync->out.createOutputQueue();

    // Create and configure windows
    const std::string windowName = "rgb-depth";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::setTrackbarPos("RGB Weight %", windowName, static_cast<int>(rgbWeight * 100));

    FPSCounter fpsCounter;

    // Start pipeline
    pipeline.start();

    while(true) {
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        if(frameDepth != nullptr) {
            cv::Mat cvFrame = frameRgb->getCvFrame();

            // Colorize depth
            cv::Mat alignedDepthColorized = colorizeDepth(frameDepth->getFrame());
            cv::imshow("Depth aligned", alignedDepthColorized);

            // Convert grayscale to BGR if needed
            if(cvFrame.channels() == 1) {
                cv::cvtColor(cvFrame, cvFrame, cv::COLOR_GRAY2BGR);
            }

            // Blend frames
            cv::Mat blended;
            cv::addWeighted(cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0, blended);

            // Add FPS text
            cv::putText(blended, "FPS: " + std::to_string(fpsCounter.getFps()), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

            cv::imshow(windowName, blended);
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
