#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/depthai.hpp"

// Define constants from the Python script
constexpr float FPS = 10.0f;
const dai::CameraBoardSocket RGB_SOCKET = dai::CameraBoardSocket::CAM_A;
const dai::CameraBoardSocket LEFT_SOCKET = dai::CameraBoardSocket::CAM_B;
const dai::CameraBoardSocket RIGHT_SOCKET = dai::CameraBoardSocket::CAM_C;

// FPS Counter class to calculate and display frames per second
class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        // Keep the last 10 timestamps, same as the Python version
        if(frameTimes.size() > 10) {
            frameTimes.pop_front();
        }
    }

    double getFps() const {
        if(frameTimes.size() <= 1) {
            return 0.0;
        }
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(frameTimes.back() - frameTimes.front()).count();
        return (static_cast<double>(frameTimes.size()) - 1.0) / duration;
    }

   private:
    std::deque<std::chrono::steady_clock::time_point> frameTimes;
};

// Function to colorize a depth frame for visualization
cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    if(frameDepth.empty() || frameDepth.channels() != 1) {
        return cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }

    cv::Mat depth32f;
    frameDepth.convertTo(depth32f, CV_32F);

    const cv::Mat nonZeroMask = depth32f != 0.0f;
    const int nz = cv::countNonZero(nonZeroMask);
    if(nz == 0) {
        return cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }

    // Extract non-zero depth values to calculate percentiles
    std::vector<float> values;
    values.reserve(nz);
    for(int r = 0; r < depth32f.rows; ++r) {
        const float* d = depth32f.ptr<float>(r);
        const uchar* m = nonZeroMask.ptr<uchar>(r);
        for(int c = 0; c < depth32f.cols; ++c) {
            if(m[c]) {
                values.push_back(d[c]);
            }
        }
    }

    std::sort(values.begin(), values.end());

    // Lambda to calculate percentile
    auto pct = [&](double p) {
        if(values.empty()) return 0.0f;
        size_t idx = static_cast<size_t>(std::round((p / 100.0) * (values.size() - 1)));
        return values[idx];
    };

    const float minDepth = pct(3.0);
    const float maxDepth = pct(95.0);

    // Apply logarithmic scaling
    cv::Mat logDepth;
    depth32f.copyTo(logDepth);
    logDepth.setTo(minDepth, ~nonZeroMask);  // Replace zeros to avoid log(0)
    cv::log(logDepth, logDepth);

    const float logMinDepth = std::log(minDepth);
    const float logMaxDepth = std::log(maxDepth);

    // Clip and linearly scale to the [0, 255] range
    cv::min(logDepth, logMaxDepth, logDepth);
    cv::max(logDepth, logMinDepth, logDepth);
    if(logMaxDepth > logMinDepth) {
        logDepth = (logDepth - logMinDepth) * (255.0f / (logMaxDepth - logMinDepth));
    }

    cv::Mat depth8U;
    logDepth.convertTo(depth8U, CV_8U);

    // Apply color map and set invalid pixels to black
    cv::Mat depthFrameColor;
    cv::applyColorMap(depth8U, depthFrameColor, cv::COLORMAP_JET);
    depthFrameColor.setTo(cv::Scalar::all(0), ~nonZeroMask);

    return depthFrameColor;
}

// Global variables for blending weights, controlled by the trackbar
float rgbWeight = 0.4f;
float depthWeight = 0.6f;

// Callback function for the OpenCV trackbar
void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<float>(percentRgb) / 100.0f;
    depthWeight = 1.0f - rgbWeight;
}

int main() {
    // Create the DepthAI pipeline
    dai::Pipeline pipeline;

    // --- Define pipeline nodes ---
    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->build(RGB_SOCKET);

    auto left = pipeline.create<dai::node::Camera>();
    left->build(LEFT_SOCKET);

    auto right = pipeline.create<dai::node::Camera>();
    right->build(RIGHT_SOCKET);

    auto stereo = pipeline.create<dai::node::NeuralDepth>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto align = pipeline.create<dai::node::ImageAlign>();

    // --- Configure nodes ---
    sync->setSyncThreshold(std::chrono::milliseconds(static_cast<long long>(1000.0 / (2.0 * FPS))));

    // auto* rgbOut = camRgb->requestOutput(std::make_pair(1280, 960), std::nullopt, std::nullopt, FPS, true);
    auto* rgbOut = camRgb->requestOutput(std::make_pair(1280, 960), std::nullopt, dai::ImgResizeMode::CROP, FPS, true);
    auto* leftOut = left->requestFullResolutionOutput(std::nullopt, FPS);
    auto* rightOut = right->requestFullResolutionOutput(std::nullopt, FPS);

    stereo->build(*leftOut, *rightOut, dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);

    // --- Link pipeline nodes ---
    stereo->depth.link(align->input);
    rgbOut->link(align->inputAlignTo);
    rgbOut->link(sync->inputs["rgb"]);
    align->outputAligned.link(sync->inputs["depth_aligned"]);

    // Create an output queue for the synchronized frames
    auto queue = sync->out.createOutputQueue();

    // Start the pipeline
    pipeline.start();

    // --- Setup OpenCV windows and trackbar ---
    const std::string windowName = "rgb-depth";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1280, 720);
    cv::createTrackbar("RGB Weight %", windowName, nullptr, 100, updateBlendWeights);
    cv::setTrackbarPos("RGB Weight %", windowName, static_cast<int>(rgbWeight * 100));

    FPSCounter fpsCounter;

    while(true) {
        // Get the synchronized message group from the queue
        auto messageGroup = queue->get<dai::MessageGroup>();
        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        if(frameRgb && frameDepth) {
            cv::Mat cvFrame = frameRgb->getCvFrame();

            // Colorize the aligned depth frame for visualization
            cv::Mat alignedDepthColorized = colorizeDepth(frameDepth->getFrame());
            cv::imshow("Depth aligned", alignedDepthColorized);

            // Blend the RGB and colorized depth frames
            cv::Mat blended;
            cv::addWeighted(cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0, blended);

            // Add FPS text to the blended image
            char fpsStr[20];
            snprintf(fpsStr, sizeof(fpsStr), "FPS: %.2f", fpsCounter.getFps());
            cv::putText(blended, fpsStr, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

            // Show the final blended result
            cv::imshow(windowName, blended);
        }

        // Check for 'q' key press to exit
        int key = cv::waitKey(1);
        if(key == 'q' || key == 27) {  // 'q' or ESC
            break;
        }
    }

    return 0;
}
