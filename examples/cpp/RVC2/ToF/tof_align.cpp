#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

// Constants from the Python script
constexpr float FPS = 30.0f;
const dai::CameraBoardSocket RGB_SOCKET = dai::CameraBoardSocket::CAM_C;
const dai::CameraBoardSocket TOF_SOCKET = dai::CameraBoardSocket::CAM_A;
const cv::Size SIZE(640, 400);

// FPSCounter class, similar to the one in the Python script
class FPSCounter {
   public:
    void tick() {
        auto now = std::chrono::steady_clock::now();
        frameTimes.push_back(now);
        // Keep the last 100 timestamps, similar to the Python example
        while(frameTimes.size() > 100) {
            frameTimes.pop_front();
        }
    }

    double getFps() {
        if(frameTimes.size() <= 1) {
            return 0.0;
        }
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(frameTimes.back() - frameTimes.front()).count();
        return (static_cast<double>(frameTimes.size()) - 1.0) / duration;
    }

   private:
    std::deque<std::chrono::steady_clock::time_point> frameTimes;
};

cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    // -----------------------------------------------------------------------
    // 1.  Basic checks & convert to CV_32F
    // -----------------------------------------------------------------------
    if(frameDepth.empty() || frameDepth.channels() != 1) return cv::Mat::zeros(frameDepth.size(), CV_8UC3);

    cv::Mat depth32f;
    frameDepth.convertTo(depth32f, CV_32F);  // safe for any input type

    // -----------------------------------------------------------------------
    // 2.  Build mask of valid (non-zero) pixels
    // -----------------------------------------------------------------------
    const cv::Mat nonZeroMask = depth32f != 0.0f;
    const int nz = cv::countNonZero(nonZeroMask);
    if(nz == 0) return cv::Mat::zeros(frameDepth.size(), CV_8UC3);

    // -----------------------------------------------------------------------
    // 3.  3 % / 95 % percentiles (identical to Python version)
    // -----------------------------------------------------------------------
    std::vector<float> values;
    values.reserve(nz);
    for(int r = 0; r < depth32f.rows; ++r) {
        const float* d = depth32f.ptr<float>(r);
        const uchar* m = nonZeroMask.ptr<uchar>(r);
        for(int c = 0; c < depth32f.cols; ++c)
            if(m[c]) values.push_back(d[c]);
    }

    std::sort(values.begin(), values.end());
    auto pct = [&](double p) {
        size_t idx = static_cast<size_t>(std::round((p / 100.0) * (values.size() - 1)));
        return values[idx];
    };

    const float minDepth = pct(3.0);
    const float maxDepth = pct(95.0);

    // -----------------------------------------------------------------------
    // 4.  Logarithm (zeros replaced by minDepth to avoid -inf)
    // -----------------------------------------------------------------------
    cv::Mat logDepth;
    depth32f.copyTo(logDepth);
    logDepth.setTo(minDepth, ~nonZeroMask);  // overwrite zeros
    cv::log(logDepth, logDepth);

    const float logMinDepth = std::log(minDepth);
    const float logMaxDepth = std::log(maxDepth);

    // -----------------------------------------------------------------------
    // 5.  Clip & linearly scale to [0,255]  (same as np.interp)
    // -----------------------------------------------------------------------
    cv::min(logDepth, logMaxDepth, logDepth);
    cv::max(logDepth, logMinDepth, logDepth);
    logDepth = (logDepth - logMinDepth) * (255.0f / (logMaxDepth - logMinDepth));

    cv::Mat depth8U;
    logDepth.convertTo(depth8U, CV_8U);

    // -----------------------------------------------------------------------
    // 6.  Colour map + set invalid pixels to black
    // -----------------------------------------------------------------------
    cv::Mat depthFrameColor;
    cv::applyColorMap(depth8U, depthFrameColor, cv::COLORMAP_JET);
    depthFrameColor.setTo(cv::Scalar::all(0), ~nonZeroMask);

    return depthFrameColor;
}

// Global variables for blending weights
float rgbWeight = 0.4f;
float depthWeight = 0.6f;

// Callback function for the trackbar
void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<float>(percentRgb) / 100.0f;
    depthWeight = 1.0f - rgbWeight;
}

int main() {
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::Camera>();
    auto tof = pipeline.create<dai::node::ToF>();
    auto sync = pipeline.create<dai::node::Sync>();
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);

    camRgb->build(RGB_SOCKET);
    tof->build(TOF_SOCKET);

    // Set sync threshold
    sync->setSyncThreshold(std::chrono::milliseconds(static_cast<uint32_t>(500 / FPS)));
    sync->setRunOnHost(true);

    // Linking
    auto cameraOutput = camRgb->requestOutput(std::make_pair(SIZE.width, SIZE.height), std::nullopt, dai::ImgResizeMode::CROP, FPS, true);

    cameraOutput->link(sync->inputs["rgb"]);
    tof->depth.link(align->input);
    align->outputAligned.link(sync->inputs["depth_aligned"]);
    sync->inputs["rgb"].setBlocking(false);
    cameraOutput->link(align->inputAlignTo);
    auto syncQueue = sync->out.createOutputQueue();

    // Start the pipeline
    pipeline.start();

    // Configure windows and trackbar
    const std::string rgbDepthWindowName = "rgb-depth";
    cv::namedWindow(rgbDepthWindowName);
    cv::createTrackbar("RGB Weight %", rgbDepthWindowName, nullptr, 100, updateBlendWeights);
    cv::setTrackbarPos("RGB Weight %", rgbDepthWindowName, static_cast<int>(rgbWeight * 100));

    FPSCounter fpsCounter;

    while(true) {
        auto messageGroup = syncQueue->get<dai::MessageGroup>();
        if(messageGroup == nullptr) continue;

        fpsCounter.tick();

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        if(frameRgb && frameDepth) {
            cv::Mat cvFrame = frameRgb->getCvFrame();
            cv::Mat alignedDepthColorized = colorizeDepth(frameDepth->getFrame());

            // Add FPS text to the depth frame
            std::string fpsText = "FPS: " + std::to_string(fpsCounter.getFps());
            cv::putText(alignedDepthColorized, fpsText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
            cv::imshow("depth", alignedDepthColorized);

            // Blend the RGB and depth frames
            cv::Mat blended;
            cv::addWeighted(cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0, blended);
            cv::imshow(rgbDepthWindowName, blended);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 27) {  // 'q' or ESC
            break;
        }
    }

    return 0;
}
