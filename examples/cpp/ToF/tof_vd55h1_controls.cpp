#include <algorithm>
#include <array>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>

#include "depthai/depthai.hpp"

namespace {

constexpr float FPS = 30.0f;
constexpr const char* WINDOW = "VD55H1 controls";
constexpr float UNWRAP_THRESHOLD_MM = 192.0f;  // IPP range [0, 10000]; startup only.

// Example slider ranges, not IPP-supported limits: bilateral std 0.01-10,
// TNR gain 1-100 frames, TNR std 0-5, FP depth 1-1000 mm, FP occurrence 0-24.99.
std::shared_ptr<dai::ToFConfig> configFromTrackbars(const std::array<int, 8>& value) {
    auto config = std::make_shared<dai::ToFConfig>();
    config->vd55h1 = {};  // Send only runtime controls.
    auto& vd55h1 = config->vd55h1;
    vd55h1.enableBilateralFilter = value[0] != 0;
    vd55h1.bilateralStdFactor = std::max(1, value[1]) / 100.0f;
    vd55h1.enableTemporalNoiseReduction = value[2] != 0;
    vd55h1.temporalNoiseReductionMaxGain = std::max(1, value[3]);
    vd55h1.temporalNoiseReductionStdFactor = value[4] / 100.0f;
    vd55h1.enableFlyingPixelFilter = value[5] != 0;
    vd55h1.flyingPixelDepthThreshold = std::max(1, value[6]);
    vd55h1.flyingPixelMinDepthOccurrence = value[7] / 100.0f;
    return config;
}

}  // namespace

int main() {
    dai::Pipeline pipeline;
    const auto cameras = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
    const auto sensor = std::find_if(cameras.begin(), cameras.end(), [](const auto& camera) { return camera.sensorName == "VD55H1"; });
    if(sensor == cameras.end()) {
        std::ostringstream message;
        message << "This example requires a VD55H1 ToF sensor. Found sensors: ";
        for(std::size_t i = 0; i < cameras.size(); ++i) {
            if(i != 0) message << ", ";
            message << cameras[i].sensorName;
        }
        if(cameras.empty()) message << "none";
        throw std::runtime_error(message.str());
    }
    auto tof = pipeline.create<dai::node::ToF>()->build(sensor->socket, dai::ToFConfig::Profile::MID_RANGE, FPS);
    tof->tofBaseNode.initialConfig->vd55h1.phaseUnwrapErrorThreshold = UNWRAP_THRESHOLD_MM;
    auto depthQueue = tof->depth.createOutputQueue(1, false);
    auto configQueue = tof->tofBaseInputConfig.createInputQueue();

    cv::namedWindow(WINDOW);
    std::array<int, 8> value = {1, 205, 1, 27, 82, 1, 101, 1356};
    cv::createTrackbar("bilateral", WINDOW, &value[0], 1);
    cv::createTrackbar("bilateral std x100", WINDOW, &value[1], 1000);
    cv::createTrackbar("temporal NR", WINDOW, &value[2], 1);
    cv::createTrackbar("TNR max gain", WINDOW, &value[3], 100);
    cv::createTrackbar("TNR std x100", WINDOW, &value[4], 500);
    cv::createTrackbar("flying pixel", WINDOW, &value[5], 1);
    cv::createTrackbar("FP depth threshold", WINDOW, &value[6], 1000);
    cv::createTrackbar("FP min occurrence x100", WINDOW, &value[7], 2499);

    pipeline.start();
    std::array<int, 8> previous{};
    previous.fill(-1);
    while(pipeline.isRunning()) {
        if(value != previous) {
            configQueue->send(configFromTrackbars(value));
            previous = value;
        }

        if(const auto frame = depthQueue->tryGet<dai::ImgFrame>()) {
            cv::imshow("ToF depth", dai::utility::colorizeDepthFrame(*frame, 100.0f, 7000.0f, cv::COLORMAP_JET, true).getCvFrame());
        }
        if(cv::waitKey(1) == 'q') break;
    }
    return 0;
}
