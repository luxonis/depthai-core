#include <algorithm>
#include <array>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>

#include "depthai/depthai.hpp"

namespace {

constexpr float FPS = 30.0f;
constexpr const char* WINDOW = "S5K33D / S5K63D controls";
// Example tuning ranges: unwrap level 0-5 and threshold 0-500 mm.
// Toggle sliders use 0/1. Level 0 also disables phase unwrapping.
// Burst mode reduces output FPS by four. Corrections use available calibration.
std::shared_ptr<dai::ToFConfig> configFromTrackbars(const std::array<int, 9>& value, bool isS5K63D) {
    auto config = std::make_shared<dai::ToFConfig>();
    // S5K63D is a type alias; the accessor returns the same settings as s5k33d.
    auto& params = isS5K63D ? config->s5k63d() : config->s5k33d;
    params.phaseUnwrappingLevel = value[0];
    params.phaseUnwrapErrorThreshold = value[1];
    params.enablePhaseShuffleTemporalFilter = value[2] != 0;
    params.enableBurstMode = value[3] != 0;
    params.enableFPPNCorrection = value[4] != 0;
    params.enableOpticalCorrection = value[5] != 0;
    params.enableTemperatureCorrection = value[6] != 0;
    params.enableWiggleCorrection = value[7] != 0;
    params.enablePhaseUnwrapping = value[8] != 0;
    return config;
}

}  // namespace

int main() {
    dai::Pipeline pipeline;
    const auto cameras = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
    const auto sensor =
        std::find_if(cameras.begin(), cameras.end(), [](const auto& camera) { return camera.sensorName == "S5K33D" || camera.sensorName == "S5K63D"; });
    if(sensor == cameras.end()) {
        std::ostringstream message;
        message << "This example requires an S5K33D or S5K63D ToF sensor. Found sensors: ";
        for(std::size_t i = 0; i < cameras.size(); ++i) {
            if(i != 0) message << ", ";
            message << cameras[i].sensorName;
        }
        if(cameras.empty()) message << "none";
        throw std::runtime_error(message.str());
    }
    auto tof = pipeline.create<dai::node::ToF>()->build(sensor->socket, dai::ToFConfig::Profile::MID_RANGE, FPS);
    const bool isS5K63D = sensor->sensorName == "S5K63D";
    auto depthQueue = tof->depth.createOutputQueue(1, false);
    auto configQueue = tof->tofBaseInputConfig.createInputQueue();

    cv::namedWindow(WINDOW);
    std::array<int, 9> value = {4, 75, 1, 0, 1, 1, 1, 1, 1};  // MID_RANGE threshold.
    cv::createTrackbar("unwrap level", WINDOW, &value[0], 5);
    cv::createTrackbar("unwrap threshold mm", WINDOW, &value[1], 500);
    cv::createTrackbar("phase shuffle", WINDOW, &value[2], 1);
    cv::createTrackbar("burst mode", WINDOW, &value[3], 1);
    cv::createTrackbar("FPPN correction", WINDOW, &value[4], 1);
    cv::createTrackbar("optical correction", WINDOW, &value[5], 1);
    cv::createTrackbar("temperature correction", WINDOW, &value[6], 1);
    cv::createTrackbar("wiggle correction", WINDOW, &value[7], 1);
    cv::createTrackbar("phase unwrapping", WINDOW, &value[8], 1);

    *tof->tofBaseNode.initialConfig = *configFromTrackbars(value, isS5K63D);
    pipeline.start();
    auto previous = value;
    while(pipeline.isRunning()) {
        if(value != previous) {
            configQueue->send(configFromTrackbars(value, isS5K63D));
            previous = value;
        }

        if(const auto frame = depthQueue->tryGet<dai::ImgFrame>()) {
            cv::imshow("ToF depth", dai::utility::colorizeDepthFrame(*frame, 100.0f, 7000.0f, cv::COLORMAP_JET, true).getCvFrame());
        }
        if(cv::waitKey(1) == 'q') break;
    }
    cv::destroyAllWindows();
    return 0;
}
