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
// Level 0 disables phase unwrapping.
std::shared_ptr<dai::ToFConfig> configFromTrackbars(const std::array<int, 2>& value, bool isS5K63D) {
    auto config = std::make_shared<dai::ToFConfig>();
    // S5K63D is a type alias; the accessor returns the same settings as s5k33d.
    auto& params = isS5K63D ? config->s5k63d() : config->s5k33d;
    params.phaseUnwrappingLevel = value[0];
    params.phaseUnwrapErrorThreshold = value[1];
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
    std::array<int, 2> value = {4, 75};  // MID_RANGE threshold.
    cv::createTrackbar("unwrap level", WINDOW, &value[0], 5);
    cv::createTrackbar("unwrap threshold mm", WINDOW, &value[1], 500);

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
