#include <array>
#include <memory>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

namespace {

constexpr float FPS = 30.0f;
constexpr const char* WINDOW = "VD55H1 controls";

std::shared_ptr<dai::ToFConfig> configFromTrackbars(const std::array<int, 10>& value) {
    auto config = std::make_shared<dai::ToFConfig>();
    auto& vd55h1 = config->vd55h1;
    vd55h1.phaseUnwrapErrorThreshold = value[0];
    vd55h1.enableBilateralFilter = value[1] != 0;
    vd55h1.bilateralStdFactor = value[2] / 100.0f;
    vd55h1.bilateralKernelSize = value[3];
    vd55h1.enableTemporalNoiseReduction = value[4] != 0;
    vd55h1.temporalNoiseReductionMaxGain = value[5];
    vd55h1.temporalNoiseReductionStdFactor = value[6] / 100.0f;
    vd55h1.enableFlyingPixelFilter = value[7] != 0;
    vd55h1.flyingPixelDepthThreshold = value[8];
    vd55h1.flyingPixelMinDepthOccurrence = value[9] / 100.0f;
    return config;
}

}  // namespace

int main() {
    dai::Pipeline pipeline;
    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, FPS);
    auto depthQueue = tof->depth.createOutputQueue(1, false);
    auto configQueue = tof->tofBaseInputConfig.createInputQueue();

    cv::namedWindow(WINDOW);
    std::array<int, 10> value = {192, 1, 205, 5, 1, 27, 82, 1, 101, 1356};
    cv::createTrackbar("unwrap threshold", WINDOW, &value[0], 500);
    cv::createTrackbar("bilateral", WINDOW, &value[1], 1);
    cv::createTrackbar("bilateral std x100", WINDOW, &value[2], 1000);
    cv::createTrackbar("bilateral kernel", WINDOW, &value[3], 15);
    cv::createTrackbar("temporal NR", WINDOW, &value[4], 1);
    cv::createTrackbar("TNR max gain", WINDOW, &value[5], 100);
    cv::createTrackbar("TNR std x100", WINDOW, &value[6], 500);
    cv::createTrackbar("flying pixel", WINDOW, &value[7], 1);
    cv::createTrackbar("FP depth threshold", WINDOW, &value[8], 1000);
    cv::createTrackbar("FP min occurrence x100", WINDOW, &value[9], 5000);

    pipeline.start();
    std::array<int, 10> previous{};
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
