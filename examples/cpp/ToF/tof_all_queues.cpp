#include <cmath>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>

#include "depthai/depthai.hpp"

cv::Mat colorizeDepth(const cv::Mat& frame, float minDepth, float maxDepth) {
    cv::Mat depth32f;
    frame.convertTo(depth32f, CV_32F);

    cv::Mat invalidMask = depth32f == 0.0f;

    try {
        cv::Mat logDepth = depth32f + 1e-6f;
        cv::log(logDepth, logDepth);
        logDepth.setTo(0.0f, invalidMask);

        const float logMinDepth = std::log(minDepth + 1e-6f);
        const float logMaxDepth = std::log(maxDepth + 1e-6f);

        cv::min(logDepth, logMaxDepth, logDepth);
        cv::max(logDepth, logMinDepth, logDepth);

        cv::Mat validMask = invalidMask == 0;
        double validMin = 0.0;
        double validMax = 0.0;
        cv::minMaxLoc(logDepth, &validMin, &validMax, nullptr, nullptr, validMask);

        if(validMax <= validMin) {
            return cv::Mat::zeros(frame.size(), CV_8UC3);
        }

        cv::Mat colored;
        logDepth.convertTo(colored, CV_8U, 255.0 / (validMax - validMin), -validMin * 255.0 / (validMax - validMin));
        cv::applyColorMap(colored, colored, cv::COLORMAP_JET);
        colored.setTo(cv::Scalar::all(0), invalidMask);
        return colored;
    } catch(const cv::Exception&) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }
}

cv::Mat normalizeFrame(const cv::Mat& frame) {
    cv::Mat normalized;
    cv::normalize(frame, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    return normalized;
}

int main() {
    dai::Pipeline pipeline;

    // show depth in range 0.1m - 7m
    constexpr float minDepth = 100.0f;
    constexpr float maxDepth = 7000.0f;

    // choose one of profiles LOW_RANGE / MID_RANGE / HIGH_RANGE
    auto profile = dai::ToFConfig::Profile::MID_RANGE;

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, profile);

    std::map<std::string, std::shared_ptr<dai::MessageQueue>> outputQueues = {
        {"depth", tof->depth.createOutputQueue(1, false)},
        {"amplitude", tof->amplitude.createOutputQueue(1, false)},
        {"intensity", tof->intensity.createOutputQueue(1, false)},
        // {"rawDepth", tof->rawDepth.createOutputQueue(1, false)},  // not supported on RVC4
        // {"confidence", tof->confidence.createOutputQueue(1, false)},  // not supported on RVC2
    };

    pipeline.start();
    while(pipeline.isRunning()) {
        for(const auto& [name, queue] : outputQueues) {
            auto frame = queue->tryGet<dai::ImgFrame>();
            if(frame == nullptr) {
                continue;
            }

            cv::Mat displayFrame;

            if(name == "depth" || name == "rawDepth") {
                displayFrame = colorizeDepth(frame->getCvFrame(), minDepth, maxDepth);
            } else {
                displayFrame = normalizeFrame(frame->getCvFrame());
            }

            cv::imshow(name, displayFrame);
        }

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
