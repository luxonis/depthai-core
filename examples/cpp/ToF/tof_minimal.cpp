#include <cmath>
#include <opencv2/opencv.hpp>

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

int main() {
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Show depth in range 0.5 m to 10 m.
    constexpr float minDepth = 500.0f;
    constexpr float maxDepth = 10000.0f;

    // Choose one of the profiles: LOW_RANGE, MID_RANGE, or HIGH_RANGE.
    auto profile = dai::ToFConfig::Profile::MID_RANGE;

    auto tof = pipeline.create<dai::node::ToF>()->build(dai::CameraBoardSocket::AUTO, profile);

    auto depthOutputQueue = tof->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthOutputQueue->get<dai::ImgFrame>();
        cv::imshow("depth", colorizeDepth(depth->getCvFrame(), minDepth, maxDepth));

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
