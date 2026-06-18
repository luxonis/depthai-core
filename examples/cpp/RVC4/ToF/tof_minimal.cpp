/**
 * Minimal RVC4 ToF example — auto-created Camera, IPP depth via unified ToF node.
 *
 * Requires OpenCV. Build with the depthai C++ examples target tof_minimal_rvc4.
 */

#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/common/ToFPreset.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/ToF.hpp"

namespace {

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

    std::vector<float> values;
    values.reserve(static_cast<size_t>(nz));
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
    auto pct = [&](double p) {
        const size_t idx = static_cast<size_t>(std::round((p / 100.0) * (values.size() - 1)));
        return values[idx];
    };

    const float minDepth = pct(3.0);
    const float maxDepth = pct(95.0);

    cv::Mat logDepth;
    depth32f.copyTo(logDepth);
    logDepth.setTo(minDepth, ~nonZeroMask);
    cv::log(logDepth, logDepth);

    const float logMinDepth = std::log(minDepth);
    const float logMaxDepth = std::log(maxDepth);

    cv::min(logDepth, logMaxDepth, logDepth);
    cv::max(logDepth, logMinDepth, logDepth);
    logDepth = (logDepth - logMinDepth) * (255.0f / (logMaxDepth - logMinDepth));

    cv::Mat depth8U;
    logDepth.convertTo(depth8U, CV_8U);

    cv::Mat depthFrameColor;
    cv::applyColorMap(depth8U, depthFrameColor, cv::COLORMAP_JET);
    depthFrameColor.setTo(cv::Scalar::all(0), ~nonZeroMask);
    return depthFrameColor;
}

}  // namespace

int main() {
    dai::Pipeline pipeline;

    auto tof = pipeline.create<dai::node::ToF>();

    dai::ToFBuildOptions options;
    options.boardSocket = dai::CameraBoardSocket::AUTO;
    options.fps = 10.f;  // VD55H1 capture mode fixed to F3_FULL internally
    options.preset = dai::ToFPreset::MID_RANGE;
    tof->build(options);

    const auto depthQueue = tof->depth.createOutputQueue();

    pipeline.start();

    if(tof->getCamera()) {
        const auto [outW, outH] = tof->getOutputResolution();
        const auto [rawW, rawH] = tof->getRawResolution();
        std::cout << "Auto-created Camera on socket " << static_cast<int>(tof->getBoardSocket()) << std::endl;
        std::cout << "Output resolution: " << outW << "x" << outH << ", raw superframe: " << rawW << "x" << rawH << std::endl;
    }

    while(pipeline.isRunning()) {
        const auto depth = depthQueue->tryGet<dai::ImgFrame>();
        if(depth) {
            cv::imshow("depth (IPP)", colorizeDepth(depth->getCvFrame()));
        }
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
