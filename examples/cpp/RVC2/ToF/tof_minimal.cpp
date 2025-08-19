#include <iostream>
#include <opencv2/opencv.hpp>
#include <xtensor/containers/xadapt.hpp>
#include <xtensor/containers/xarray.hpp>

#include "depthai/depthai.hpp"

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

int main() {
    dai::Pipeline pipeline;

    // ToF node
    dai::CameraBoardSocket socket = dai::CameraBoardSocket::AUTO;
    dai::ImageFiltersPresetMode presetMode = dai::ImageFiltersPresetMode::TOF_MID_RANGE;
    std::shared_ptr<dai::node::ToF> tof = pipeline.create<dai::node::ToF>()->build(socket, presetMode);

    // Output queues
    std::shared_ptr<dai::MessageQueue> depthQueue = tof->depth.createOutputQueue();
    std::shared_ptr<dai::MessageQueue> depthRawQueue = tof->rawDepth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        // Visualize raw depth (unfiltered depth directly from the ToF sensor)
        std::shared_ptr<dai::ImgFrame> depthRaw = depthRawQueue->get<dai::ImgFrame>();
        cv::Mat depthRawImage = colorizeDepth(depthRaw->getCvFrame());
        cv::imshow("depthRaw", depthRawImage);

        // Visualize depth (which is filtered depthRaw)
        std::shared_ptr<dai::ImgFrame> depth = depthQueue->get<dai::ImgFrame>();
        cv::Mat depthImage = colorizeDepth(depth->getCvFrame());
        cv::imshow("depth", depthImage);

        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}