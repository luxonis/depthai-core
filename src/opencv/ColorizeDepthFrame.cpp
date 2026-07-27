#include "depthai/utility/ColorizeDepthFrame.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace dai {
namespace utility {

static std::pair<float, float> computeAutoDepthRange(const cv::Mat& depth32f, const cv::Mat& invalidMask) {
    std::vector<float> values;
    const int nz = cv::countNonZero(invalidMask == 0);
    if(nz > 0) {
        values.reserve(static_cast<size_t>(nz));
        for(int r = 0; r < depth32f.rows; ++r) {
            const float* d = depth32f.ptr<float>(r);
            const uchar* m = invalidMask.ptr<uchar>(r);
            for(int c = 0; c < depth32f.cols; ++c) {
                if(!m[c]) {
                    values.push_back(d[c]);
                }
            }
        }
    }

    if(values.empty()) {
        return {0.0f, 0.0f};
    }

    std::sort(values.begin(), values.end());
    auto percentile = [&](double p) -> float {
        const size_t idx = static_cast<size_t>(std::round((p / 100.0) * (values.size() - 1)));
        return values[idx];
    };

    return {percentile(3.0), percentile(95.0)};
}

cv::Mat colorizeDepthFrame(const cv::Mat& frame, float minDepth, float maxDepth, cv::ColormapTypes colormap, bool useLog) {
    if(frame.empty() || frame.channels() != 1) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }

    try {
        cv::Mat depth32f;
        frame.convertTo(depth32f, CV_32F);

        // Zero depth values are treated as invalid/missing.
        cv::Mat invalidMask = depth32f == 0.0f;

        if(maxDepth <= minDepth) {
            const auto range = computeAutoDepthRange(depth32f, invalidMask);
            minDepth = range.first;
            maxDepth = range.second;
            if(maxDepth <= minDepth) {
                return cv::Mat::zeros(frame.size(), CV_8UC3);
            }
        }

        cv::Mat scaled;
        if(useLog) {
            // Avoid log(0) by adding a small epsilon.
            constexpr float EPS = 1e-6f;
            const float logMinDepth = std::log(minDepth + EPS);
            const float logMaxDepth = std::log(maxDepth + EPS);

            if(logMaxDepth <= logMinDepth) {
                return cv::Mat::zeros(frame.size(), CV_8UC3);
            }

            cv::Mat logDepth = depth32f + EPS;
            cv::log(logDepth, logDepth);

            // Set invalid pixels to the lower log bound so they map to 0.
            logDepth.setTo(logMinDepth, invalidMask);

            scaled = (logDepth - logMinDepth) * (255.0f / (logMaxDepth - logMinDepth));
        } else {
            scaled = (depth32f - minDepth) * (255.0f / (maxDepth - minDepth));
        }

        cv::Mat depth8U;
        scaled.convertTo(depth8U, CV_8U);

        cv::Mat colored;
        cv::applyColorMap(depth8U, colored, colormap);

        // Set invalid depth pixels to black.
        colored.setTo(cv::Scalar::all(0), invalidMask);

        return colored;
    } catch(const cv::Exception&) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }
}

ImgFrame colorizeDepthFrame(const ImgFrame& frame, float minDepth, float maxDepth, cv::ColormapTypes colormap, bool useLog) {
    try {
        // getCvFrame() is logically const; make a shallow copy to call it.
        ImgFrame temp = frame;
        cv::Mat src = temp.getCvFrame();
        cv::Mat colored = colorizeDepthFrame(src, minDepth, maxDepth, colormap, useLog);

        ImgFrame output;
        output.setMetadata(frame);
        output.setCvFrame(colored, ImgFrame::Type::BGR888i);
        return output;
    } catch(const std::exception&) {
        ImgFrame output;
        if(frame.getWidth() > 0 && frame.getHeight() > 0) {
            output.setSize(frame.getWidth(), frame.getHeight());
            output.setType(ImgFrame::Type::BGR888i);
            std::vector<uint8_t> zeros(static_cast<size_t>(frame.getWidth()) * frame.getHeight() * 3, 0);
            output.setData(std::move(zeros));
        }
        return output;
    }
}

}  // namespace utility
}  // namespace dai
