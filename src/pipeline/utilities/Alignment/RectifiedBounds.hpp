#pragma once

#include <cmath>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

namespace dai {
namespace detail {

// Limit the four float rectification maps to 256 MiB, before frame pools.
constexpr int MAX_RECTIFIED_DIMENSION = 16384;
constexpr int MAX_RECTIFIED_PIXELS = 16 * 1024 * 1024;

inline void validateRectifiedSize(int width, int height) {
    if(width <= 0 || height <= 0 || width > MAX_RECTIFIED_DIMENSION || height > MAX_RECTIFIED_DIMENSION
       || static_cast<std::int64_t>(width) * height > MAX_RECTIFIED_PIXELS) {
        throw std::runtime_error("Alignment: rectified image exceeds supported canvas size");
    }
}

// Keep both viewing cones in the intermediate image before applying the depth-dependent shift.
inline cv::Rect rectifiedBounds(cv::Size sourceSize,
                                const cv::Mat& sourceK,
                                const cv::Mat& sourceDistortion,
                                const cv::Mat& sourceRotation,
                                cv::Size targetSize,
                                const cv::Mat& targetK,
                                const cv::Mat& targetRotation) {
    validateRectifiedSize(sourceSize.width, sourceSize.height);
    cv::Rect bounds(0, 0, sourceSize.width, sourceSize.height);
    const auto includeBorder = [&](cv::Size size, const cv::Mat& intrinsics, const cv::Mat& distortion, const cv::Mat& rotation) {
        validateRectifiedSize(size.width, size.height);
        std::vector<cv::Point2f> border, rectified;
        border.reserve(2 * (size.width + size.height + 2));
        for(int x = 0; x <= size.width; ++x) {
            border.emplace_back(x, 0);
            border.emplace_back(x, size.height);
        }
        for(int y = 0; y <= size.height; ++y) {
            border.emplace_back(0, y);
            border.emplace_back(size.width, y);
        }
        cv::undistortPoints(border, rectified, intrinsics, distortion, rotation, sourceK, cv::TermCriteria(cv::TermCriteria::COUNT, 30, 0.01));
        for(const auto& point : rectified) {
            if(!std::isfinite(point.x) || !std::isfinite(point.y) || point.x <= -MAX_RECTIFIED_DIMENSION || point.x >= MAX_RECTIFIED_DIMENSION
               || point.y <= -MAX_RECTIFIED_DIMENSION || point.y >= MAX_RECTIFIED_DIMENSION) {
                throw std::runtime_error("Alignment: invalid rectified image bounds");
            }
        }
        bounds |= cv::boundingRect(rectified);
        validateRectifiedSize(bounds.width, bounds.height);
    };
    includeBorder(sourceSize, sourceK, sourceDistortion, sourceRotation);
    includeBorder(targetSize, targetK, cv::Mat{}, targetRotation);
    return bounds;
}

}  // namespace detail
}  // namespace dai
