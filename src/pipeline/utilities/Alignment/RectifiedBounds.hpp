#pragma once

#include <spdlog/logger.h>

#include <cmath>
#include <cstdint>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace dai {
namespace detail {

// Limit the four float rectification maps to 256 MiB, before frame pools.
constexpr int MAX_RECTIFIED_DIMENSION = 16384;
constexpr int MAX_RECTIFIED_PIXELS = 16 * 1024 * 1024;

inline bool isRectifiedSizeSupported(int width, int height) {
    return width > 0 && height > 0 && width <= MAX_RECTIFIED_DIMENSION && height <= MAX_RECTIFIED_DIMENSION
           && static_cast<std::int64_t>(width) * height <= MAX_RECTIFIED_PIXELS;
}

// Keep both viewing cones in the intermediate image before applying the depth-dependent shift.
// When the geometry gives no usable bounds (a border ray behind the rectified camera or too far out, or a canvas past the size
// limits), return the source image: the canvas from before the bounds were expanded. The aligned image can then lose coverage
// near its edges, but alignment keeps running. The size check includes the padding to even sizes for chroma-subsampled formats.
// When a logger is given, the fallback is logged at debug level.
inline cv::Rect rectifiedBounds(cv::Size sourceSize,
                                const cv::Mat& sourceK,
                                const cv::Mat& sourceDistortion,
                                const cv::Mat& sourceRotation,
                                cv::Size targetSize,
                                const cv::Mat& targetK,
                                const cv::Mat& targetRotation,
                                spdlog::logger* logger = nullptr) {
    const cv::Rect sourceBounds(0, 0, sourceSize.width, sourceSize.height);
    const auto useSourceBounds = [&]() {
        if(logger) {
            logger->debug(
                "Alignment: expanded rectified bounds are not usable, using the input size {}x{} as the rectified canvas", sourceSize.width, sourceSize.height);
        }
        return sourceBounds;
    };
    if(!isRectifiedSizeSupported(sourceSize.width, sourceSize.height) || !isRectifiedSizeSupported(targetSize.width, targetSize.height)) {
        return useSourceBounds();
    }
    cv::Rect bounds = sourceBounds;
    const auto includeBorder = [&](cv::Size size, const cv::Mat& intrinsics, const cv::Mat& distortion, const cv::Mat& rotation) {
        std::vector<cv::Point2d> border, normalized;
        std::vector<cv::Point2f> rectified;
        border.reserve(2 * (size.width + size.height + 2));
        for(int x = 0; x <= size.width; ++x) {
            border.emplace_back(x, 0);
            border.emplace_back(x, size.height);
        }
        for(int y = 0; y <= size.height; ++y) {
            border.emplace_back(0, y);
            border.emplace_back(size.width, y);
        }
        cv::undistortPoints(border, normalized, intrinsics, distortion, cv::noArray(), cv::noArray(), cv::TermCriteria(cv::TermCriteria::COUNT, 30, 0.01));
        cv::Matx33d rectification, projection;
        rotation.convertTo(rectification, CV_64F);
        sourceK.convertTo(projection, CV_64F);
        rectified.reserve(normalized.size());
        for(const auto& point : normalized) {
            const cv::Vec3d ray = rectification * cv::Vec3d(point.x, point.y, 1.0);
            if(!std::isfinite(ray[2]) || ray[2] <= 1e-6) {
                return false;
            }
            const cv::Vec3d pixel = projection * ray;
            const cv::Point2d projected(pixel[0] / pixel[2], pixel[1] / pixel[2]);
            if(!std::isfinite(projected.x) || !std::isfinite(projected.y) || projected.x <= -MAX_RECTIFIED_DIMENSION || projected.x >= MAX_RECTIFIED_DIMENSION
               || projected.y <= -MAX_RECTIFIED_DIMENSION || projected.y >= MAX_RECTIFIED_DIMENSION) {
                return false;
            }
            rectified.emplace_back(projected.x, projected.y);
        }
        bounds |= cv::boundingRect(rectified);
        return true;
    };
    if(!includeBorder(sourceSize, sourceK, sourceDistortion, sourceRotation) || !includeBorder(targetSize, targetK, cv::Mat{}, targetRotation)
       || !isRectifiedSizeSupported(bounds.width + bounds.width % 2, bounds.height + bounds.height % 2)) {
        return useSourceBounds();
    }
    return bounds;
}

}  // namespace detail
}  // namespace dai
