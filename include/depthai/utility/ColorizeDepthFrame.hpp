#pragma once

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    // project
    #include "depthai/pipeline/datatype/ImgFrame.hpp"

    // opencv
    #include <opencv2/opencv.hpp>

namespace dai {
namespace utility {

/**
 * Colorize a single-channel depth frame into a BGR888i ImgFrame.
 *
 * If @p maxDepth is less than or equal to @p minDepth (e.g. both set to 0),
 * the effective bounds are auto-computed from the 3rd and 95th percentiles of
 * the finite positive (valid) depth values.
 *
 * @param frame Input depth frame (single channel, e.g. RAW16/GRAYF16/GRAY8).
 *            Depth values are usually in millimeters.
 * @param minDepth Lower depth bound in millimeters of the range to map to 0 (default: 500 mm).
 * @param maxDepth Upper depth bound in millimeters of the range to map to 255 (default: 12000 mm).
 * @param colormap OpenCV colormap to apply (default: COLORMAP_JET).
 * @param useLog If true, apply logarithmic scaling before mapping to [0, 255].
 * @returns New BGR888i ImgFrame containing the colorized depth image. Invalid (non-positive or non-finite) pixels are set to black.
 */
ImgFrame colorizeDepthFrame(
    const ImgFrame& frame, float minDepth = 500.0f, float maxDepth = 12000.0f, cv::ColormapTypes colormap = cv::COLORMAP_JET, bool useLog = true);

/**
 * Colorize a single-channel depth cv::Mat.
 *
 * If @p maxDepth is less than or equal to @p minDepth (e.g. both set to 0),
 * the effective bounds are auto-computed from the 3rd and 95th percentiles of
 * the finite positive (valid) depth values.
 *
 * @param frame Input depth frame (single channel, e.g. CV_16U/CV_32F/CV_8U).
 *            Depth values are usually in millimeters.
 * @param minDepth Lower depth bound in millimeters of the range to map to 0 (default: 500 mm).
 * @param maxDepth Upper depth bound in millimeters of the range to map to 255 (default: 12000 mm).
 * @param colormap OpenCV colormap to apply (default: COLORMAP_JET).
 * @param useLog If true, apply logarithmic scaling before mapping to [0, 255].
 * @returns BGR cv::Mat colorized depth image. Invalid (non-positive or non-finite) pixels are set to black.
 */
cv::Mat colorizeDepthFrame(
    const cv::Mat& frame, float minDepth = 500.0f, float maxDepth = 12000.0f, cv::ColormapTypes colormap = cv::COLORMAP_JET, bool useLog = true);

}  // namespace utility
}  // namespace dai

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT
