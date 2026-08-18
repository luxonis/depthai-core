#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <vector>

#include "depthai/pipeline/node/host/Stitching.hpp"

namespace dai {
namespace node {

/**
 * Composes a panorama whose registered camera geometry does not change.
 *
 * Projection maps, image regions, validity and seam masks, and exposure parameters are prepared from the first image
 * group and reused. Later groups only resize (when requested), remap, compensate and blend.
 */
class FixedPanoramaCompositor {
   public:
    struct Config {
        Stitching::CameraModel cameraModel = Stitching::CameraModel::SPHERICAL;
        Stitching::SeamFinder seamFinder = Stitching::SeamFinder::GRAPHCUT_COLOR;
        double compositingResolution = -1.0;
        double seamEstimationResolution = 0.1;
    };

    void setConfig(const Config& config);
    void reset();
    bool isPrepared() const;

    /** Build all fixed composition state from a registered OpenCV camera model and one image group. */
    void prepare(const std::vector<cv::Mat>& images, const std::vector<cv::detail::CameraParams>& cameras, double registrationScale);

    /** Compose images using only the state built by prepare(). */
    cv::Mat compose(const std::vector<cv::Mat>& images);

    cv::Size getCanvasSize() const;

   private:
    struct Source {
        cv::Size inputSize;
        cv::Size composeInputSize;
        cv::Rect roi;
        cv::Mat map1;
        cv::Mat map2;
        cv::Mat mask;
        cv::Mat seamMask;
    };

    std::vector<cv::Mat> warp(const std::vector<cv::Mat>& images) const;
    void prepareCompositing(const std::vector<cv::Mat>& warped);

    Config config;
    bool prepared = false;
    double composeScale = 1.0;
    cv::Rect canvas;
    std::vector<Source> sources;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
    cv::Ptr<cv::detail::Blender> blender;
};

}  // namespace node
}  // namespace dai
