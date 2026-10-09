#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <vector>

#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace utilities {

using node::Stitching;

/**
 * Composes a panorama whose registered camera geometry does not change.
 *
 * Projection maps and image regions are prepared once and reused. The compositor can either perform the full seam,
 * exposure, and blending pipeline or directly copy warped inputs into the panorama.
 */
class FixedPanoramaCompositor {
   public:
    enum class Composition { BLENDED, DIRECT };

    struct Config {
        CameraModel cameraModel = CameraModel::Equirectangular;
        Stitching::SeamFinder seamFinder = Stitching::SeamFinder::GRAPHCUT_COLOR;
        double compositingResolution = -1.0;
        double seamEstimationResolution = 0.1;
        Composition composition = Composition::BLENDED;
    };

    void setConfig(const Config& config);
    void reset();
    bool isPrepared() const;

    /** Build all fixed composition state from a registered OpenCV camera model and one image group. */
    void prepare(const std::vector<cv::Mat>& images, const std::vector<cv::detail::CameraParams>& cameras, double registrationScale);

    /** Compose images using only the state built by prepare(). */
    cv::Mat compose(const std::vector<cv::Mat>& images);

    cv::Size getCanvasSize() const;

    /** Region of the warper's coordinate system the panorama covers; its top-left corner is the panorama's pixel (0, 0). */
    cv::Rect getCanvas() const;

    /** Scale of the rotation warper the panorama is rendered with, i.e. the radius of the projection surface in pixels. */
    double getWarperScale() const;

    /** Visibility of one source in panorama coordinates, using the cached seam masks or direct copying order. */
    cv::Mat getSourceMask(size_t sourceIndex) const;

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
    double warperScale = 0.0;
    cv::Rect canvas;
    std::vector<Source> sources;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
    cv::Ptr<cv::detail::Blender> blender;
};

}  // namespace utilities
}  // namespace dai
