#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <optional>
#include <vector>

#include "FixedBlender.hpp"
#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace utilities {

using node::Stitching;

/**
 * Composes a panorama whose registered camera geometry does not change.
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

    /**
     * Ground the panorama is projected onto: the plane out to `wallDistance` from the eye, a vertical wall beyond it. What lies
     * on the ground then lines up across cameras mounted apart, the camera centers being their `cv::detail::CameraParams::t`.
     * All of it is in the panorama frame, in centimeters.
     */
    struct Ground {
        cv::Vec3d point;            ///< A point on the ground
        cv::Vec3d up;               ///< Unit normal of the ground, on the side of the cameras
        double wallDistance = 0.0;  ///< Horizontal distance from the eye to the wall, beyond every camera
        cv::Vec3d eye;              ///< Center the panorama is seen from
    };

    /**
     * Region of the projection surface, in pixels of a surface of radius `scale`, that one camera covers on the ground.
     * @param camera Intrinsics for `imageSize`, rotation from the camera to the panorama frame and center
     */
    static cv::Rect groundRoi(CameraModel model, double scale, const Ground& ground, const cv::detail::CameraParams& camera, const cv::Size& imageSize);

    /**
     * Build all fixed composition state from a registered OpenCV camera model and one image group.
     * @param ground Surface to project onto, using the camera centers; without it the cameras share one center and the
     * surface is at infinity
     */
    void prepare(const std::vector<cv::Mat>& images,
                 const std::vector<cv::detail::CameraParams>& cameras,
                 double registrationScale,
                 const std::optional<Ground>& ground);

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
        cv::Mat mask;
        cv::Mat seamMask;
        cv::Mat map1;
        cv::Mat map2;
    };

    std::vector<cv::Mat> composeInputs(const std::vector<cv::Mat>& images) const;
    void prepareCompositing(const std::vector<cv::Mat>& warped);
    void prepareDirectCopy();
    void prepareBlender(const std::vector<cv::Scalar>& gains, int bandWidth);

    Config config;
    bool prepared = false;
    double composeScale = 1.0;
    double warperScale = 0.0;
    cv::Rect canvas;
    std::vector<Source> sources;
    FixedBlender blender;
};

}  // namespace utilities
}  // namespace dai
