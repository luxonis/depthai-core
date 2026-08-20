#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <optional>
#include <vector>

#include "depthai/beta/node/Stitching.hpp"
#include "depthai/common/ImgTransformations.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * Projection of N images onto a common plane, rendered by a virtual pinhole camera - the implementation behind
 * `Stitching::Mode::PLANAR_PROJECTION`.
 *
 * Nothing is registered from the image content: every pixel of the rendered image is the intersection of a ray of the
 * virtual camera with the plane, projected back into each input through the calibration carried by the input's
 * `ImgTransformation`. Both the plane and the virtual camera live in the origin frame the input extrinsics are
 * expressed in, so all inputs have to share that origin camera socket.
 *
 * The geometry is assumed to be fixed: the projection maps, the seams and the exposure gains are built from the first
 * group and reused, so a frame only costs a remap and a blend. Call `reset()` to rebuild them.
 */
class PlanarStitcher {
   public:
    struct Config {
        /// Plane the images are projected onto. Required.
        std::optional<Stitching::Plane> plane;
        /// Camera the plane is rendered from. Computed from the content of the views when not set.
        std::optional<Stitching::VirtualCamera> view;
        /// Upper bound on the size of a computed view, in pixels.
        uint32_t maxViewWidth = 1920;
        uint32_t maxViewHeight = 1920;
        /// Distance from a camera center beyond which the plane is not painted, in centimeters.
        float maxRange = 1000.0f;
        /// Smallest angle between a camera ray and the plane for the ray to be used, in degrees.
        float minIncidenceAngle = 5.0f;
        Stitching::SeamFinder seamFinder = Stitching::SeamFinder::GRAPHCUT_COLOR;
    };

    /// Replace the configuration, dropping everything built from the previous one.
    void setConfig(const Config& config);
    const Config& getConfig() const;

    /// Drop the projection maps, the seams and the exposure gains, so they are rebuilt from the next group.
    void reset();

    /// True once the projection maps have been built.
    bool isPrepared() const;

    /**
     * Build the projection maps from the calibration of one synced group.
     *
     * @param transformations Transformation of every input, all expressed relative to the same origin camera socket
     * @throws std::runtime_error if the inputs are inconsistent or none of them sees the plane
     */
    void prepare(const std::vector<ImgTransformation>& transformations);

    /**
     * Check that a later synced group still has the input count and common origin used by `prepare()`.
     *
     * @throws std::runtime_error if the transformations no longer describe the prepared camera rig
     */
    void validateTransformations(const std::vector<ImgTransformation>& transformations) const;

    /**
     * Render one synced group.
     *
     * @param images Images of all the inputs, in the order `prepare()` was called with. Inputs that do not see the
     *               plane are ignored, but still have to be given
     * @return Rendered BGR image of the size of the resolved view
     */
    cv::Mat compose(const std::vector<cv::Mat>& images);

    /// View the plane is rendered from, either the configured one or the computed one. Valid after `prepare()`.
    const Stitching::VirtualCamera& getResolvedView() const;

    /// Origin camera socket the plane, view and input extrinsics are expressed in. Valid after `prepare()`.
    CameraBoardSocket getOrigin() const;

   private:
    /// One input that sees the plane, reduced to what compositing needs.
    struct Source {
        /// Index of the input among all the inputs `prepare()` was called with.
        size_t index = 0;
        /// Fixed point maps of `roi`, sampling the input image, as produced by cv::convertMaps.
        cv::Mat map1;
        cv::Mat map2;
        /// Pixels of `roi` this input contributes to.
        cv::Mat mask;
        /// Pixels of `roi` left to this input after seam estimation.
        cv::Mat seamMask;
        /// Region of the rendered image the input covers.
        cv::Rect roi;
        /// Size of the input image the maps were built for.
        cv::Size imageSize;
    };

    /// Pose of one input, in the reference frame, in centimeters.
    struct SourcePose {
        cv::Matx33d rotation;
        cv::Vec3d center;
    };

    /// Compute a view covering the intersections of all the fields of view with the plane.
    Stitching::VirtualCamera computeView(const std::vector<ImgTransformation>& transformations, const std::vector<SourcePose>& poses) const;

    /// Estimate the exposure gains and the seams, once, from the first group.
    void prepareCompositing(const std::vector<cv::Mat>& warped);

    Config config;

    bool prepared = false;
    bool compositingPrepared = false;
    /// Number of inputs `prepare()` was called with, including the ones that do not see the plane.
    size_t numInputs = 0;
    std::vector<ImgTransformation> preparedTransformations;
    CameraBoardSocket origin = CameraBoardSocket::AUTO;
    Stitching::VirtualCamera view;
    /// Plane point and unit normal in the reference frame, in centimeters.
    cv::Vec3d planePoint;
    cv::Vec3d planeNormal;
    std::vector<Source> sources;
    cv::Ptr<cv::detail::ExposureCompensator> compensator;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
