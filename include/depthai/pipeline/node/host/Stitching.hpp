#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {
namespace node {

/**
 * @brief Stitching node. Combines N time-synced image streams into a single stitched image.
 *
 * The node is host only. Inputs are fixed at build() time and synced by an internal Sync subnode, so the sources may
 * come from different devices. Two independent stitching modes are available:
 *
 *  - `Mode::PANORAMA` wraps OpenCV's cv::Stitcher and registers the images from their content, so no calibration is
 *    needed, but the cameras have to overlap.
 *  - `Mode::PLANAR_PROJECTION` projects the images onto a plane given in the reference frame of the inputs
 *    (bird's-eye view), driven purely by the calibration carried in the messages, so it also works without overlap.
 *    All inputs must be expressed in the same reference frame - place a `CoordinateFrameTransform` in front of the
 *    node when the streams come from different devices.
 */
class Stitching : public NodeCRTP<ThreadedHostNode, Stitching> {
   public:
    constexpr static const char* NAME = "Stitching";

    /**
     * Stitching mode.
     */
    enum class Mode {
        /// Photo panorama, images related by a perspective (rotation only) transform, registered from image content
        PANORAMA,
        /// Projection of all views onto a plane, rendered by a virtual camera. Driven by the calibration of the inputs
        PLANAR_PROJECTION,
    };

    /**
     * A plane the images are projected onto in `Mode::PLANAR_PROJECTION`, expressed in the reference frame the input
     * transformations are expressed in.
     */
    struct Plane {
        /// A point lying on the plane.
        Point3f point;
        /// Normal of the plane, does not have to be of unit length.
        Point3f normal;
        /// Length unit of `point`.
        LengthUnit unit = LengthUnit::CENTIMETER;
    };

    /**
     * The pinhole camera `Mode::PLANAR_PROJECTION` renders the plane from.
     */
    struct VirtualCamera {
        /// Pose of the camera w.r.t. the reference frame, i.e. the matrix mapping camera points into that frame.
        std::array<std::array<float, 4>, 4> pose = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
        /// Length unit of the translation part of `pose`.
        LengthUnit unit = LengthUnit::CENTIMETER;
        /// Intrinsic matrix of the camera.
        std::array<std::array<float, 3>, 3> intrinsics = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        /// Width of the rendered image in pixels.
        uint32_t width = 0;
        /// Height of the rendered image in pixels.
        uint32_t height = 0;

        /**
         * Build a camera placed at `position` and looking at `target`.
         *
         * @param position Camera center, in the reference frame
         * @param target Point the optical axis passes through, in the reference frame
         * @param up Direction that ends up pointing up in the rendered image, does not have to be perpendicular to the
         * optical axis
         * @param hFovDegrees Horizontal field of view of the camera
         * @param width Width of the rendered image in pixels
         * @param height Height of the rendered image in pixels
         * @param unit Length unit of `position` and `target`
         */
        static VirtualCamera lookAt(const Point3f& position,
                                    const Point3f& target,
                                    const Point3f& up,
                                    float hFovDegrees,
                                    uint32_t width,
                                    uint32_t height,
                                    LengthUnit unit = LengthUnit::CENTIMETER);
    };

    /**
     * Camera projection model the images are warped onto.
     */
    enum class CameraModel {
        /// Spherical surface, the OpenCV default
        SPHERICAL,
        /// Plane, appropriate for a pinhole camera and small fields of view
        PINHOLE,
        /// Cylindrical surface
        CYLINDRICAL,
    };

    /**
     * Feature detector used for registration.
     */
    enum class FeaturesFinder { ORB, SIFT, AKAZE, BRISK };

    /**
     * Pairwise features matcher.
     */
    enum class FeaturesMatcher { HOMOGRAPHY, AFFINE };

    /**
     * Initial camera parameters estimator.
     */
    enum class Estimator { HOMOGRAPHY, AFFINE };

    /**
     * Bundle adjustment cost function refining the estimated camera parameters.
     */
    enum class BundleAdjuster { NONE, RAY, REPROJECTION, AFFINE, AFFINE_PARTIAL };

    /**
     * Exposure compensation applied before blending.
     */
    enum class ExposureCompensator { NONE, GAIN, GAIN_BLOCKS, CHANNELS, CHANNELS_BLOCKS };

    /**
     * Seam estimation method.
     */
    enum class SeamFinder { NONE, VORONOI, DP_COLOR, DP_COLOR_GRAD, GRAPHCUT_COLOR, GRAPHCUT_COLOR_GRAD };

    /**
     * Blender merging the warped images.
     */
    enum class Blender { NONE, FEATHER, MULTI_BAND };

    /**
     * Wave correction straightening the panorama.
     */
    enum class WaveCorrection { NONE, HORIZONTAL, VERTICAL };

    /**
     * Interpolation used when warping.
     */
    enum class Interpolation { NEAREST, LINEAR, CUBIC, AREA, LANCZOS4 };

    Stitching();
    ~Stitching();

    /**
     * Internal Sync node time-aligning the inputs.
     */
    Subnode<node::Sync> sync{*this, "sync"};

    /**
     * A map of inputs, one per stitched source. Populated by build().
     */
    InputMap& inputs = sync->inputs;

    /**
     * Stitched image, ImgFrame of type BGR888i.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Build the node with a fixed set of sources. The number of inputs cannot change afterwards.
     * @param sources Outputs to stitch, at least two. Inputs are named input0..inputN-1 in this order
     */
    std::shared_ptr<Stitching> build(const std::vector<Node::Output*>& sources);

    /**
     * Build the node with a fixed number of unlinked inputs, to be fed from host queues.
     * @param numInputs Number of inputs to stitch, at least two. Inputs are named input0..inputN-1
     */
    std::shared_ptr<Stitching> build(size_t numInputs);

    /**
     * Number of inputs the node was built with.
     */
    size_t getNumInputs() const;

    /**
     * Set the maximal interval between messages of a synced group.
     * @param syncThreshold Maximal interval between messages in the group
     */
    void setSyncThreshold(std::chrono::nanoseconds syncThreshold);

    /**
     * Set the stitching mode. `Mode::PLANAR_PROJECTION` additionally needs a plane, see `setPlane()`.
     */
    void setMode(Mode mode);
    Mode getMode() const;

    /**
     * Plane the images are projected onto, in `Mode::PLANAR_PROJECTION`. Required in that mode.
     *
     * The plane is expressed in the reference frame of the incoming transformations, which is the frame the extrinsics
     * of the messages are relative to - a camera of one of the devices, typically set with a
     * `CoordinateFrameTransform` in front of this node. A plane that all cameras look at from the same side gives the
     * usual bird's-eye view; the plane does not have to be horizontal.
     */
    void setPlane(const Plane& plane);
    void setPlane(const Point3f& point, const Point3f& normal, LengthUnit unit = LengthUnit::CENTIMETER);
    std::optional<Plane> getPlane() const;

    /**
     * Camera the plane is rendered from, in `Mode::PLANAR_PROJECTION`.
     *
     * By default the view is computed from the content: the node intersects the field of view of every input with the
     * plane and places a camera looking straight at the plane so that all of the footprints fit, at a resolution
     * derived from the inputs and bounded by `setMaxViewSize()`.
     */
    void setView(const VirtualCamera& view);
    void setViewAuto();
    std::optional<VirtualCamera> getView() const;

    /**
     * Upper bound on the size of the automatically computed view, in pixels. Defaults to 1920x1920.
     */
    void setMaxViewSize(uint32_t width, uint32_t height);

    /**
     * Distance from a camera center beyond which the plane is not painted anymore. Bounds the automatic view and cuts
     * off the region around the horizon, where a few pixels are stretched over a large part of the plane. Defaults to
     * 10 meters.
     */
    void setMaxRange(float range, LengthUnit unit = LengthUnit::CENTIMETER);
    float getMaxRange(LengthUnit unit = LengthUnit::CENTIMETER) const;

    /**
     * Smallest angle between a camera ray and the plane for the ray to still be used. Rays hitting the plane at a
     * shallower angle are heavily stretched, so they are dropped. Defaults to 5 degrees.
     */
    void setMinIncidenceAngle(float degrees);
    float getMinIncidenceAngle() const;

    /**
     * Set the projection surface the images are warped onto. Defaults to SPHERICAL, same as OpenCV.
     * Only used in `Mode::PANORAMA`.
     */
    void setCameraModel(CameraModel model);
    CameraModel getCameraModel() const;

    /**
     * Re-estimate the camera parameters on every frame. Only used in `Mode::PANORAMA`.
     *
     * When true, registration runs for every synced group, which is slow but tolerates cameras that
     * move relative to each other. When false, registration runs for getEstimationFrames() groups,
     * the results are averaged, and the resulting transform is reused for all later frames.
     */
    void setContinuous(bool continuous);
    bool getContinuous() const;

    /**
     * Number of synced groups the camera parameters are averaged over before the transform is fixed.
     * Only used when continuous is false.
     */
    void setEstimationFrames(uint32_t frames);
    uint32_t getEstimationFrames() const;

    /**
     * Discard the fixed transform and re-run the estimation. In `Mode::PLANAR_PROJECTION` the projection maps, the
     * seams and the exposure gains are rebuilt from the next synced group.
     */
    void resetTransform();

    /// Resolution in megapixels used for registration, negative to use the full resolution
    void setRegistrationResolution(double megapixels);
    double getRegistrationResolution() const;

    /// Resolution in megapixels used for seam estimation
    void setSeamEstimationResolution(double megapixels);
    double getSeamEstimationResolution() const;

    /// Resolution in megapixels used for compositing, negative to use the full resolution
    void setCompositingResolution(double megapixels);
    double getCompositingResolution() const;

    /// Confidence below which an image is dropped from the panorama
    void setPanoConfidenceThreshold(double threshold);
    double getPanoConfidenceThreshold() const;

    void setWaveCorrection(WaveCorrection correction);
    WaveCorrection getWaveCorrection() const;

    void setInterpolation(Interpolation interpolation);
    Interpolation getInterpolation() const;

    void setFeaturesFinder(FeaturesFinder finder);
    FeaturesFinder getFeaturesFinder() const;

    /**
     * Set the pairwise matcher.
     * @param matcher Matcher type
     * @param matchConf Confidence two features are a match, negative to use the OpenCV default
     */
    void setFeaturesMatcher(FeaturesMatcher matcher, float matchConf = -1.0f);
    FeaturesMatcher getFeaturesMatcher() const;

    void setEstimator(Estimator estimator);
    Estimator getEstimator() const;

    void setBundleAdjuster(BundleAdjuster adjuster);
    BundleAdjuster getBundleAdjuster() const;

    void setExposureCompensator(ExposureCompensator compensator);
    ExposureCompensator getExposureCompensator() const;

    void setSeamFinder(SeamFinder finder);
    SeamFinder getSeamFinder() const;

    /**
     * Set the blender.
     * @param blender Blender type
     * @param strength Blending strength in percent of the panorama size, for FEATHER and MULTI_BAND
     */
    void setBlender(Blender blender, float strength = 5.0f);
    Blender getBlender() const;

    void buildInternal() override;

   private:
    void run() override;

    class Impl;
    Pimpl<Impl> impl;

    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 4, {{DatatypeEnum::MessageGroup, true}}}};
    std::vector<std::string> inputNames;
};

}  // namespace node
}  // namespace dai
