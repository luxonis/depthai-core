#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {
namespace node {

/**
 * @brief Stitching node. Combines N time-synced image streams into a single stitched image.
 *
 * The node is host only and wraps OpenCV's cv::Stitcher. Inputs are fixed at build() time and
 * synced by an internal Sync subnode, so the sources may come from different devices.
 */
class Stitching : public NodeCRTP<ThreadedHostNode, Stitching> {
   public:
    constexpr static const char* NAME = "Stitching";

    /**
     * Stitching mode.
     */
    enum class Mode {
        /// Photo panorama, images related by a perspective (rotation only) transform
        PANORAMA,
        /// Planar projection, images related by an affine transform. Not implemented yet
        PLANAR_PROJECTION,
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
     * Set the stitching mode. PLANAR_PROJECTION is not implemented yet and throws.
     */
    void setMode(Mode mode);
    Mode getMode() const;

    /**
     * Set the projection surface the images are warped onto. Defaults to SPHERICAL, same as OpenCV.
     */
    void setCameraModel(CameraModel model);
    CameraModel getCameraModel() const;

    /**
     * Re-estimate the camera parameters on every frame.
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
     * Discard the fixed transform and re-run the estimation. Only useful when continuous is false.
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
