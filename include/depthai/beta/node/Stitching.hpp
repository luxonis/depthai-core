#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/StitchingProperties.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Sync.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief Stitching node. Combines N time-synced image streams into a single stitched image.
 *
 * The node runs on the host by default and can run on an RVC4 device when selected with `setRunOnHost(false)`. Host
 * execution requires depthai-core OpenCV support. Inputs are fixed at build() time and synced by an internal host Sync
 * subnode, so the sources may come from different devices.
 * Two independent stitching modes are available:
 *
 *  - `Mode::PANORAMA` wraps OpenCV's cv::Stitcher and registers the images from their content, so no calibration is
 *    needed, but the cameras have to overlap.
 *  - `Mode::PLANAR_PROJECTION` projects the images onto a plane given in the common origin frame of the inputs
 *    (bird's-eye view), driven purely by the calibration carried in the messages, so it also works without overlap.
 *    All input transformations must have the same origin camera socket.
 */
class Stitching : public DeviceNodeCRTP<BetaNode, Stitching, StitchingProperties> {
   public:
    constexpr static const char* NAME = "Stitching";

    /**
     * Stitching mode.
     */
    using Mode = StitchingProperties::Mode;

    /**
     * A plane the images are projected onto in `Mode::PLANAR_PROJECTION`, expressed in the common origin frame of the
     * input transformations.
     */
    using Plane = StitchingProperties::Plane;

    /**
     * The pinhole camera `Mode::PLANAR_PROJECTION` renders the plane from.
     */
    using VirtualCamera = StitchingProperties::VirtualCamera;

    /**
     * Camera projection model the images are warped onto.
     */
    using CameraModel = StitchingProperties::CameraModel;

    /**
     * Seam estimation method.
     */
    using SeamFinder = StitchingProperties::SeamFinder;

   protected:
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:
    Stitching();
    Stitching(std::unique_ptr<Properties> props);
    ~Stitching() override;

    /**
     * Internal Sync node time-aligning the inputs.
     */
    Subnode<dai::node::Sync> sync{*this, "sync"};

   private:
    // Configure-mode nodes do not instantiate subnodes, so retain their deserialized dynamic input interface locally.
    std::unique_ptr<InputMap> configuredInputs =
        configureMode ? std::make_unique<InputMap>(
                            *this, "inputs", InputDescription{"", DEFAULT_GROUP, false, 10, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE})
                      : nullptr;

   public:
    /**
     * A map of inputs, one per stitched source. Populated by build().
     */
    InputMap& inputs = configuredInputs ? *configuredInputs : sync->inputs;

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
     * Specify whether to run on host or an RVC4 device. By default, the node runs on host.
     */
    void setRunOnHost(bool runOnHost) override;

    /**
     * Check whether the node is configured to run on host.
     */
    bool runOnHost() const override;

    /**
     * Set the stitching mode. `Mode::PLANAR_PROJECTION` additionally needs a plane, see `setPlane()`.
     */
    void setMode(Mode mode);
    Mode getMode() const;

    /**
     * Plane the images are projected onto, in `Mode::PLANAR_PROJECTION`. Required in that mode.
     *
     * The plane is expressed in the common origin frame of the incoming transformations, which is the camera socket
     * their extrinsics are relative to. A plane that all cameras look at from the same side gives the usual bird's-eye
     * view; the plane does not have to be horizontal.
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
     * move relative to each other. When false, registration evaluates getEstimationFrames() complete
     * candidates without composing them, selects the one with the strongest geometrically consistent feature-match score,
     * and then starts emitting panoramas using that transform. Projection maps, output regions, seam masks and exposure
     * parameters are prepared with the first emitted panorama and reused for subsequent groups.
     */
    void setContinuous(bool continuous);
    bool getContinuous() const;

    /**
     * Number of complete registration candidates evaluated before the strongest transform is fixed.
     * Failed registrations, oversized candidates, and candidates that omit an input do not count. No panorama is emitted
     * while the candidates are being evaluated. Only used when continuous is false.
     */
    void setEstimationFrames(uint32_t frames);
    uint32_t getEstimationFrames() const;

    /**
     * Reject panorama registrations whose projected canvas exceeds this size before OpenCV allocates and composes it.
     * This protects against degenerate feature matches producing extremely large canvases. By default the size is
     * unbounded.
     */
    void setMaxPanoramaSize(uint32_t width, uint32_t height);

    /**
     * Discard the fixed transform and composition state and re-run the estimation. In `Mode::PLANAR_PROJECTION` the
     * projection maps, seams and exposure gains are rebuilt from the next synced group. In non-continuous
     * `Mode::PANORAMA`, registration is repeated and the fixed maps, regions, seams and exposure parameters are rebuilt.
     */
    void resetTransform();

    /// Confidence below which an image is dropped from the panorama
    void setPanoConfidenceThreshold(double threshold);
    double getPanoConfidenceThreshold() const;

    void setSeamFinder(SeamFinder finder);
    SeamFinder getSeamFinder() const;

    void buildInternal() override;
    void run() override;

   private:
    void initializeHostState();
    void invalidateHostState();
    void initializeInputNames(size_t numInputs);

    class Impl;
    std::shared_ptr<Impl> impl;
    std::atomic_bool hostStateInvalidated{false};
    mutable std::mutex hostPropertiesMutex;

    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 4, {{DatatypeEnum::MessageGroup, true}}}};
    std::vector<std::string> inputNames;
    bool runOnHostVar = true;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
