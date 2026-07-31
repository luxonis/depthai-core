#pragma once

#include <chrono>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/CoordinateFrame.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/spimpl.h"

namespace dai {
namespace node {

/**
 * @brief Estimates the rig calibration of a multi-device setup, i.e. the transformations between the coordinate frames
 * of cameras attached to *different* devices, from images of a shared scene.
 *
 * The node is a plain host node: it synchronizes the camera streams it is given, feeds them to the dynamic calibration
 * library and emits the resulting rig edges as a `MultiDeviceCalibrationResult`. Only inter-device edges are emitted -
 * the intra-device calibration stays with the device and is read from it, so the result is a forest that can be handed
 * to `MultiDeviceCalibrationHandler` / `Pipeline::setMultiDeviceCalibration()` directly.
 *
 * Two things have to be supplied by the user, because they cannot be observed from images alone:
 *  - a rough initial guess of the pose of every device w.r.t. the first one (`setInitialGuess()`), used to initialize
 *    the optimization,
 *  - the metric scale. The translation between two cameras that do not share a device is only observable up to scale,
 *    so at least one known distance per inter-device edge is needed (`setKnownDistance()`). For devices with a stereo
 *    pair (e.g. OAK-4D) the known factory baselines are used automatically and no distance has to be given.
 */
class MultiDeviceCalibration : public NodeCRTP<ThreadedHostNode, MultiDeviceCalibration> {
   public:
    constexpr static const char* NAME = "MultiDeviceCalibration";

    /**
     * How the inter-device transformations are estimated.
     */
    enum class Method {
        /**
         * The dynamic calibration library. Works with a single camera per device but needs a rough initial guess of
         * every device's pose (`setInitialGuess()`) and, unless every device has a stereo pair, a known inter-device
         * distance (`setKnownDistance()`) to fix the metric scale.
         */
        DYNAMIC_CALIBRATION,
        /**
         * A target-free structure-from-motion estimate that needs no initial guess and no known distance. SIFT
         * four-view tracks are matched within each device's stereo pair and across the devices, an essential matrix
         * between the two reference cameras gives the rotation and the translation direction, and the metric
         * translation scale is recovered by triangulating the same tracks in each device's own metric stereo pair and
         * robustly fitting the scalar that aligns the two point clouds. Requires exactly a stereo pair (two cameras)
         * per device.
         */
        FEATURE_TRACKS,
    };

    MultiDeviceCalibration();
    ~MultiDeviceCalibration() override;

    /**
     * Synchronized camera streams the rig is estimated from. Prefer `build()` / `addCamera()` over linking directly,
     * since every stream has to be associated with the coordinate frame it comes from.
     */
    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    /**
     * Synchronized group of images, one per registered camera.
     */
    Input syncInput{*this, {"inSync", DEFAULT_GROUP, false, 1, {{DatatypeEnum::MessageGroup, true}}}};

    /**
     * Estimated rig calibration.
     */
    Output rigCalibration{*this, {"rigCalibration", DEFAULT_GROUP, {{{DatatypeEnum::MultiDeviceCalibrationResult, false}}}}};

    /**
     * Register the camera streams to estimate the rig from. At least two cameras on at least two different devices are
     * needed.
     *
     * @param sources Camera outputs together with the coordinate frame each of them belongs to
     */
    std::shared_ptr<MultiDeviceCalibration> build(const std::vector<std::pair<CoordinateFrame, Node::Output*>>& sources);

    /**
     * Register a single camera stream.
     *
     * @param frame Coordinate frame the stream comes from, i.e. the device id and the board socket of the camera
     * @param source Camera output
     */
    void addCamera(const CoordinateFrame& frame, Node::Output& source);

    /**
     * Rough initial guess of the pose of `from` w.r.t. `to`, used to initialize the optimization. Needed once per
     * device pair connecting the rig, between frames registered with `addCamera()`.
     *
     * @param from Source frame
     * @param to Reference frame the guess is expressed in
     * @param guess Pose of `from` w.r.t. `to`
     */
    void setInitialGuess(const CoordinateFrame& from, const CoordinateFrame& to, const Extrinsics& guess);

    /**
     * Known distance between the centers of two cameras, fixing the metric scale of the estimated rig. Distances
     * between cameras of the same device are taken from the device calibration automatically, so only inter-device
     * distances have to be given - and only for rigs without a stereo pair per device.
     *
     * @param from First frame
     * @param to Second frame
     * @param distance Distance between the two camera centers
     * @param unit Unit of `distance`
     */
    void setKnownDistance(const CoordinateFrame& from, const CoordinateFrame& to, float distance, LengthUnit unit = LengthUnit::CENTIMETER);

    /**
     * Number of synchronized image sets to accumulate before estimating the rig. Defaults to 10.
     */
    void setSampleCount(size_t sampleCount);

    /**
     * Keep estimating the rig, emitting a new result every `sampleCount` image sets. Off by default, i.e. the node
     * emits a single result and then stops consuming images.
     */
    void setContinuous(bool continuous);

    /**
     * Performance mode passed to the dynamic calibration library. Setting an explicit mode turns the automatic
     * multi-strategy search off, so only this mode is attempted.
     */
    void setPerformanceMode(DynamicCalibrationControl::PerformanceMode mode);

    /**
     * When enabled (the default), the node does not trust a single solve: it attempts several strategies - a joint
     * solve over all cameras and a pairwise solve per device, each across a sweep of performance modes, of the
     * keep-camera-centers flag and of yaw perturbations of the initial guess - scores every candidate with the
     * dynamic calibration library's own calibration-confidence and Sampson-error metrics, and keeps the best edge per
     * device. Disabled implicitly by `setPerformanceMode()`.
     */
    void setAutoStrategy(bool enable);

    /**
     * Yaw offsets, in degrees, added to the initial guess of each device when searching for the best rig during the
     * automatic strategy. A rough or stale guess is a common reason for a poor solve, so perturbing it and keeping the
     * best-scoring result makes the estimation robust to it. Defaults to a small symmetric sweep around zero.
     */
    void setGuessYawSweep(const std::vector<float>& offsetsDegrees);

    /**
     * Method used to estimate the inter-device transformations. Defaults to `DYNAMIC_CALIBRATION`. Select
     * `FEATURE_TRACKS` for a target-free estimate that needs neither an initial guess nor a known distance, at the
     * cost of requiring a stereo pair per device.
     */
    void setMethod(Method method);

    void buildInternal() override;

   private:
    void run() override;

    /// Estimate the rig with the dynamic calibration library and emit the result.
    void estimate();

    /// Estimate the rig from SIFT four-view tracks (no DCL) and emit the result.
    void estimateFromTracks();

    /// Scale correction of one inter-device edge, derived from the known distances.
    float resolveScale(const std::vector<std::vector<float>>& transform,
                       const CoordinateFrame& baseReference,
                       const CoordinateFrame& reference,
                       std::vector<std::string>& notes) const;

    class Impl;
    spimpl::unique_impl_ptr<Impl> pimpl;
};

}  // namespace node
}  // namespace dai
