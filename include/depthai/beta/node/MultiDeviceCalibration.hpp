#pragma once

#include <depthai/utility/spimpl.h>

#include <cstddef>
#include <depthai/beta/datatype/MultiDeviceCalibrationControl.hpp>
#include <depthai/beta/datatype/MultiDeviceCalibrationResult.hpp>
#include <depthai/beta/device/MultiDeviceCalibrationHandler.hpp>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/common/DepthUnit.hpp>
#include <depthai/common/Extrinsics.hpp>
#include <depthai/common/WorldConstraints.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace dai {
class MessageGroup;
namespace beta {
namespace node {

/**
 * @brief Estimates metric cross-device calibration from synchronized camera images.
 *
 * The node is host-only and one-shot. Configuration is supplied before the
 * `start` control command; the node emits a pure-data result and enters its
 * completed state. A `reset` command clears the cycle and permits another run.
 */
class MultiDeviceCalibration : public NodeCRTP<dai::node::ThreadedHostNode, MultiDeviceCalibration, false> {
   public:
    constexpr static const char* NAME = "MultiDeviceCalibration";

    MultiDeviceCalibration();
    ~MultiDeviceCalibration() override;

    /** Synchronized camera streams, keyed internally by device ID and socket. */
    Subnode<dai::node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    /** Explicit lifecycle control. */
    Input inputControl{*this, {"inputControl", DEFAULT_GROUP, NON_BLOCKING_QUEUE, 5, {{{DatatypeEnum::MultiDeviceCalibrationControl, false}}}}};

    /** Synchronized image groups. */
    Input syncInput{*this, {"inSync", DEFAULT_GROUP, NON_BLOCKING_QUEUE, 2, {{{DatatypeEnum::MessageGroup, true}}}}};

    /** Pure-data calibration result. */
    Output calibrationOutput{*this, {"calibrationOutput", DEFAULT_GROUP, {{{DatatypeEnum::MultiDeviceCalibrationResult, false}}}}};

    /**
     * Register a live camera stream.
     *
     * The device ID and socket are taken from the Camera node that owns the
     * output, so the output must come directly from a built Camera created on a
     * device. For outputs of other nodes (ImageManip, host nodes, replayed
     * streams) use the explicit overload.
     */
    void addCamera(Node::Output& cameraOutput);

    /**
     * Register an image stream with an explicit identity.
     *
     * Use this when the output does not come directly from a Camera node, for
     * example for replayed or recorded streams. Image metadata is not a
     * reliable source of cross-device identity, so the identity is passed in.
     */
    void addCamera(std::string deviceId, CameraBoardSocket socket, Node::Output& cameraOutput);

    /** Set the number of complete synchronized groups required before solving. */
    void setSampleCount(std::size_t sampleCount);
    std::size_t getSampleCount() const;

    /** Supply a metric camera-center distance for an endpoint pair. */
    void setKnownDistance(std::string fromDeviceId,
                          CameraBoardSocket fromSocket,
                          std::string toDeviceId,
                          CameraBoardSocket toSocket,
                          float distance,
                          LengthUnit unit = LengthUnit::CENTIMETER);

    /**
     * Supply an optional initial pose estimate between two devices.
     *
     * The edge may connect any two sockets known to the devices' calibrations,
     * registered or not; the node converts it to the devices' calibration
     * origins when the run starts. One guess per device pair, the last one set
     * wins. Sockets unknown to a device's calibration fail the run with a
     * descriptive result.
     */
    void setInitialGuess(const MultiDeviceExtrinsics& guess);

    /** Supply several initial guesses at once, for example a previous result's graph. */
    void setInitialGuesses(const std::vector<MultiDeviceExtrinsics>& guesses);

    /** Seed the run with every edge of a previous calibration. */
    void setInitialGuesses(const MultiDeviceCalibrationHandler& previous);

    /**
     * Optional. Restrict metric scale recovery to an explicit factory-calibrated
     * stereo pair. By default every pair of registered cameras on the same
     * device may anchor the metric scale.
     */
    void setStereoPair(std::string deviceId, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket);

    /** Override live calibration for recorded/offline streams and tests. */
    void setDeviceCalibration(std::string deviceId, const CalibrationHandler& calibration);

    /**
     * Give the calibration a metric reference through fiducial markers placed in the scene: measured distances
     * between marker centers, distances from marker centers to cameras, and printed marker sizes, as described in
     * WorldConstraints. Markers are detected in every synchronized group; a group counts towards the sample count
     * only when every marker the constraints refer to is seen by at least two registered cameras, of the same
     * device or of different devices.
     *
     * The markers take part in the whole calibration. A marker center seen from two devices is a point match
     * that is used next to the image features, and the measurements are fitted together with both: they set how
     * far apart the devices are, which one camera per device cannot tell, and they refine the rotations.
     *
     * The result is reported as failed, with the reason in its info, when it does not reproduce a measurement,
     * when fitting the measurements raises the epipolar error of the image features, or when the distance
     * between two devices leaves what registered stereo pairs give on their own.
     *
     * A socket distance must name the device of its socket in its deviceId, with the ID the cameras of that
     * device are registered under. The socket is a registered camera or any other calibrated socket of the
     * device. Can be changed only while the node is idle.
     */
    void setWorldConstraints(const WorldConstraints& constraints);
    /// Same as setWorldConstraints() with the JSON form described in WorldConstraints.
    void setWorldConstraintsFromJson(const std::string& json);
    /// Same as setWorldConstraints() with the JSON form described in WorldConstraints read from a file.
    void setWorldConstraintsFromFile(const std::filesystem::path& path);
    /// Remove the marker constraints again. Can be changed only while the node is idle.
    void clearWorldConstraints();
    /// The marker constraints set with setWorldConstraints(), or none.
    std::optional<WorldConstraints> getWorldConstraints() const;

    void buildInternal() override;

   private:
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    bool initializeCycle(const std::shared_ptr<MessageGroup>& firstGroup, std::string& error);
    bool loadCompleteGroup(const std::shared_ptr<MessageGroup>& group, std::string& error);
    void estimateAndEmit();
#endif

    void run() override;

    class Impl;
    spimpl::unique_impl_ptr<Impl> pimpl;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
