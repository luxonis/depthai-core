#pragma once

#include <depthai/utility/spimpl.h>

#include <cstddef>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/common/DepthUnit.hpp>
#include <depthai/common/Extrinsics.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/ThreadedHostNode.hpp>
#include <depthai/pipeline/datatype/MultiDeviceCalibrationControl.hpp>
#include <depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <memory>
#include <string>

namespace dai {
class MessageGroup;
namespace node {

/**
 * @brief Estimates metric cross-device calibration from synchronized camera images.
 *
 * The node is host-only and one-shot. Configuration is supplied before the
 * `start` control command; the node emits a pure-data result and enters its
 * completed state. A `reset` command clears the cycle and permits another run.
 */
class MultiDeviceCalibration : public NodeCRTP<ThreadedHostNode, MultiDeviceCalibration, false> {
   public:
    constexpr static const char* NAME = "MultiDeviceCalibration";

    MultiDeviceCalibration();
    ~MultiDeviceCalibration() override;

    /** Synchronized camera streams, keyed internally by device ID and socket. */
    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    /** Explicit lifecycle control. */
    Input inputControl{*this, {"inputControl", DEFAULT_GROUP, NON_BLOCKING_QUEUE, 5, {{{DatatypeEnum::MultiDeviceCalibrationControl, false}}}}};

    /** Synchronized image groups. */
    Input syncInput{*this, {"inSync", DEFAULT_GROUP, NON_BLOCKING_QUEUE, 2, {{{DatatypeEnum::MessageGroup, true}}}}};

    /** Pure-data calibration result. */
    Output calibrationOutput{*this, {"calibrationOutput", DEFAULT_GROUP, {{{DatatypeEnum::MultiDeviceCalibrationResult, false}}}}};

    /**
     * Register an image stream.
     *
     * The device ID and socket are explicit because image metadata is not a
     * reliable source of cross-device identity.
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

    /** Supply an optional local-origin-to-local-origin initial pose estimate. */
    void setInitialGuess(std::string fromDeviceId, CameraBoardSocket fromSocket, std::string toDeviceId, CameraBoardSocket toSocket, const Extrinsics& guess);

    /** Register an explicit factory-calibrated stereo pair for scale recovery. */
    void setStereoPair(std::string deviceId, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket);

    /** Override live calibration for recorded/offline streams and tests. */
    void setDeviceCalibration(std::string deviceId, const CalibrationHandler& calibration);

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
}  // namespace dai
