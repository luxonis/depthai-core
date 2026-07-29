#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "depthai/common/CoordinateFrame.hpp"
#include "depthai/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

namespace dai {
namespace node {

/**
 * @brief Re-expresses the transformation metadata of messages in a different coordinate frame.
 *
 * In a multi-device pipeline, messages coming from different devices carry extrinsics relative to their own device's
 * reference camera, so they cannot be combined directly. This node rewrites those extrinsics into a single target
 * frame, using the rig calibration set on the pipeline (`Pipeline::setMultiDeviceCalibration()`).
 *
 * The node is strictly metadata-only: pixels, intrinsics and distortion are untouched. Warping an image into another
 * camera's view remains the job of `ImageAlign`, which can be placed after this node.
 *
 * Inputs are exposed as an input map with one output per input, named `input0..inputN-1` / `output0..outputN-1`.
 * A message whose reference frame cannot be resolved to the target frame is a configuration error and throws.
 */
class CoordinateFrameTransform : public NodeCRTP<ThreadedHostNode, CoordinateFrameTransform> {
   public:
    constexpr static const char* NAME = "CoordinateFrameTransform";

    /**
     * Inputs to re-express, one per source. Populated by build().
     */
    InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::Buffer, true}}}, true}};

    /**
     * Re-expressed messages, one per input. `outputN` corresponds to `inputN`.
     */
    OutputMap outputs{*this, "outputs", {"", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Build the node with a fixed set of sources.
     * @param sources Outputs to re-express. Inputs are named input0..inputN-1 in this order
     * @param target Frame all messages are re-expressed in
     */
    std::shared_ptr<CoordinateFrameTransform> build(const std::vector<Node::Output*>& sources, const CoordinateFrame& target);

    /**
     * Build the node with a fixed number of unlinked inputs, to be fed from host queues.
     * @param numInputs Number of inputs, at least one
     * @param target Frame all messages are re-expressed in
     */
    std::shared_ptr<CoordinateFrameTransform> build(size_t numInputs, const CoordinateFrame& target);

    /// Number of inputs the node was built with.
    size_t getNumInputs() const;

    /// Frame all messages are re-expressed in.
    void setTarget(const CoordinateFrame& target);
    CoordinateFrame getTarget() const;

    /**
     * Declare the reference frame of the messages arriving on an input.
     *
     * Only needed for messages whose reference frame is not fully qualified, e.g. frames produced by firmware that
     * does not report the device id, or host-created messages. The declared frame is applied to the message's
     * transformation before it is re-expressed; the socket of the message is kept if it already has one.
     *
     * @param inputIndex Index of the input, as passed to build()
     * @param frame Reference frame of that input's messages
     */
    void setSourceFrame(size_t inputIndex, const CoordinateFrame& frame);

    /**
     * Use a specific rig calibration instead of the one set on the pipeline.
     */
    void setCalibration(const MultiDeviceCalibrationHandler& calibration);

    void buildInternal() override;

   private:
    void run() override;

    /// Rig calibration to use: the explicitly set one, else the pipeline's.
    std::shared_ptr<const MultiDeviceCalibrationHandler> resolveCalibration() const;

    static std::string inputName(size_t index);
    static std::string outputName(size_t index);

    size_t numInputs = 0;
    CoordinateFrame target;
    std::map<size_t, CoordinateFrame> sourceFrames;
    std::shared_ptr<const MultiDeviceCalibrationHandler> calibration;
};

}  // namespace node
}  // namespace dai
