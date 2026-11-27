#pragma once
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/properties/OpticalFlowProperties.hpp"
namespace dai {
namespace node {
/**
 * Native bidirectional dense optical flow on RVC4, without feature detection or
 * track maintenance. Requires matching device firmware; no RVC2/CPU fallback.
 * See docs/optical_flow.md for the packet and hardware precision contract.
 */
class OpticalFlow : public DeviceNodeCRTP<DeviceNode, OpticalFlow, OpticalFlowProperties> {
   public:
    constexpr static const char* NAME = "OpticalFlow";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    /**
     * GRAY8/RAW8 input. Width: 128..1920, divisible by 128; height: 128..1056,
     * divisible by 32. Rows must be packed (stride == width, including implicit
     * stride 0), first-plane and memory offsets zero, payload at least width * height bytes.
     * Trailing allocation padding is allowed. Geometry and format must stay fixed.
     * Blocking queue, size 4. Pairs use consecutive consumed images, including
     * sequence gaps if the queue is explicitly made nonblocking.
     */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::ImgFrame, false}}}, true}};
    /**
     * Owning native MessageGroup per pair; none on the first frame. Contains
     * "flow" NNData: FP32 HWC [height,width,2] "forward" (reference->target)
     * and "backward" (target->reference), dx/dy in input pixels per pair;
     * U8 [height,width,1] "confidence", one raw score map (not a probability).
     * "reference" and "target" are metadata-only Buffers preserving exact pair
     * sequence numbers and timestamps. Group/NNData metadata identify target.
     * FP32 preserves backend precision, not necessarily subpixel estimation.
     * No CPU interpolation or masking. Unconnected output runs no accelerator.
     */
    Output outputFlow{*this, {"outputFlow", DEFAULT_GROUP, {{{DatatypeEnum::MessageGroup, false}}}}};
    /** Every accepted input image, including the first frame. */
    Output passthroughInputImage{*this, {"passthroughInputImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    /** Validate the target platform during pipeline construction. */
    void buildInternal() override;
};
}  // namespace node
}  // namespace dai
