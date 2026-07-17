#pragma once

#include <depthai/common/DeviceModelZoo.hpp>
#include <depthai/pipeline/datatype/ImgDetections.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/pipeline/node/host/HostNode.hpp>

#include <array>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace dai {
namespace node {

class FocusController : public CustomNode<FocusController> {
   public:
    FocusController() = default;

    std::shared_ptr<FocusController> build(float targetFps = 30.0f);

    Input depthCrop{*this, {"depthCrop", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input confidenceCrop{*this,
                         {"confidenceCrop", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output leftConfig{*this, {"leftConfig", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output rightConfig{*this, {"rightConfig", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};

    // The synchronized frames the crops are computed from. Feeding these to the crop
    // ImageManips (instead of the free-running rectification stream) keeps the left and
    // right crops on the same timestamp so the backend's left/right Sync can pair them.
    Output leftImage{*this, {"leftImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output rightImage{*this, {"rightImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    Output gateControlNeural{*this, {"gateControlNeural", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, true}}}}};
    Output gateControlStereo{*this, {"gateControlStereo", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, true}}}}};
    Output gateControlGpu{*this, {"gateControlGpu", DEFAULT_GROUP, {{{DatatypeEnum::GateControl, true}}}}};

    Output confidenceOut{*this, {"confidenceOut", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    std::shared_ptr<Buffer> processGroup(std::shared_ptr<MessageGroup> in) override;

    // Geometry of a single focused region derived from a detection.
    struct Crop {
        // Backend crop: detection box expanded by max disparity horizontally and clamped to the frame.
        int x, y, w, h;
        // Detection box in pixels; the focused output is written back only over this region.
        float detX, detY, detW, detH;
    };

    // Convert normalized {xmin, ymin, xmax, ymax} detection boxes into backend crops for a
    // frameWidth x frameHeight frame. Degenerate or fully out-of-frame boxes are dropped, and
    // crops are clamped so they always stay within the frame. Pure and free of device state so
    // the crop geometry (multiple/varying/edge-touching regions) can be tested on host.
    static std::vector<Crop> computeCrops(int frameWidth, int frameHeight, const std::vector<std::array<float, 4>>& normalizedBoxes);

    // Where a crop's depth/confidence is written back into the full frame. src* index into the
    // crop mat (sized w x h from Crop), dst* index into the frameWidth x frameHeight output, and
    // (w, h) is the copied region. The region is clamped to both the crop mat and the frame, so
    // rounding between the detection box and the crop never overflows either buffer. valid is
    // false when nothing should be copied. Pure so the reassembly bounds are host-testable.
    struct CopyRegion {
        int srcX, srcY, dstX, dstY, w, h;
        bool valid;
    };
    static CopyRegion computeCopyRegion(const Crop& crop, int frameWidth, int frameHeight);

    // Multiplicative correction applied to a crop's backend depth so it becomes metric-correct
    // regardless of crop size. The backend estimates disparity in the crop resized to outW and
    // converts to depth with fxUsed (the focal on the crop frame's intrinsic), but the crop's
    // true focal is fxFull * outW / cropW (fxFull is the full rectified-frame focal in full-frame
    // px). The correction is (fxFull * outW / cropW) / fxUsed. Returns 1.0 when inputs are
    // invalid or the correction is a no-op, so it is safe to always apply. Pure and host-testable.
    static float depthFocalScale(float fxFull, float fxUsed, int outW, int cropW);

    void setTargetFps(float targetFps);
    void setNeuralModel(DeviceModelZoo model);
    void setStereoSize(int width, int height);
    void setStereoFps(float fps);

    constexpr static const char* NAME = "FocusController";

   private:
    enum class Backend { NEURAL, STEREO, GPU };

    float targetFps_ = 30.0f;
    DeviceModelZoo neuralModel_ = DeviceModelZoo::NEURAL_DEPTH_480X300;
    std::pair<int, int> neuralSize_ = {480, 300};
    float neuralFps_ = 56.0f;
    std::pair<int, int> stereoSize_ = {640, 400};
    float stereoFps_ = 30.0f;
};

}  // namespace node
}  // namespace dai
