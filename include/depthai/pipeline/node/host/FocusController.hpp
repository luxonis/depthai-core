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

    enum class SelectionMode { ALL, LARGEST };
    enum class DispatchMode { SINGLE_TIER_PER_FRAME, TIME_BUDGET };

    std::shared_ptr<FocusController> build(float targetFps = 30.0f);

    // A backend tier: a NeuralDepth model of a fixed input size. Crops are routed to the smallest
    // tier that fits them (selectTier), so a small detection uses the fast small model and a large
    // one uses a larger model instead of stretching every crop to a single size.
    struct Tier {
        DeviceModelZoo model;
        int w;
        int h;
    };
    // Sizes are capped at 768x480: on RVC4 the largest models (e.g. 1248x780) run at ~8.5 fps, so a
    // large crop is downscaled to 768x480 (with per-crop focal correction keeping depth metric)
    // rather than routed to a much slower model. The smallest tier is a fast model for small scenes.
    static constexpr int kNumTiers = 3;
    static constexpr std::array<Tier, kNumTiers> kTiers{{
        {DeviceModelZoo::NEURAL_DEPTH_288X180, 288, 180},
        {DeviceModelZoo::NEURAL_DEPTH_480X300, 480, 300},
        {DeviceModelZoo::NEURAL_DEPTH_768X480, 768, 480},
    }};

    void setModels(const std::vector<DeviceModelZoo>& models);
    const std::array<Tier, kNumTiers>& getTiers() const {
        return tiers_;
    }
    int getTierCount() const {
        return tierCount_;
    }
    void setSelectionMode(SelectionMode mode) {
        selectionMode_ = mode;
    }
    SelectionMode getSelectionMode() const {
        return selectionMode_;
    }
    void setDispatchMode(DispatchMode mode) {
        dispatchMode_ = mode;
    }
    DispatchMode getDispatchMode() const {
        return dispatchMode_;
    }

    // Upper bound on merged crops processed per frame. Also the depth/confidence crop input queue
    // depth, so every crop dispatched in a frame can be buffered before it is collected (the
    // dispatch-all-then-collect pipeline would otherwise deadlock if a tier's results piled up).
    static constexpr int kMaxCropsPerFrame = 24;

    // One depth/confidence crop input and one left/right config output per tier. Each tier has its
    // own fixed-size crop ImageManips and NeuralDepth backend, so left/right crops always match in
    // size (no backend size-mismatch crash) and different sizes never share a manip.
    Input depthCrop0{*this, {"depthCrop0", DEFAULT_GROUP, DEFAULT_BLOCKING, kMaxCropsPerFrame, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input depthCrop1{*this, {"depthCrop1", DEFAULT_GROUP, DEFAULT_BLOCKING, kMaxCropsPerFrame, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input depthCrop2{*this, {"depthCrop2", DEFAULT_GROUP, DEFAULT_BLOCKING, kMaxCropsPerFrame, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input confidenceCrop0{*this, {"confidenceCrop0", DEFAULT_GROUP, DEFAULT_BLOCKING, kMaxCropsPerFrame, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input confidenceCrop1{*this, {"confidenceCrop1", DEFAULT_GROUP, DEFAULT_BLOCKING, kMaxCropsPerFrame, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};
    Input confidenceCrop2{*this, {"confidenceCrop2", DEFAULT_GROUP, DEFAULT_BLOCKING, kMaxCropsPerFrame, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output leftConfig0{*this, {"leftConfig0", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output leftConfig1{*this, {"leftConfig1", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output leftConfig2{*this, {"leftConfig2", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output rightConfig0{*this, {"rightConfig0", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output rightConfig1{*this, {"rightConfig1", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};
    Output rightConfig2{*this, {"rightConfig2", DEFAULT_GROUP, {{{DatatypeEnum::ImageManipConfig, true}}}}};

    // The synchronized frames the crops are computed from. Broadcast to every tier's crop
    // ImageManips (instead of the free-running rectification stream) so the left and right crops
    // stay on the same timestamp and each backend's left/right Sync can pair them.
    Output leftImage{*this, {"leftImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output rightImage{*this, {"rightImage", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    Output confidenceOut{*this, {"confidenceOut", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    // Per-tier port accessors (tier in [0, kNumTiers)).
    Input& depthCropTier(int tier);
    Input& confidenceCropTier(int tier);
    Output& leftConfigTier(int tier);
    Output& rightConfigTier(int tier);

    std::shared_ptr<Buffer> processGroup(std::shared_ptr<MessageGroup> in) override;

    // Geometry of a single focused region derived from a detection.
    struct Crop {
        // Backend crop: detection box expanded by max disparity horizontally and clamped to the frame.
        int x, y, w, h;
        // Detection box in pixels; the focused output is written back only over this region.
        float detX, detY, detW, detH;
    };

    // A crop shared by one or more detections. Disparity padding makes neighbouring detections'
    // crops overlap; merging them into the union rectangle runs one inference instead of several,
    // while the focused output is still written back only over each detection's own box.
    struct MergedCrop {
        int x, y, w, h;
        // Detection boxes covered by this crop, each {detX, detY, detW, detH} in pixels.
        std::vector<std::array<float, 4>> dets;
    };

    // Convert normalized {xmin, ymin, xmax, ymax} detection boxes into backend crops for a
    // frameWidth x frameHeight frame. Degenerate or fully out-of-frame boxes are dropped, and
    // crops are clamped so they always stay within the frame. Pure and free of device state so
    // the crop geometry (multiple/varying/edge-touching regions) can be tested on host.
    static std::vector<Crop> computeCrops(int frameWidth, int frameHeight, const std::vector<std::array<float, 4>>& normalizedBoxes);
    static std::vector<std::array<float, 4>> selectLargest(const std::vector<std::array<float, 4>>& normalizedBoxes);

    // Merge crops whose backend rectangles overlap into their bounding union, accumulating the
    // detection boxes each union covers. Non-overlapping crops become single-detection merged
    // crops. The result has no two overlapping rectangles. Pure and host-testable.
    static std::vector<MergedCrop> mergeCrops(const std::vector<Crop>& crops);

    // Smallest tier whose model fits a cropW x cropH crop; the largest tier if none fit. Pure.
    static int selectTier(int cropW, int cropH);
    static int selectTier(const std::array<Tier, kNumTiers>& tiers, int cropW, int cropH);
    static int selectTier(const std::array<Tier, kNumTiers>& tiers, int tierCount, int cropW, int cropH);

    // Return crops in descending area order. Pure so dispatch ordering can be tested without a device.
    static std::vector<MergedCrop> orderCropsByArea(const std::vector<MergedCrop>& crops);

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

    constexpr static const char* NAME = "FocusController";

   private:
    float targetFps_ = 30.0f;
    std::array<Tier, kNumTiers> tiers_ = kTiers;
    int tierCount_ = kNumTiers;
    SelectionMode selectionMode_ = SelectionMode::ALL;
    DispatchMode dispatchMode_ = DispatchMode::SINGLE_TIER_PER_FRAME;
};

}  // namespace node
}  // namespace dai
