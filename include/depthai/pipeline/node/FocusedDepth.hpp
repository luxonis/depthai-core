#pragma once

#include <depthai/common/optional.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/ImageManip.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/host/FocusController.hpp>
#include <vector>

namespace dai {
namespace node {

class FocusedDepth : public DeviceNodeGroup {
   public:
    FocusedDepth();
    ~FocusedDepth() override = default;

    std::shared_ptr<FocusedDepth> build(Node::Output& left,
                                        Node::Output& right,
                                        std::optional<float> fps = std::nullopt,
                                        std::optional<std::pair<uint32_t, uint32_t>> resolution = std::nullopt);

    /**
     * Select ROI-only, bounded ROI holding, or full-frame EVA stereo with neural ROI enhancement.
     * Must be called before focused outputs are wired. Default: ROI. HYBRID requires RVC4.
     * All modes wait for detection messages; HOLD bridges empty messages, not a stalled detector.
     */
    std::shared_ptr<FocusedDepth> setFocusMode(FocusController::Mode mode);

    /** Maximum consecutive empty detection messages to bridge in HOLD mode. Default: 2; zero disables holding. */
    std::shared_ptr<FocusedDepth> setFocusHoldFrames(unsigned int frames);

    /** EVA input size in HYBRID mode (default 384x240). Must be positive, width divisible by 128, and at most 1280x800. */
    std::shared_ptr<FocusedDepth> setFocusStereoSize(unsigned int width, unsigned int height);

    std::shared_ptr<FocusedDepth> setFocusModels(const std::vector<DeviceModelZoo>& models);
    std::shared_ptr<FocusedDepth> setFocusSelectionMode(FocusController::SelectionMode mode);
    std::shared_ptr<FocusedDepth> setFocusDispatchMode(FocusController::DispatchMode mode);

    constexpr static const char* NAME = "FocusedDepth";

   private:
    bool built_ = false;
    FocusController::Mode mode_ = FocusController::Mode::ROI;
    std::pair<unsigned int, unsigned int> stereoSize_{384, 240};
    std::unique_ptr<Subnode<ImageManip>> stereoLeft_;
    std::unique_ptr<Subnode<ImageManip>> stereoRight_;
    std::unique_ptr<Subnode<StereoDepth>> stereoBase_;

    // NOTE: the Subnodes must be declared before the Input&/Output& reference
    // members below, because the constructor initializes those references from
    // focusController. Members are initialized in declaration order, so
    // focusController has to exist first.
    Subnode<FocusController> focusController{*this, "focusController"};
    Subnode<Rectification> rectification{*this, "rectification"};

    // Allocate only the configured tiers; a single-model pipeline needs one backend.
    std::vector<std::unique_ptr<Subnode<ImageManip>>> leftManips;
    std::vector<std::unique_ptr<Subnode<ImageManip>>> rightManips;
    std::vector<std::unique_ptr<Subnode<NeuralDepth>>> neuralDepths;

   public:
    Input& inputDetections;
    Output& depth;
    Output& confidence;
    Output& focusDebug;
};

}  // namespace node
}  // namespace dai
