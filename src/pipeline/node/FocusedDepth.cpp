#include "depthai/pipeline/node/FocusedDepth.hpp"

#include <stdexcept>
#include <string>

namespace dai {
namespace node {

FocusedDepth::FocusedDepth()
    : DeviceNodeGroup(nullptr),
      inputDetections(focusController->inputs["inputDetections"]),
      depth(focusController->out),
      confidence(focusController->confidenceOut),
      focusDebug(focusController->focusDebug) {}

std::shared_ptr<FocusedDepth> FocusedDepth::setFocusMode(FocusController::Mode mode) {
    if(built_) throw std::logic_error("FocusedDepth configuration must be set before build().");
    if(mode != FocusController::Mode::ROI && mode != FocusController::Mode::HOLD && mode != FocusController::Mode::HYBRID) {
        throw std::invalid_argument("Unknown focused depth mode");
    }
    mode_ = mode;
    focusController->setMode(mode);
    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

std::shared_ptr<FocusedDepth> FocusedDepth::setFocusHoldFrames(unsigned int frames) {
    if(built_) throw std::logic_error("FocusedDepth configuration must be set before build().");
    focusController->setHoldFrames(frames);
    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

std::shared_ptr<FocusedDepth> FocusedDepth::setFocusStereoSize(unsigned int width, unsigned int height) {
    if(built_) throw std::logic_error("FocusedDepth configuration must be set before build().");
    if(width == 0 || height == 0 || width % 128 != 0 || width > 1280 || height > 800) {
        throw std::invalid_argument("Focused EVA size must be positive, width divisible by 128, and at most 1280x800");
    }
    stereoSize_ = {width, height};
    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

std::shared_ptr<FocusedDepth> FocusedDepth::setFocusModels(const std::vector<DeviceModelZoo>& models) {
    if(built_) {
        throw std::logic_error("FocusedDepth configuration must be set before build().");
    }
    focusController->setModels(models);
    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

std::shared_ptr<FocusedDepth> FocusedDepth::setFocusSelectionMode(FocusController::SelectionMode mode) {
    if(built_) {
        throw std::logic_error("FocusedDepth configuration must be set before build().");
    }
    focusController->setSelectionMode(mode);
    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

std::shared_ptr<FocusedDepth> FocusedDepth::setFocusDispatchMode(FocusController::DispatchMode mode) {
    if(built_) {
        throw std::logic_error("FocusedDepth configuration must be set before build().");
    }
    focusController->setDispatchMode(mode);
    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

std::shared_ptr<FocusedDepth> FocusedDepth::build(Node::Output& left,
                                                  Node::Output& right,
                                                  std::optional<float> fps,
                                                  std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    if(built_) {
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
    }
    if(mode_ == FocusController::Mode::HYBRID && (!getDevice() || getDevice()->getPlatform() != Platform::RVC4)) {
        throw std::runtime_error("Hybrid focused depth requires an RVC4 device with EVA stereo");
    }
    built_ = true;

    // Synchronize detections with the left/right rectified streams.
    focusController->inputs["left"].setWaitForMessage(true);
    focusController->inputs["right"].setWaitForMessage(true);
    focusController->inputs["inputDetections"].setWaitForMessage(inputDetections.isConnected());

    // Rectification step for the focused sub-flow.
    left.link(rectification->input1);
    right.link(rectification->input2);
    if(resolution) {
        rectification->setOutputSize(resolution->first, resolution->second);
    }

    rectification->output1.link(focusController->inputs["left"]);
    rectification->output2.link(focusController->inputs["right"]);

    if(mode_ == FocusController::Mode::HYBRID) {
        stereoLeft_ = std::make_unique<Subnode<ImageManip>>(*this, "stereoLeft");
        stereoRight_ = std::make_unique<Subnode<ImageManip>>(*this, "stereoRight");
        stereoBase_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoBase");
        for(auto* manip : {&**stereoLeft_, &**stereoRight_}) {
            manip->initialConfig->setOutputSize(stereoSize_.first, stereoSize_.second, ImageManipConfig::ResizeMode::STRETCH);
            manip->initialConfig->setFrameType(ImgFrame::Type::GRAY8);
            // Allow device stride/plane padding for all supported input sizes.
            manip->setMaxOutputFrameSize(2 * 1024 * 1024);
            manip->setNumFramesPool(4);
        }
        rectification->output1.link((*stereoLeft_)->inputImage);
        rectification->output2.link((*stereoRight_)->inputImage);
        (*stereoLeft_)->out.link((*stereoBase_)->left);
        (*stereoRight_)->out.link((*stereoBase_)->right);
        // RVC4 StereoDepth uses EVA. Both backends share rectification, so fusion only needs resizing.
        (*stereoBase_)->setDefaultProfilePreset(StereoDepth::PresetMode::FAST_ACCURACY);
        (*stereoBase_)->setRectification(false);
        (*stereoBase_)->setDepthAlign(StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
        (*stereoBase_)->depth.link(focusController->inputs["baseDepth"]);
        (*stereoBase_)->confidenceMap.link(focusController->inputs["baseConfidence"]);
        focusController->inputs["baseDepth"].setWaitForMessage(true);
        focusController->inputs["baseConfidence"].setWaitForMessage(true);
    }

    for(int tier = 0; tier < focusController->getTierCount(); ++tier) {
        const auto suffix = std::to_string(tier);
        leftManips.push_back(std::make_unique<Subnode<ImageManip>>(*this, "leftManip" + suffix));
        rightManips.push_back(std::make_unique<Subnode<ImageManip>>(*this, "rightManip" + suffix));
        neuralDepths.push_back(std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth" + suffix));
        auto* lm = &**leftManips.back();
        auto* rm = &**rightManips.back();
        auto* nd = &**neuralDepths.back();

        // Crop the exact synchronized pair the controller emits (not the free-running rectification
        // stream), so a tier's left/right crops share a timestamp for its backend Sync. The first
        // config each frame consumes the image; later crops on the tier reuse it.
        focusController->leftImage.link(lm->inputImage);
        focusController->rightImage.link(rm->inputImage);

        lm->inputConfig.setWaitForMessage(true);
        rm->inputConfig.setWaitForMessage(true);
        // All of a frame's crop configs are dispatched up front, so the config queue must hold them
        // (one round-trip per crop was the old bottleneck; this lets the manips + backend pipeline).
        lm->inputConfig.setMaxSize(FocusController::kMaxCropsPerFrame);
        rm->inputConfig.setMaxSize(FocusController::kMaxCropsPerFrame);
        // Single-model dispatch keeps frames in order; inactive multi-model tiers keep only the latest.
        lm->inputImage.setBlocking(focusController->getTierCount() == 1);
        rm->inputImage.setBlocking(focusController->getTierCount() == 1);
        lm->inputImage.setMaxSize(focusController->getTierCount() == 1 ? 4 : 1);
        rm->inputImage.setMaxSize(focusController->getTierCount() == 1 ? 4 : 1);
        lm->setMaxOutputFrameSize(8 * 1024 * 1024);
        rm->setMaxOutputFrameSize(8 * 1024 * 1024);
        lm->setNumFramesPool(4);
        rm->setNumFramesPool(4);

        focusController->leftConfigTier(tier).link(lm->inputConfig);
        focusController->rightConfigTier(tier).link(rm->inputConfig);

        const auto& tierConfig = focusController->getTiers()[tier];
        nd->setRectification(false).build(lm->out, rm->out, tierConfig.model);
        nd->depth.link(focusController->depthCropTier(tier));
        nd->confidence.link(focusController->confidenceCropTier(tier));
    }

    focusController->setTargetFps(fps.value_or(30.0f));

    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

}  // namespace node
}  // namespace dai
