#include "depthai/pipeline/node/FocusedDepth.hpp"

namespace dai {
namespace node {

FocusedDepth::FocusedDepth()
    : DeviceNodeGroup(nullptr),
      inputDetections(focusController->inputs["inputDetections"]),
      depth(focusController->out),
      confidence(focusController->confidenceOut) {}

std::shared_ptr<FocusedDepth> FocusedDepth::build(Node::Output& left,
                                                  Node::Output& right,
                                                  std::optional<float> fps,
                                                  std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    if(built_) {
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
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

    ImageManip* leftManips[FocusController::kNumTiers] = {&*leftManip0, &*leftManip1, &*leftManip2};
    ImageManip* rightManips[FocusController::kNumTiers] = {&*rightManip0, &*rightManip1, &*rightManip2};
    NeuralDepth* neurals[FocusController::kNumTiers] = {&*neuralDepth0, &*neuralDepth1, &*neuralDepth2};

    for(int tier = 0; tier < FocusController::kNumTiers; ++tier) {
        ImageManip* lm = leftManips[tier];
        ImageManip* rm = rightManips[tier];
        NeuralDepth* nd = neurals[tier];

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
        // The image is broadcast once per frame and reused across the frame's crops; keep the latest.
        lm->inputImage.setBlocking(false);
        rm->inputImage.setBlocking(false);
        lm->inputImage.setMaxSize(1);
        rm->inputImage.setMaxSize(1);
        lm->setMaxOutputFrameSize(8 * 1024 * 1024);
        rm->setMaxOutputFrameSize(8 * 1024 * 1024);
        lm->setNumFramesPool(4);
        rm->setNumFramesPool(4);

        focusController->leftConfigTier(tier).link(lm->inputConfig);
        focusController->rightConfigTier(tier).link(rm->inputConfig);

        nd->setRectification(false).build(lm->out, rm->out, FocusController::kTiers[tier].model);
        nd->depth.link(focusController->depthCropTier(tier));
        nd->confidence.link(focusController->confidenceCropTier(tier));
    }

    focusController->setTargetFps(fps.value_or(30.0f));

    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

}  // namespace node
}  // namespace dai
