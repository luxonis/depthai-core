#include "depthai/beta/node/ToFStereoFusion.hpp"

#include <chrono>
#include <stdexcept>

namespace dai::beta::node {

ToFStereoFusion::ToFStereoFusion(const std::shared_ptr<Device>& device) : DeviceNodeCRTP(device) {
    if(device && device->getPlatform() != Platform::RVC4) {
        throw std::runtime_error("ToFStereoFusion is only supported on RVC4 devices.");
    }
}

ToFStereoFusion::~ToFStereoFusion() = default;

void ToFStereoFusion::buildInternal() {
    if(getDevice() && getDevice()->getPlatform() != Platform::RVC4) {
        throw std::runtime_error("ToFStereoFusion is only supported on RVC4 devices.");
    }
    sync->out.link(syncedInputs);
    sync->setRunOnHost(false);
    sync->setSyncThreshold(std::chrono::milliseconds(50));
    nnInput.link(neuralNetwork->input);
    neuralNetwork->out.link(nnDataInput);
    nnDataInput.setBlocking(false);
    neuralNetwork->setModelFromDeviceZoo(DeviceModelZoo::TOF_NEURAL_FUSION_672X804);
    neuralNetwork->setBackendProperties({{"output_tensors", "logits_a,logits_spec"}});
    neuralDepthAlign->setRunOnHost(false);
}

std::shared_ptr<ToFStereoFusion> ToFStereoFusion::build(const std::shared_ptr<dai::node::Camera>& left, const std::shared_ptr<dai::node::Camera>& right) {
    if(!left || !right) {
        throw std::invalid_argument("ToFStereoFusion requires two camera nodes");
    }

    const auto leftSize = std::make_pair(left->getMaxWidth(), left->getMaxHeight());
    const auto rightSize = std::make_pair(right->getMaxWidth(), right->getMaxHeight());
    if(leftSize != rightSize) {
        throw std::invalid_argument("ToFStereoFusion camera nodes must have matching maximum resolutions");
    }

    constexpr auto neuralDepthModel = DeviceModelZoo::NEURAL_DEPTH_MEDIUM;
    const auto neuralDepthInputSize = dai::node::NeuralDepth::getInputSize(neuralDepthModel);
    auto* leftOutput = left->requestOutput(neuralDepthInputSize, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, std::nullopt, true, 0.0f);
    auto* rightOutput = right->requestOutput(neuralDepthInputSize, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, std::nullopt, true, 0.0f);
    neuralDepth->build(*leftOutput, *rightOutput, neuralDepthModel);
    neuralDepth->initialConfig->setConfidenceThreshold(209);
#ifdef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftOutput->link(neuralDepth->sync->inputs["left"]);
    rightOutput->link(neuralDepth->sync->inputs["right"]);
#endif
    tof->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, 30.0f);
    tof->setOutputUndistortion(true);
    neuralDepth->depth.link(neuralDepthAlign->input);
    tof->tofBaseNode.depth.link(sync->inputs["tofDepth"]);
    tof->tofBaseNode.confidence.link(sync->inputs["tofConfidence"]);
    tof->tofBaseNode.intensity.link(sync->inputs["tofIntensity"]);
    tof->tofBaseNode.depth.link(neuralDepthAlign->inputAlignTo);
    neuralDepthAlign->outputAligned.link(sync->inputs["neuralDepth"]);
    return std::static_pointer_cast<ToFStereoFusion>(shared_from_this());
}

}  // namespace dai::beta::node
