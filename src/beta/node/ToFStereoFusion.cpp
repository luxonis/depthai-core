#include "depthai/beta/node/ToFStereoFusion.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "utility/Logging.hpp"

namespace dai::beta {

ToFStereoFusionProperties::~ToFStereoFusionProperties() = default;

}  // namespace dai::beta

namespace dai::beta::node {

ToFStereoFusion::ToFStereoFusion(const std::shared_ptr<Device>& device)
    : DeviceNodeCRTP(device), initialConfig(std::make_shared<ToFStereoFusionConfig>(properties.initialConfig)) {
    if(device && device->getPlatform() != Platform::RVC4) {
        throw std::runtime_error("ToFStereoFusion is only supported on RVC4 devices.");
    }
}

ToFStereoFusion::~ToFStereoFusion() = default;

ToFStereoFusion::Properties& ToFStereoFusion::getProperties() {
    properties.initialConfig = *initialConfig;
    // initialConfig is usually set after build(): check it when the pipeline is serialized.
    if(properties.initialConfig.cropToOverlap) {
        logger::warn("ToFStereoFusion: cropToOverlap is not supported; the output is the full rectified-left view");
    }
    return properties;
}

namespace {

// The network shipped with the RVC4 firmware and its input size.
constexpr const char* FIRMWARE_NETWORK = "TOF_STEREO_FUSION_1024x640.dlc";
constexpr std::pair<uint32_t, uint32_t> NETWORK_INPUT_SIZE{1024, 640};
// Stereo pairs and ToF groups share syncedInputs; room for both at 30 fps while the node waits for a free slot.
constexpr unsigned SYNCED_INPUTS_QUEUE_SIZE = 8;

}  // namespace

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
    // The network ships with the RVC4 firmware; a model set with neuralNetwork->setModelPath() takes precedence. Network I/O
    // as dma-bufs (one per tensor), registered with SNPE: the device pre- and post-processing read and write them in place.
    neuralNetwork->setBackendProperties(
        {{"firmware_model", FIRMWARE_NETWORK}, {"output_tensors", "disp_low,head9,conf"}, {"output_memory", "dma_buf"}, {"register_dma_buf", "true"}});
}

std::shared_ptr<ToFStereoFusion> ToFStereoFusion::build(const std::shared_ptr<dai::node::Camera>& left,
                                                        const std::shared_ptr<dai::node::Camera>& right,
                                                        float fps) {
    if(!std::isfinite(fps) || fps <= 0.0f) {
        throw std::invalid_argument("ToFStereoFusion FPS must be finite and positive");
    }
    if(!left || !right) {
        throw std::invalid_argument("ToFStereoFusion requires two camera nodes");
    }
    if(rectification) {
        throw std::logic_error("ToFStereoFusion::build() may only be called once");
    }

    const auto leftSize = std::make_pair(left->getMaxWidth(), left->getMaxHeight());
    const auto rightSize = std::make_pair(right->getMaxWidth(), right->getMaxHeight());
    if(leftSize != rightSize) {
        throw std::invalid_argument("ToFStereoFusion camera nodes must have matching maximum resolutions");
    }
    const auto leftFps = left->properties.fps;
    const auto rightFps = right->properties.fps;
    if((leftFps != CameraProperties::AUTO && leftFps < fps) || (rightFps != CameraProperties::AUTO && rightFps < fps)) {
        throw std::invalid_argument("ToFStereoFusion camera FPS must not be below fusion FPS");
    }
    if(leftFps != CameraProperties::AUTO && rightFps != CameraProperties::AUTO && leftFps != rightFps) {
        logger::warn("ToFStereoFusion camera FPS values differ (left: {}, right: {}); outputs may not synchronize", leftFps, rightFps);
    }
    // NeuralDepth is not used; drop it so that its network is not loaded.
    remove(neuralDepth->shared_from_this());

    pairSync = std::make_unique<Subnode<dai::node::Sync>>(*this, "pairSync");
    pairDemux = std::make_unique<Subnode<dai::node::MessageDemux>>(*this, "pairDemux");
    rectification = std::make_unique<Subnode<dai::node::Rectification>>(*this, "rectification");
    (*pairSync)->setRunOnHost(false);
    (*pairSync)->out.link((*pairDemux)->input);
    (*pairDemux)->outputs["left"].link((*rectification)->input1);
    (*pairDemux)->outputs["right"].link((*rectification)->input2);
    (*rectification)->setOutputSize(NETWORK_INPUT_SIZE);
    // EVA warp (CPU fallback for layouts it cannot take). Alpha 0 is the rectified camera the network was trained on.
    (*rectification)->setBackend(dai::node::Rectification::Backend::HW);
    (*rectification)->setAlphaScaling(0.0f);

    // Scale, do not crop: the network was trained on the full field of view. Rectification also undistorts.
    auto* leftOutput = left->requestOutput(NETWORK_INPUT_SIZE, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, fps, false);
    auto* rightOutput = right->requestOutput(NETWORK_INPUT_SIZE, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, fps, false);
    leftOutput->link((*pairSync)->inputs["left"]);
    rightOutput->link((*pairSync)->inputs["right"]);
    (*rectification)->output1.link(sync->inputs["left"]);
    (*rectification)->output2.link(sync->inputs["right"]);

    // The ToF planes of one ToF frame (same timestamp) are grouped on their own, not with the stereo pair, and reach the node
    // through syncedInputs too (the device tells the groups apart by their keys): the node fuses each stereo frame with the
    // newest ToF frame it has, so the slower ToF processing does not hold the stereo frames back.
    tofSync = std::make_unique<Subnode<dai::node::Sync>>(*this, "tofSync");
    (*tofSync)->setRunOnHost(false);
    (*tofSync)->setSyncThreshold(std::chrono::milliseconds(1));
    (*tofSync)->out.link(syncedInputs);
    syncedInputs.setMaxSize(SYNCED_INPUTS_QUEUE_SIZE);
    tof->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, fps);
    tof->setOutputUndistortion(true);
    tof->tofBaseNode.depth.link((*tofSync)->inputs["tofDepth"]);
    tof->tofBaseNode.confidence.link((*tofSync)->inputs["tofConfidence"]);
    tof->tofBaseNode.amplitude.link((*tofSync)->inputs["tofAmplitude"]);
    // A linked phase output puts the ToF into the fusion depth mode, the ToF state the network is trained on.
    tof->tofBaseNode.phase.link((*tofSync)->inputs["tofPhase"]);
    return std::static_pointer_cast<ToFStereoFusion>(shared_from_this());
}

}  // namespace dai::beta::node
