#include "depthai/beta/node/ToFStereoFusion.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

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
    return properties;
}

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
}

ToFStereoFusion& ToFStereoFusion::setModel(Properties::Model model) {
    properties.model = model;
    return *this;
}

ToFStereoFusion::Properties::Model ToFStereoFusion::getModel() const {
    return properties.model;
}

ToFStereoFusion& ToFStereoFusion::setBackend(Properties::Backend backend) {
    properties.backend = backend;
    return *this;
}

ToFStereoFusion& ToFStereoFusion::setToFConfidenceScale(Properties::ToFConfidenceScale scale) {
    properties.tofConfidenceScale = scale;
    return *this;
}

ToFStereoFusion& ToFStereoFusion::setNetworkInputSize(uint32_t width, uint32_t height) {
    // The network packs its input in 1/4-resolution planes and the correlation operator works on rows of 32 cells.
    if(width < 512 || height < 256 || width % 128 != 0 || height % 32 != 0) {
        throw std::invalid_argument("ToFStereoFusion network input must be at least 512x256, width a multiple of 128 and height a multiple of 32");
    }
    networkInputSize = {width, height};
    return *this;
}

void ToFStereoFusion::buildStereoNetwork(const std::shared_ptr<dai::node::Camera>& left, const std::shared_ptr<dai::node::Camera>& right, float fps) {
    // NeuralDepth is not used by this model; drop it so that its network is not loaded.
    remove(neuralDepth->shared_from_this());

    pairSync = std::make_unique<Subnode<dai::node::Sync>>(*this, "pairSync");
    pairDemux = std::make_unique<Subnode<dai::node::MessageDemux>>(*this, "pairDemux");
    rectification = std::make_unique<Subnode<dai::node::Rectification>>(*this, "rectification");
    (*pairSync)->setRunOnHost(false);
    (*pairSync)->out.link((*pairDemux)->input);
    (*pairDemux)->outputs["left"].link((*rectification)->input1);
    (*pairDemux)->outputs["right"].link((*rectification)->input2);
    (*rectification)->setOutputSize(networkInputSize);

    // Scale, do not crop: the network was trained on the full field of view. Rectification also undistorts.
    auto* leftOutput = left->requestOutput(networkInputSize, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, fps, false);
    auto* rightOutput = right->requestOutput(networkInputSize, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, fps, false);
    leftOutput->link((*pairSync)->inputs["left"]);
    rightOutput->link((*pairSync)->inputs["right"]);
    (*rectification)->output1.link(sync->inputs["left"]);
    (*rectification)->output2.link(sync->inputs["right"]);

    tof->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, fps);
    tof->setOutputUndistortion(true);
    tof->tofBaseNode.depth.link(sync->inputs["tofDepth"]);
    tof->tofBaseNode.confidence.link(sync->inputs["tofConfidence"]);
    tof->tofBaseNode.amplitude.link(sync->inputs["tofAmplitude"]);
    // The network is trained on the ToF fusion depth mode; its phases replace the ones derived from the depth.
    if(properties.tofConfidenceScale == Properties::ToFConfidenceScale::FUSION) {
        tof->tofBaseNode.initialConfig->setDepthMode(ToFConfig::DepthMode::FUSION);
        tof->tofBaseNode.phase.link(sync->inputs["tofPhase"]);
    }

    // The network replaces the zoo model set in buildInternal(); the user provides it with neuralNetwork->setModelPath().
    neuralNetwork->properties.deviceModel.reset();
    // Network I/O as dma-bufs (one per tensor), registered with SNPE: the device pre- and post-processing read and write
    // them in place.
    neuralNetwork->setBackendProperties({{"output_tensors", "disp_low,head9,conf"}, {"output_memory", "dma_buf"}, {"register_dma_buf", "true"}});
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

    if(properties.model == Properties::Model::STEREO_NETWORK) {
        buildStereoNetwork(left, right, fps);
        return std::static_pointer_cast<ToFStereoFusion>(shared_from_this());
    }

    constexpr auto neuralDepthModel = DeviceModelZoo::NEURAL_DEPTH_MEDIUM;
    const auto neuralDepthInputSize = dai::node::NeuralDepth::getInputSize(neuralDepthModel);
    auto* leftOutput = left->requestOutput(neuralDepthInputSize, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, fps, true, 0.0f);
    auto* rightOutput = right->requestOutput(neuralDepthInputSize, ImgFrame::Type::GRAY8, ImgResizeMode::STRETCH, fps, true, 0.0f);
    neuralDepth->build(*leftOutput, *rightOutput, neuralDepthModel);
    neuralDepth->initialConfig->setConfidenceThreshold(209);
#ifdef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftOutput->link(neuralDepth->sync->inputs["left"]);
    rightOutput->link(neuralDepth->sync->inputs["right"]);
#endif
    tof->build(CameraBoardSocket::AUTO, ToFConfig::Profile::MID_RANGE, fps);
    tof->setOutputUndistortion(true);
    neuralDepth->depth.link(sync->inputs["neuralDepth"]);
    neuralDepth->confidence.link(sync->inputs["neuralConfidence"]);
    tof->tofBaseNode.depth.link(sync->inputs["tofDepth"]);
    tof->tofBaseNode.confidence.link(sync->inputs["tofConfidence"]);
    return std::static_pointer_cast<ToFStereoFusion>(shared_from_this());
}

}  // namespace dai::beta::node
