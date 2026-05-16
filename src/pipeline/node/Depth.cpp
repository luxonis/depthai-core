#include "depthai/pipeline/node/Depth.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {
namespace {

// Depth algorithm selection is device-dependent:
// - TOF requires a connected ToF sensor.
// - NEURAL_ASSISTED_STEREO / GPU_STEREO require RVC4.
// - AUTO prefers NEURAL on RVC4, TOF on RVC2 with ToF, otherwise STEREO.

constexpr std::pair<uint32_t, uint32_t> kStereoDepthMonoSize{640, 400};

/** Fixed NeuralDepth model for ``Algorithm::NEURAL`` and for ``AUTO`` on RVC4. */
constexpr DeviceModelZoo kDefaultNeuralDepthModel = DeviceModelZoo::NEURAL_DEPTH_SMALL;
/** Fixed NeuralAssistedStereo neural model / rectify for ``Algorithm::NEURAL_ASSISTED_STEREO``. */
constexpr DeviceModelZoo kDefaultNasNeuralModel = DeviceModelZoo::NEURAL_DEPTH_NANO;
constexpr bool kDefaultNasRectify = true;

std::pair<std::shared_ptr<Camera>, std::shared_ptr<Camera>> findCamerasForPair(const Pipeline& pipeline, const StereoPair& pair) {
    std::shared_ptr<Camera> left;
    std::shared_ptr<Camera> right;
    for(const auto& node : pipeline.getAllNodes()) {
        if(std::strcmp(node->getName(), Camera::NAME) != 0) {
            continue;
        }
        auto cam = std::static_pointer_cast<Camera>(node);
        const auto socket = cam->getBoardSocket();
        if(socket == pair.left) {
            left = std::move(cam);
        } else if(socket == pair.right) {
            right = std::move(cam);
        }
    }
    return {left, right};
}

StereoPair requireFirstStereoPair(const std::shared_ptr<Device>& device) {
    const auto pairs = device->getStereoPairs();
    DAI_CHECK_V(!pairs.empty(), "Device has no stereo camera pair for Depth node.");
    return pairs[0];
}

/** True if any connected camera advertises CameraSensorType::TOF. */
bool cameraFeaturesIncludeTof(const std::vector<dai::CameraFeatures>& features) {
    for(const auto& cf : features) {
        for(const auto sensorType : cf.supportedTypes) {
            if(sensorType == dai::CameraSensorType::TOF) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace

Depth::Depth(Algorithm algorithm) : DeviceNodeGroup(), algorithmOverride_(algorithm) {}

Depth::Depth() : Depth(Algorithm::AUTO) {}

std::shared_ptr<Depth> Depth::build(std::optional<float> fps) {
    DAI_CHECK_V(!graphBuilt_, "Depth::build(fps) must be called before the graph is wired (before first depth()/confidence() access).");
    stereoOutputFps_ = fps;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm, std::optional<float> fps) {
    DAI_CHECK_V(!graphBuilt_, "Depth::build(algorithm, fps) must be called before the graph is wired (before first depth()/confidence() access).");
    algorithmOverride_ = algorithm;
    stereoOutputFps_ = fps;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

StereoPair Depth::getStereoPair() const {
    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "Depth node requires a device to query its stereo pair.");
    return requireFirstStereoPair(device);
}

std::vector<Depth::Algorithm> Depth::getSupportedAlgorithms(const std::shared_ptr<Device>& device) const {
    std::vector<Algorithm> supported = {Algorithm::STEREO, Algorithm::NEURAL};

    if(device->getPlatform() == Platform::RVC4) {
        supported.push_back(Algorithm::NEURAL_ASSISTED_STEREO);
        supported.push_back(Algorithm::GPU_STEREO);
    }
    if(cameraFeaturesIncludeTof(device->getConnectedCameraFeatures())) {
        supported.push_back(Algorithm::TOF);
    }

    return supported;
}

Depth::Algorithm Depth::selectAlgorithm(const std::shared_ptr<Device>& device) const {
    const auto supported = getSupportedAlgorithms(device);

    if(algorithmOverride_ == Algorithm::AUTO) {
        if(device->getPlatform() == Platform::RVC4) {
            return Algorithm::NEURAL;
        }
        if(device->getPlatform() == Platform::RVC2
           && std::find(supported.begin(), supported.end(), Algorithm::TOF) != supported.end()) {
            return Algorithm::TOF;
        }
        return Algorithm::STEREO;
    }

    if(std::find(supported.begin(), supported.end(), algorithmOverride_) != supported.end()) {
        return algorithmOverride_;
    }

    switch(algorithmOverride_) {
        case Algorithm::NEURAL_ASSISTED_STEREO:
            DAI_CHECK_V(false, "NeuralAssistedStereo is only supported on RVC4.");
            break;
        case Algorithm::GPU_STEREO:
            DAI_CHECK_V(false, "GPUStereo is only supported on RVC4.");
            break;
        case Algorithm::TOF:
            DAI_CHECK_V(false, "Depth Algorithm::TOF requires a connected ToF camera (e.g. OAK-D ToF / OAK-TOF series).");
            break;
        case Algorithm::AUTO:
        case Algorithm::STEREO:
        case Algorithm::NEURAL:
            break;
    }

    DAI_CHECK_V(false, "Depth algorithm is not supported on this device.");
    return Algorithm::STEREO;
}

void Depth::buildInternal() {
    if(graphBuilt_) {
        return;
    }
    if(parent.lock() == nullptr) {
        return;
    }

    Pipeline pipeline = getParentPipeline();
    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "Depth node requires a device (set on create, when added to a pipeline with a default device, or from pipeline at first wiring).");

    const Algorithm active = selectAlgorithm(device);

    switch(active) {
        case Algorithm::AUTO:
            DAI_CHECK_V(false, "Depth: AUTO must be resolved before wiring.");
            break;
        case Algorithm::TOF: {
            tofBackend_ = ToF::create(device);
            add(tofBackend_);
            tofBackend_->build(CameraBoardSocket::AUTO, ImageFiltersPresetMode::TOF_MID_RANGE, stereoOutputFps_);
            depthOut_ = &tofBackend_->depth;
            confidenceOut_ = &tofBackend_->amplitude;
            break;
        }
        case Algorithm::NEURAL_ASSISTED_STEREO: {
            const auto pair = requireFirstStereoPair(device);
            nasBackend_ = std::make_shared<NeuralAssistedStereo>(device);
            add(nasBackend_);
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, pair, std::nullopt, stereoOutputFps_);
            nasBackend_->build(*leftOut, *rightOut, kDefaultNasNeuralModel, kDefaultNasRectify);
            depthOut_ = &nasBackend_->depth;
            confidenceOut_ = &(*nasBackend_->stereoDepth).confidenceMap;
            break;
        }
        case Algorithm::GPU_STEREO: {
            const auto pair = requireFirstStereoPair(device);
            gpuStereoBackend_ = std::make_unique<Subnode<GPUStereo>>(*this, "gpuStereo");
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, pair, kStereoDepthMonoSize, stereoOutputFps_);
            (*gpuStereoBackend_)->setRectification(true).build(*leftOut, *rightOut);
            depthOut_ = &(**gpuStereoBackend_).depth;
            confidenceOut_ = &(**gpuStereoBackend_).disparity;
            break;
        }
        case Algorithm::NEURAL: {
            const auto pair = requireFirstStereoPair(device);
            neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth");
            const auto is = NeuralDepth::getInputSize(kDefaultNeuralDepthModel);
            const std::pair<uint32_t, uint32_t> monoSize{static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, pair, monoSize, stereoOutputFps_);
            (*neuralBackend_)->build(*leftOut, *rightOut, kDefaultNeuralDepthModel);
            depthOut_ = &(**neuralBackend_).depth;
            confidenceOut_ = &(**neuralBackend_).confidence;
            break;
        }
        case Algorithm::STEREO: {
            const auto pair = requireFirstStereoPair(device);
            stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoDepth");
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, pair, kStereoDepthMonoSize, stereoOutputFps_);
            (*stereoBackend_)->build(*leftOut, *rightOut, StereoDepth::PresetMode::DEFAULT);
            depthOut_ = &(**stereoBackend_).depth;
            confidenceOut_ = &(**stereoBackend_).confidenceMap;
            break;
        }
        default:
            DAI_CHECK_V(false, "Depth: no backend was selected for wiring.");
    }

    graphBuilt_ = true;
}

std::pair<Node::Output*, Node::Output*> Depth::ensureStereoOutputs(Pipeline& pipeline,
                                                                   const StereoPair& pair,
                                                                   std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                                                   const std::optional<float>& fps) {
    auto [left, right] = findCamerasForPair(pipeline, pair);

    if(!left) {
        left = pipeline.create<Camera>()->build(pair.left);
    }
    if(!right) {
        right = pipeline.create<Camera>()->build(pair.right);
    }

    Node::Output* lo = nullptr;
    Node::Output* ro = nullptr;
    if(frameSize) {
        lo = left->requestOutput(*frameSize, std::nullopt, ImgResizeMode::CROP, fps);
        ro = right->requestOutput(*frameSize, std::nullopt, ImgResizeMode::CROP, fps);
        DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera stereo output request failed.");
    } else {
        lo = left->requestFullResolutionOutput(std::nullopt, fps, false);
        ro = right->requestFullResolutionOutput(std::nullopt, fps, false);
        DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera full-resolution stereo output request failed.");
    }
    return {lo, ro};
}

Node::Output& Depth::depth() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(depthOut_ != nullptr, "Depth backend output missing.");
    return *depthOut_;
}

Node::Output& Depth::confidence() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(confidenceOut_ != nullptr, "Depth backend confidence output missing.");
    return *confidenceOut_;
}

}  // namespace node
}  // namespace dai
