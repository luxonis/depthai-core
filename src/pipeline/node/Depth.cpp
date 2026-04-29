#include "depthai/pipeline/node/Depth.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <string>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {
namespace {

// ---------------------------------------------------------------------------
// Depth algorithm guards (validateAlgorithm)
//
// TOF: require an actual ToF sensor in getConnectedCameraFeatures() (same idea
// as the ToF node), not only a product name.
// NeuralAssistedStereo / GPUStereo: platform and build rules; GPUStereo also
// uses a product-name heuristic for Lite SKUs without the GPU stereo path.
// ---------------------------------------------------------------------------

constexpr float kStereoMonoFps = 30.f;
constexpr std::pair<uint32_t, uint32_t> kStereoDepthMonoSize{640, 400};

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

/** Safe wrapper: missing device or RPC errors mean no ToF for validation. */
bool deviceHasTofSensor(const std::shared_ptr<Device>& device) {
    if(device == nullptr) {
        return false;
    }
    try {
        return cameraFeaturesIncludeTof(device->getConnectedCameraFeatures());
    } catch(...) {
        return false;
    }
}

#if defined(DEPTHAI_ENABLE_KOMPUTE)
/** RVC4 + Kompute build; excludes Lite-class product names (no GPU block). */
bool deviceGpuStereoSupported(const std::shared_ptr<Device>& device) {
    if(device == nullptr || device->getPlatform() != Platform::RVC4) {
        return false;
    }
    try {
        std::string product = device->getProductName();
        std::transform(product.begin(), product.end(), product.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        // RVC4 "Lite" SKUs ship without the GPU used by GPUStereo; extend this list as product names are finalized.
        if(product.find("lite") != std::string::npos) {
            return false;
        }
    } catch(...) {
        // If product metadata is unavailable, allow GPUStereo on RVC4 when Kompute is enabled.
    }
    return true;
}
#endif

}  // namespace

Depth::Depth(const std::shared_ptr<Device>& device, Algorithm algorithm) : DeviceNodeGroup(device), algorithmOverride_(algorithm) {
    DAI_CHECK_V(device != nullptr, "Depth node requires a device.");
}

Depth::Algorithm Depth::resolveAlgorithm(const std::shared_ptr<Device>& device) const {
    if(algorithmOverride_ != Algorithm::AUTO) {
        return algorithmOverride_;
    }
    if(device->getPlatform() == Platform::RVC4) {
        return Algorithm::NEURAL;
    }
    return Algorithm::STEREO;
}

void Depth::validateAlgorithm(const std::shared_ptr<Device>& device, Algorithm active) const {
    // Fail fast before buildInternal wires backends (setAlgorithm / create(..., algorithm)).
    switch(active) {
        case Algorithm::AUTO:
            break;
        case Algorithm::STEREO:
        case Algorithm::NEURAL:
            break;
        case Algorithm::NEURAL_ASSISTED_STEREO:
            DAI_CHECK_V(device->getPlatform() == Platform::RVC4, "NeuralAssistedStereo is only supported on RVC4.");
            break;
        case Algorithm::GPU_STEREO:
#if defined(DEPTHAI_ENABLE_KOMPUTE)
            DAI_CHECK_V(device->getPlatform() == Platform::RVC4, "GPUStereo is only supported on RVC4.");
            DAI_CHECK_V(deviceGpuStereoSupported(device),
                        "GPUStereo requires an RVC4 device with GPU stereo support (Kompute-enabled build and a SKU that includes the GPU block; Lite variants "
                        "are excluded).");
#else
            DAI_CHECK_V(false, "GPUStereo requires depthai-core built with Kompute (DEPTHAI_ENABLE_KOMPUTE).");
#endif
            break;
        case Algorithm::TOF:
            DAI_CHECK_V(deviceHasTofSensor(device),
                        "Depth Algorithm::TOF requires a connected ToF camera (e.g. OAK-D ToF / OAK-TOF series).");
            break;
    }
}

void Depth::buildInternal() {
    if(graphBuilt_) {
        return;
    }
    if(parent.lock() == nullptr) {
        return;
    }

    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "Depth node requires a device.");

    Pipeline pipeline = getParentPipeline();
    DAI_CHECK_V(pipeline.impl() != nullptr, "Depth node must be part of a pipeline.");

    const Algorithm active = resolveAlgorithm(device);
    validateAlgorithm(device, active);

    switch(active) {
        case Algorithm::AUTO:
            DAI_CHECK_V(false, "Depth: AUTO must be resolved before wiring.");
            break;
        case Algorithm::TOF: {
            tofBackend_ = ToF::create(device);
            add(tofBackend_);
            tofBackend_->build(tofSocket_, tofPreset_, tofFps_);
            depthOut_ = &tofBackend_->depth;
            confidenceOut_ = &tofBackend_->amplitude;
            break;
        }
        case Algorithm::NEURAL_ASSISTED_STEREO: {
            const auto pair = requireFirstStereoPair(device);
            nasBackend_ = std::make_shared<NeuralAssistedStereo>(device);
            add(nasBackend_);
            auto [leftOut, rightOut] = ensureStereoFullResolutionOutputs(pipeline, pair, kStereoMonoFps);
            nasBackend_->build(*leftOut, *rightOut, nasNeuralModel_, nasRectify_);
            depthOut_ = &nasBackend_->depth;
            confidenceOut_ = &(*nasBackend_->stereoDepth).confidenceMap;
            break;
        }
        case Algorithm::GPU_STEREO: {
            const auto pair = requireFirstStereoPair(device);
            gpuStereoBackend_ = std::make_unique<Subnode<GPUStereo>>(*this, "gpuStereo");
            auto [leftOut, rightOut] = ensureStereoCameraOutputs(pipeline, pair, kStereoDepthMonoSize, kStereoMonoFps);
            (*gpuStereoBackend_)->setRectification(true).build(*leftOut, *rightOut);
            depthOut_ = &(**gpuStereoBackend_).depth;
            confidenceOut_ = &(**gpuStereoBackend_).disparity;
            break;
        }
        case Algorithm::NEURAL: {
            const auto pair = requireFirstStereoPair(device);
            neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth");
            const auto is = NeuralDepth::getInputSize(neuralModel_);
            const std::pair<uint32_t, uint32_t> monoSize{static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
            auto [leftOut, rightOut] = ensureStereoCameraOutputs(pipeline, pair, monoSize, kStereoMonoFps);
            (*neuralBackend_)->build(*leftOut, *rightOut, neuralModel_);
            depthOut_ = &(**neuralBackend_).depth;
            confidenceOut_ = &(**neuralBackend_).confidence;
            break;
        }
        case Algorithm::STEREO: {
            const auto pair = requireFirstStereoPair(device);
            stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoDepth");
            auto [leftOut, rightOut] = ensureStereoCameraOutputs(pipeline, pair, kStereoDepthMonoSize, kStereoMonoFps);
            (*stereoBackend_)->build(*leftOut, *rightOut, StereoDepth::PresetMode::DEFAULT);
            depthOut_ = &(**stereoBackend_).depth;
            confidenceOut_ = &(**stereoBackend_).confidenceMap;
            break;
        }
    }

    graphBuilt_ = true;
}

std::pair<Node::Output*, Node::Output*> Depth::ensureStereoCameraOutputs(Pipeline& pipeline,
                                                                        const StereoPair& pair,
                                                                        std::pair<uint32_t, uint32_t> frameSize,
                                                                        float monoFps) {
    auto [leftFound, rightFound] = findCamerasForPair(pipeline, pair);

    std::shared_ptr<Camera> left = leftFound;
    std::shared_ptr<Camera> right = rightFound;

    if(!left) {
        left = pipeline.create<Camera>()->build(pair.left);
    }
    if(!right) {
        right = pipeline.create<Camera>()->build(pair.right);
    }

    auto* lo = left->requestOutput(frameSize, std::nullopt, ImgResizeMode::CROP, monoFps);
    auto* ro = right->requestOutput(frameSize, std::nullopt, ImgResizeMode::CROP, monoFps);
    DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera stereo output request failed.");
    return {lo, ro};
}

std::pair<Node::Output*, Node::Output*> Depth::ensureStereoFullResolutionOutputs(Pipeline& pipeline,
                                                                                const StereoPair& pair,
                                                                                float monoFps) {
    auto [leftFound, rightFound] = findCamerasForPair(pipeline, pair);

    std::shared_ptr<Camera> left = leftFound;
    std::shared_ptr<Camera> right = rightFound;

    if(!left) {
        left = pipeline.create<Camera>()->build(pair.left);
    }
    if(!right) {
        right = pipeline.create<Camera>()->build(pair.right);
    }

    auto* lo = left->requestFullResolutionOutput(std::nullopt, monoFps, false);
    auto* ro = right->requestFullResolutionOutput(std::nullopt, monoFps, false);
    DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera full-resolution stereo output request failed.");
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
