/**
 * @file Depth.cpp
 * @brief Implementation of the composite Depth node: algorithm selection, lazy backend wiring,
 *        stereo camera provisioning, and unified depth/confidence outputs.
 */

#include "depthai/pipeline/node/Depth.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <optional>
#include <utility>
#include <variant>

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

// Anonymous helpers: device capability probes and stereo camera discovery.
namespace {

constexpr DeviceModelZoo kDefaultNeuralDepthModel = DeviceModelZoo::NEURAL_DEPTH_SMALL;
constexpr DeviceModelZoo kDefaultNasNeuralModel = DeviceModelZoo::NEURAL_DEPTH_NANO;
constexpr bool kDefaultNasRectify = true;

constexpr uint32_t kStereoDepthMaxWidth = 1280;
constexpr uint32_t kStereoDepthMaxHeight = 1280;
constexpr float kNasMaxFps = 60.f;
constexpr float kGpuStereoMaxFps = 30.f;
constexpr float kSelectionFpsSafetyMargin = 0.85f;

/** Per-model NeuralDepth limits (catalog order = largest / best quality first). */
struct NeuralModelEntry {
    DeviceModelZoo model;
    uint32_t tensorWidth;
    uint32_t tensorHeight;
    float maxFps;
};

constexpr std::array<NeuralModelEntry, 10> kNeuralModels = {{
    {DeviceModelZoo::NEURAL_DEPTH_1248X780, 1248, 780, 1.6f},
    {DeviceModelZoo::NEURAL_DEPTH_1056X660, 1056, 660, 3.0f},
    {DeviceModelZoo::NEURAL_DEPTH_960X600, 960, 600, 5.0f},
    {DeviceModelZoo::NEURAL_DEPTH_864X540, 864, 540, 8.0f},
    {DeviceModelZoo::NEURAL_DEPTH_768X480, 768, 480, 10.f},
    {DeviceModelZoo::NEURAL_DEPTH_576X360, 576, 360, 24.f},
    {DeviceModelZoo::NEURAL_DEPTH_480X300, 480, 300, 40.f},
    {DeviceModelZoo::NEURAL_DEPTH_384X240, 384, 240, 55.f},
    {DeviceModelZoo::NEURAL_DEPTH_288X180, 288, 180, 55.f},
    {DeviceModelZoo::NEURAL_DEPTH_192X120, 192, 120, 55.f},
}};

/** StereoDepth presets in quality order (same FPS tier: first listed wins). */
struct StereoPresetEntry {
    StereoDepth::PresetMode preset;
    float maxFps;
};

constexpr std::array<StereoPresetEntry, 8> kStereoPresets = {{
    {StereoDepth::PresetMode::ACCURACY, 15.f},
    {StereoDepth::PresetMode::HIGH_DETAIL, 15.f},
    {StereoDepth::PresetMode::DENSITY, 30.f},
    {StereoDepth::PresetMode::DEFAULT, 30.f},
    {StereoDepth::PresetMode::FACE, 30.f},
    {StereoDepth::PresetMode::ROBOTICS, 30.f},
    {StereoDepth::PresetMode::FAST_DENSITY, 60.f},
    {StereoDepth::PresetMode::FAST_ACCURACY, 60.f},
}};

/**
 * True when a user frame can be resized into this model's tensor without aggressive scaling.
 * Per axis the acceptable range is roughly tensor / sqrt(2) … tensor * sqrt(2).
 */
bool neuralModelCoversUserResolution(const NeuralModelEntry& entry, uint32_t userWidth, uint32_t userHeight) {
    constexpr float kScale = 1.4142135f;
    const uint32_t minW = static_cast<uint32_t>(static_cast<float>(entry.tensorWidth) / kScale);
    const uint32_t maxW = static_cast<uint32_t>(static_cast<float>(entry.tensorWidth) * kScale);
    const uint32_t minH = static_cast<uint32_t>(static_cast<float>(entry.tensorHeight) / kScale);
    const uint32_t maxH = static_cast<uint32_t>(static_cast<float>(entry.tensorHeight) * kScale);
    return userWidth >= minW && userWidth <= maxW && userHeight >= minH && userHeight <= maxH;
}

bool isModelOnDevice(DeviceModelZoo model, const std::vector<DeviceModelZoo>& supportedModels) {
    if(supportedModels.empty()) {
        return true;
    }
    return std::find(supportedModels.begin(), supportedModels.end(), model) != supportedModels.end();
}

bool userResolutionWithinStereoCap(uint32_t userWidth, uint32_t userHeight) {
    return userWidth <= kStereoDepthMaxWidth && userHeight <= kStereoDepthMaxHeight;
}

bool supportsAlgorithm(const std::vector<Depth::Algorithm>& supported, Depth::Algorithm algorithm) {
    return std::find(supported.begin(), supported.end(), algorithm) != supported.end();
}

/** Best NeuralDepth model meeting @p requiredFps; when @p resolution is set it must also be covered. */
std::optional<DeviceModelZoo> pickNeuralModel(float requiredFps,
                                               std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                               const std::vector<DeviceModelZoo>& supportedModels) {
    for(const auto& entry : kNeuralModels) {
        if(entry.maxFps < requiredFps) {
            continue;
        }
        if(!isModelOnDevice(entry.model, supportedModels)) {
            continue;
        }
        if(resolution.has_value() && !neuralModelCoversUserResolution(entry, resolution->first, resolution->second)) {
            continue;
        }
        return entry.model;
    }
    return std::nullopt;
}

/** Best StereoDepth preset meeting @p requiredFps; when @p resolution is set it must fit 1280x1280. */
std::optional<StereoDepth::PresetMode> pickStereoPreset(float requiredFps,
                                                         std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    for(const auto& entry : kStereoPresets) {
        if(entry.maxFps < requiredFps) {
            continue;
        }
        if(resolution.has_value() && !userResolutionWithinStereoCap(resolution->first, resolution->second)) {
            continue;
        }
        return entry.preset;
    }
    return std::nullopt;
}

/** Locate existing Camera nodes for @p pair.left / @p pair.right sockets, if already in @p pipeline. */
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

/**
 * When both stereo cameras exist before Depth wiring, use their configured size.
 * ``Camera::getMaxWidth`` / ``getMaxHeight`` return ``build(..., sensorResolution)`` when set,
 * otherwise the sensor maximum for that socket.
 */
std::optional<std::pair<uint32_t, uint32_t>> stereoSizeFromExistingCameras(const std::shared_ptr<Camera>& left,
                                                                           const std::shared_ptr<Camera>& right) {
    if(!left || !right) {
        return std::nullopt;
    }
    const uint32_t width = left->getMaxWidth();
    const uint32_t height = left->getMaxHeight();
    if(width > 0 && height > 0 && width == right->getMaxWidth() && height == right->getMaxHeight()) {
        return {{width, height}};
    }
    return std::nullopt;
}

/** Returns the device's primary stereo pair; fails if none are configured. */
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

// --- Construction ---

Depth::Depth(Algorithm algorithm) : DeviceNodeGroup(), algorithmOverride_(algorithm) {}

Depth::Depth() : Depth(Algorithm::AUTO) {}

// --- Pre-wiring configuration ---

std::shared_ptr<Depth> Depth::build(std::optional<float> fps) {
    DAI_CHECK_V(!graphBuilt_, "Depth::build(fps) must be called before the graph is wired (before first depth()/confidence() access).");
    stereoOutputFps_ = fps;  // Applied to Camera outputs when a stereo-based backend is wired.
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm, std::optional<float> fps) {
    return build(algorithm, fps, std::nullopt);
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm,
                                      std::optional<float> fps,
                                      std::optional<std::pair<uint32_t, uint32_t>> stereoSize) {
    DAI_CHECK_V(!graphBuilt_,
                "Depth::build(algorithm, fps, stereoSize) must be called before the graph is wired (before first "
                "depth()/confidence() access).");
    algorithmOverride_ = algorithm;
    stereoOutputFps_ = fps;
    stereoSizeOverride_ = stereoSize;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(DeviceModelZoo neuralModel) {
    DAI_CHECK_V(!graphBuilt_, "Depth::build(neuralModel) must be called before the graph is wired (before first depth()/confidence() access).");
    neuralModelOverride_ = neuralModel;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

// --- Device queries ---

std::vector<Depth::Algorithm> Depth::getSupportedAlgorithms(const std::shared_ptr<Device>& device) const {
    std::vector<Algorithm> supported = {Algorithm::STEREO};

    if(device->isNeuralDepthSupported()) {
        supported.push_back(Algorithm::NEURAL);
        if(device->getPlatform() == Platform::RVC4) {
            supported.push_back(Algorithm::NEURAL_ASSISTED_STEREO);
        }
    }
    if(device->isGpuStereoSupported() && !device->getStereoPairs().empty()) {
        supported.push_back(Algorithm::GPU_STEREO);
    }
    if(cameraFeaturesIncludeTof(device->getConnectedCameraFeatures())) {
        supported.push_back(Algorithm::TOF);
    }

    return supported;
}

// --- Algorithm resolution ---

bool Depth::exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height) {
    return width > kStereoDepthMaxWidth || height > kStereoDepthMaxHeight;
}

DeviceModelZoo Depth::getResolvedNeuralModel() const {
    if(const auto* m = std::get_if<DeviceModelZoo>(&resolved_.config)) {
        return *m;
    }
    if(resolved_.algorithm == Algorithm::NEURAL_ASSISTED_STEREO) {
        return kDefaultNasNeuralModel;
    }
    return kDefaultNeuralDepthModel;
}

StereoDepth::PresetMode Depth::getResolvedStereoPreset() const {
    if(const auto* p = std::get_if<StereoDepth::PresetMode>(&resolved_.config)) {
        return *p;
    }
    return StereoDepth::PresetMode::DEFAULT;
}

Depth::Selection Depth::selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                       float targetFps,
                                       const std::vector<Algorithm>& supportedAlgorithms,
                                       const std::vector<DeviceModelZoo>& supportedModels) {
    if(targetFps <= 0.f) {
        targetFps = 30.f;
    }
    const float requiredFps = targetFps * kSelectionFpsSafetyMargin;

    // 1. NeuralDepth — keep FPS; when resolution is set, pick the best model that also covers it.
    if(supportsAlgorithm(supportedAlgorithms, Algorithm::NEURAL)) {
        if(const auto model = pickNeuralModel(requiredFps, resolution, supportedModels)) {
            return {Algorithm::NEURAL, *model};
        }
    }

    // 2. NeuralAssistedStereo — same FPS and resolution rules (1280x1280 cap).
    if(supportsAlgorithm(supportedAlgorithms, Algorithm::NEURAL_ASSISTED_STEREO) && kNasMaxFps >= requiredFps) {
        const bool resolutionOk = !resolution.has_value()
                                  || userResolutionWithinStereoCap(resolution->first, resolution->second);
        if(resolutionOk) {
            return {Algorithm::NEURAL_ASSISTED_STEREO, std::monostate{}};
        }
    }

    // 3. StereoDepth — quality-ordered presets that keep FPS (and resolution when set).
    if(supportsAlgorithm(supportedAlgorithms, Algorithm::STEREO)) {
        if(const auto preset = pickStereoPreset(requiredFps, resolution)) {
            return {Algorithm::STEREO, *preset};
        }
    }

    // 4. GPUStereo — unlimited resolution; only FPS is checked.
    if(supportsAlgorithm(supportedAlgorithms, Algorithm::GPU_STEREO) && kGpuStereoMaxFps >= requiredFps) {
        return {Algorithm::GPU_STEREO, std::monostate{}};
    }

    // Requested FPS exceeds every backend: still honor resolution, return the fastest stereo preset.
    if(supportsAlgorithm(supportedAlgorithms, Algorithm::STEREO)) {
        if(const auto preset = pickStereoPreset(0.f, resolution)) {
            return {Algorithm::STEREO, *preset};
        }
    }
    return {Algorithm::STEREO, StereoDepth::PresetMode::FAST_ACCURACY};
}

void Depth::resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline) {
    const auto supported = getSupportedAlgorithms(device);

    if(algorithmOverride_ != Algorithm::AUTO) {
        DAI_CHECK_V(supportsAlgorithm(supported, algorithmOverride_), "Depth algorithm is not supported on this device.");
    }

    if(device->getPlatform() != Platform::RVC4) {
        if(algorithmOverride_ == Algorithm::AUTO) {
            if(supportsAlgorithm(supported, Algorithm::TOF)) {
                resolved_ = {Algorithm::TOF, std::monostate{}};
            } else {
                resolved_ = {Algorithm::STEREO, std::monostate{}};
            }
        } else {
            resolved_ = {algorithmOverride_, std::monostate{}};
        }
        return;
    }

    // RVC4: target FPS from build() override, upstream cameras, or default 30.
    const auto pair = requireFirstStereoPair(device);
    const auto [left, right] = findCamerasForPair(pipeline, pair);

    float targetFps = stereoOutputFps_.value_or(0.f);
    if(targetFps <= 0.f && left && right) {
        targetFps = std::max(left->getMaxRequestedFps(), right->getMaxRequestedFps());
    }
    if(targetFps <= 0.f) {
        targetFps = 30.f;
    }

    // User resolution from build() override or upstream stereo cameras (unset = no resolution gate).
    std::optional<std::pair<uint32_t, uint32_t>> resolution = stereoSizeOverride_;
    if(!resolution) {
        resolution = stereoSizeFromExistingCameras(left, right);
    }

    const auto supportedModels = device->getSupportedDeviceModels();

    if(algorithmOverride_ == Algorithm::AUTO) {
        resolved_ = selectBackend(resolution, targetFps, supported, supportedModels);
        if(neuralModelOverride_ && resolved_.algorithm == Algorithm::NEURAL) {
            resolved_.config = *neuralModelOverride_;
        }
        return;
    }

    // Explicit algorithm: pick only the config (model or preset) for that backend.
    resolved_.algorithm = algorithmOverride_;
    resolved_.config = std::monostate{};

    if(resolution
       && (resolved_.algorithm == Algorithm::STEREO || resolved_.algorithm == Algorithm::NEURAL_ASSISTED_STEREO)) {
        DAI_CHECK_V(!exceedsStereoDepthMaxResolution(resolution->first, resolution->second),
                    "Depth: resolution exceeds StereoDepth maximum 1280x1280; use GPUStereo or lower resolution.");
    }

    switch(resolved_.algorithm) {
        case Algorithm::NEURAL:
            if(neuralModelOverride_) {
                resolved_.config = *neuralModelOverride_;
            } else if(const auto model = pickNeuralModel(targetFps * kSelectionFpsSafetyMargin, resolution, supportedModels)) {
                resolved_.config = *model;
            } else {
                resolved_.config = kDefaultNeuralDepthModel;
            }
            break;
        case Algorithm::STEREO:
            if(const auto preset = pickStereoPreset(targetFps * kSelectionFpsSafetyMargin, resolution)) {
                resolved_.config = *preset;
            } else {
                resolved_.config = StereoDepth::PresetMode::DEFAULT;
            }
            break;
        case Algorithm::NEURAL_ASSISTED_STEREO:
        case Algorithm::GPU_STEREO:
        case Algorithm::TOF:
        case Algorithm::AUTO:
            break;
    }
}

// --- Lazy graph wiring ---

void Depth::buildInternal() {
    if(graphBuilt_) {
        return;
    }
    // Defer wiring until the node is attached to a pipeline (parent is set).
    if(parent.lock() == nullptr) {
        return;
    }

    Pipeline pipeline = getParentPipeline();
    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "Depth node requires a device (set on create, when added to a pipeline with a default device, or from pipeline at first wiring).");

    resolveWiring(device, pipeline);
    const Algorithm active = resolved_.algorithm;

    switch(active) {
        case Algorithm::AUTO:
            DAI_CHECK_V(false, "Depth: AUTO must be resolved before wiring.");
            break;
        case Algorithm::TOF:
            tofBackend_ = ToF::create(device);
            add(tofBackend_);
            tofBackend_->build(CameraBoardSocket::AUTO, ImageFiltersPresetMode::TOF_MID_RANGE, stereoOutputFps_);
            break;
        case Algorithm::NEURAL_ASSISTED_STEREO: {
            nasBackend_ = std::make_shared<NeuralAssistedStereo>(device);
            add(nasBackend_);
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, std::nullopt);
            nasBackend_->build(*leftOut, *rightOut, kDefaultNasNeuralModel, kDefaultNasRectify);
            break;
        }
        case Algorithm::GPU_STEREO: {
            gpuStereoBackend_ = std::make_unique<Subnode<GPUStereo>>(*this, "gpuStereo");
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, std::nullopt);
            (*gpuStereoBackend_)->setRectification(true).build(*leftOut, *rightOut);
            break;
        }
        case Algorithm::NEURAL: {
            const auto model = getResolvedNeuralModel();
            const auto is = NeuralDepth::getInputSize(model);
            const std::pair<uint32_t, uint32_t> monoSize{static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
            neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth");
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, monoSize);
            (*neuralBackend_)->build(*leftOut, *rightOut, model);
            break;
        }
        case Algorithm::STEREO: {
            stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoDepth");
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, std::nullopt);
            (*stereoBackend_)->build(*leftOut, *rightOut, getResolvedStereoPreset());
            break;
        }
        default:
            DAI_CHECK_V(false, "Depth: no backend was selected for wiring.");
    }

    bindBackendOutputs(active);
    graphBuilt_ = true;
}

// --- Stereo camera wiring ---

std::pair<Node::Output*, Node::Output*> Depth::stereoCameraOutputs(Pipeline& pipeline,
                                                                   const std::shared_ptr<Device>& device,
                                                                   std::optional<std::pair<uint32_t, uint32_t>> frameSize) {
    return ensureStereoOutputs(pipeline, requireFirstStereoPair(device), frameSize, stereoOutputFps_);
}

/** Wire composite outputs to the active backend. Every backend provides a confidence-like stream. */
void Depth::bindBackendOutputs(Algorithm active) {
    switch(active) {
        case Algorithm::TOF:
            depthOut_ = &tofBackend_->depth;
            confidenceOut_ = &tofBackend_->amplitude;  // ToF "confidence" is amplitude.
            break;
        case Algorithm::NEURAL_ASSISTED_STEREO:
            depthOut_ = &nasBackend_->depth;
            confidenceOut_ = &(*nasBackend_->stereoDepth).confidenceMap;
            break;
        case Algorithm::GPU_STEREO:
            depthOut_ = &(**gpuStereoBackend_).depth;
            confidenceOut_ = &(**gpuStereoBackend_).confidenceMap;
            break;
        case Algorithm::NEURAL:
            depthOut_ = &(**neuralBackend_).depth;
            confidenceOut_ = &(**neuralBackend_).confidence;
            break;
        case Algorithm::STEREO:
            depthOut_ = &(**stereoBackend_).depth;
            confidenceOut_ = &(**stereoBackend_).confidenceMap;
            break;
        case Algorithm::AUTO:
        default:
            DAI_CHECK_V(false, "Depth: no backend outputs to bind.");
    }
}

std::pair<Node::Output*, Node::Output*> Depth::ensureStereoOutputs(Pipeline& pipeline,
                                                                   const StereoPair& pair,
                                                                   std::optional<std::pair<uint32_t, uint32_t>> frameSize,
                                                                   const std::optional<float>& fps) {
    auto [left, right] = findCamerasForPair(pipeline, pair);
    const bool stereoCamerasPreexist = left && right;

    // Create missing cameras on the sockets from the device's stereo pair.
    if(!left) {
        left = pipeline.create<Camera>()->build(pair.left);
    }
    if(!right) {
        right = pipeline.create<Camera>()->build(pair.right);
    }

    // When @p frameSize is unset and both stereo cameras already exist, match their sensor resolution.
    std::optional<std::pair<uint32_t, uint32_t>> outputSize = frameSize;
    if(stereoCamerasPreexist && !frameSize.has_value()) {
        if(const auto existingSize = stereoSizeFromExistingCameras(left, right)) {
            outputSize = existingSize;
        }
    }
    const std::optional<float>& outputFps = fps;

    Node::Output* lo = nullptr;
    Node::Output* ro = nullptr;
    if(outputSize) {
        // Sized output (e.g. NeuralDepth model dimensions) or matched pre-existing camera size.
        lo = left->requestOutput(*outputSize, std::nullopt, ImgResizeMode::CROP, outputFps);
        ro = right->requestOutput(*outputSize, std::nullopt, ImgResizeMode::CROP, outputFps);
        DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera stereo output request failed.");
    } else {
        // Depth-created cameras: use full sensor resolution.
        lo = left->requestFullResolutionOutput(std::nullopt, outputFps, false);
        ro = right->requestFullResolutionOutput(std::nullopt, outputFps, false);
        DAI_CHECK_V(lo != nullptr && ro != nullptr, "Camera full-resolution stereo output request failed.");
    }
    return {lo, ro};
}

// --- Public outputs (trigger lazy wiring) ---

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
