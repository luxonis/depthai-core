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

/** Backwards-compat with the public Depth::exceedsStereoDepthMaxResolution(w,h) check (StereoDepth alone). */
constexpr uint32_t kStereoDepthMaxWidth = 1280;
constexpr uint32_t kStereoDepthMaxHeight = 1280;
constexpr float kSelectionFpsSafetyMargin = 0.85f;

/**
 * Unified backend-profile catalog.
 *
 * One row per (algorithm, sub-profile) describes everything ``Depth`` needs to score a candidate:
 * the user-visible config (a ``DeviceModelZoo`` for ``NEURAL``, a ``StereoDepth::PresetMode``
 * for ``STEREO``, or ``std::monostate`` for backends with no selectable profile), the maximum
 * input resolution, and the maximum sustained FPS.
 *
 * For ``NEURAL`` rows ``maxWidth/maxHeight`` is the model's native tensor size: the resolution
 * check uses a tensor-cover band (resize without aggressive up/down-scaling) rather than a plain
 * cap. Every other algorithm uses a simple ``user <= max`` cap.
 *
 * Rows are listed in selection priority order (NEURAL → NEURAL_ASSISTED_STEREO → STEREO →
 * GPU_STEREO); within a single algorithm rows are ordered best-quality first.
 */
struct BackendProfile {
    Depth::Algorithm algorithm;
    Depth::Config config;
    uint32_t maxWidth;
    uint32_t maxHeight;
    float maxFps;
};

inline const std::array<BackendProfile, 17> kBackendProfiles = {{
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_1248X780, 1248, 780, 1.6f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_1056X660, 1056, 660, 3.0f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_960X600, 960, 600, 5.0f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_864X540, 864, 540, 8.0f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_768X480, 768, 480, 10.f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_576X360, 576, 360, 24.f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_480X300, 480, 300, 40.f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_384X240, 384, 240, 55.f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_288X180, 288, 180, 55.f},
    {Depth::Algorithm::NEURAL, DeviceModelZoo::NEURAL_DEPTH_192X120, 192, 120, 55.f},

    {Depth::Algorithm::NEURAL_ASSISTED_STEREO, std::monostate{}, 1280, 800, 55.f},

    {Depth::Algorithm::STEREO, StereoDepth::PresetMode::DEFAULT, 1280, 1280, 30.f},
    {Depth::Algorithm::STEREO, StereoDepth::PresetMode::FAST_DENSITY, 1280, 1280, 60.f},
    {Depth::Algorithm::STEREO, StereoDepth::PresetMode::FAST_ACCURACY, 1280, 1280, 60.f},

    {Depth::Algorithm::GPU_STEREO, std::monostate{}, 2592, 1944, 5.f},
    {Depth::Algorithm::GPU_STEREO, std::monostate{}, 1280, 800, 30.f},
    {Depth::Algorithm::GPU_STEREO, std::monostate{}, 640, 400, 55.f},
}};

bool isModelOnDevice(DeviceModelZoo model, const std::vector<DeviceModelZoo>& supportedModels) {
    if(supportedModels.empty()) {
        return true;
    }
    return std::find(supportedModels.begin(), supportedModels.end(), model) != supportedModels.end();
}

bool supportsAlgorithm(const std::vector<Depth::Algorithm>& supported, Depth::Algorithm algorithm) {
    return std::find(supported.begin(), supported.end(), algorithm) != supported.end();
}

DeviceModelZoo neuralModelFromConfig(const Depth::Config& config) {
    if(const auto* m = std::get_if<DeviceModelZoo>(&config)) {
        return *m;
    }
    return kDefaultNeuralDepthModel;
}

StereoDepth::PresetMode stereoPresetFromConfig(const Depth::Config& config) {
    if(const auto* p = std::get_if<StereoDepth::PresetMode>(&config)) {
        return *p;
    }
    return StereoDepth::PresetMode::DEFAULT;
}

/**
 * Per-algorithm resolution semantics: NEURAL uses a tensor-cover band (±sqrt(2)) around the
 * model's native size; everything else is a simple ``user <= max`` cap.
 */
bool resolutionFits(const BackendProfile& profile, std::pair<uint32_t, uint32_t> resolution) {
    if(profile.algorithm == Depth::Algorithm::NEURAL) {
        constexpr float kScale = 1.4142135f;
        const uint32_t minW = static_cast<uint32_t>(static_cast<float>(profile.maxWidth) / kScale);
        const uint32_t maxW = static_cast<uint32_t>(static_cast<float>(profile.maxWidth) * kScale);
        const uint32_t minH = static_cast<uint32_t>(static_cast<float>(profile.maxHeight) / kScale);
        const uint32_t maxH = static_cast<uint32_t>(static_cast<float>(profile.maxHeight) * kScale);
        return resolution.first >= minW && resolution.first <= maxW && resolution.second >= minH && resolution.second <= maxH;
    }
    return resolution.first <= profile.maxWidth && resolution.second <= profile.maxHeight;
}

/** True when @p profile would accept this (algorithm, model, fps, resolution) request. */
bool profileMatches(const BackendProfile& profile,
                    const std::vector<Depth::Algorithm>& supportedAlgorithms,
                    const std::vector<DeviceModelZoo>& supportedModels,
                    float requiredFps,
                    std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    if(!supportsAlgorithm(supportedAlgorithms, profile.algorithm)) {
        return false;
    }
    if(profile.maxFps < requiredFps) {
        return false;
    }
    if(const auto* model = std::get_if<DeviceModelZoo>(&profile.config)) {
        if(!isModelOnDevice(*model, supportedModels)) {
            return false;
        }
    }
    if(resolution.has_value() && !resolutionFits(profile, *resolution)) {
        return false;
    }
    return true;
}

/** Walks the catalog in priority order and returns the first row that satisfies the request. */
std::optional<BackendProfile> pickBackendProfile(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                                  float requiredFps,
                                                  const std::vector<Depth::Algorithm>& supportedAlgorithms,
                                                  const std::vector<DeviceModelZoo>& supportedModels) {
    for(const auto& profile : kBackendProfiles) {
        if(profileMatches(profile, supportedAlgorithms, supportedModels, requiredFps, resolution)) {
            return profile;
        }
    }
    return std::nullopt;
}

/**
 * Among catalog rows that accept @p resolution (and pass algorithm/model gates), pick the one with
 * the highest ``maxFps``. Used when the priority scan finds no row meeting the requested FPS, so
 * the algorithm able to serve the resolution at the highest available FPS is preferred over the
 * generic StereoDepth last-resort.
 */
std::optional<BackendProfile> pickHighestFpsForResolution(std::pair<uint32_t, uint32_t> resolution,
                                                          const std::vector<Depth::Algorithm>& supportedAlgorithms,
                                                          const std::vector<DeviceModelZoo>& supportedModels) {
    std::optional<BackendProfile> best;
    for(const auto& profile : kBackendProfiles) {
        if(!supportsAlgorithm(supportedAlgorithms, profile.algorithm)) continue;
        if(const auto* model = std::get_if<DeviceModelZoo>(&profile.config); model && !isModelOnDevice(*model, supportedModels)) continue;
        if(!resolutionFits(profile, resolution)) continue;
        if(!best || profile.maxFps > best->maxFps) {
            best = profile;
        }
    }
    return best;
}

/** NeuralDepth model lookup for the explicit-algorithm path (skips non-NEURAL rows). */
std::optional<DeviceModelZoo> pickNeuralModel(float requiredFps,
                                               std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                               const std::vector<DeviceModelZoo>& supportedModels) {
    for(const auto& profile : kBackendProfiles) {
        if(profile.algorithm != Depth::Algorithm::NEURAL) continue;
        if(profile.maxFps < requiredFps) continue;
        const auto* model = std::get_if<DeviceModelZoo>(&profile.config);
        if(model == nullptr || !isModelOnDevice(*model, supportedModels)) continue;
        if(resolution.has_value() && !resolutionFits(profile, *resolution)) continue;
        return *model;
    }
    return std::nullopt;
}

/** StereoDepth preset lookup for the explicit-algorithm path (skips non-STEREO rows). */
std::optional<StereoDepth::PresetMode> pickStereoPreset(float requiredFps,
                                                         std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    for(const auto& profile : kBackendProfiles) {
        if(profile.algorithm != Depth::Algorithm::STEREO) continue;
        if(profile.maxFps < requiredFps) continue;
        if(resolution.has_value() && !resolutionFits(profile, *resolution)) continue;
        const auto* preset = std::get_if<StereoDepth::PresetMode>(&profile.config);
        if(preset == nullptr) continue;
        return *preset;
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
 * When both stereo cameras exist before Depth wiring, use their requested output size if the user
 * already requested camera outputs. Otherwise fall back to the cameras' configured sensor size.
 */
std::optional<std::pair<uint32_t, uint32_t>> stereoSizeFromExistingCameras(const std::shared_ptr<Camera>& left,
                                                                           const std::shared_ptr<Camera>& right) {
    if(!left || !right) {
        return std::nullopt;
    }
    const uint32_t width = left->getMaxRequestedWidth();
    const uint32_t height = left->getMaxRequestedHeight();
    if(width > 0 && height > 0 && width == right->getMaxRequestedWidth() && height == right->getMaxRequestedHeight()) {
        return {{width, height}};
    }
    return std::nullopt;
}

/**
 * Sensor native max for the stereo pair sockets, queried directly from the device when no
 * upstream ``Camera`` nodes are present and the user has not provided a ``stereoSize`` override.
 * Returns the smaller of ``left`` / ``right`` per axis when both sockets advertise features;
 * ``std::nullopt`` if neither socket is advertised.
 */
std::optional<std::pair<uint32_t, uint32_t>> stereoSizeFromDeviceFeatures(const std::shared_ptr<Device>& device, const StereoPair& pair) {
    const auto features = device->getConnectedCameraFeatures();
    auto findSocket = [&](CameraBoardSocket socket) -> std::optional<std::pair<uint32_t, uint32_t>> {
        for(const auto& f : features) {
            if(f.socket == socket && f.width > 0 && f.height > 0) {
                return std::make_pair(static_cast<uint32_t>(f.width), static_cast<uint32_t>(f.height));
            }
        }
        return std::nullopt;
    };
    const auto leftSize = findSocket(pair.left);
    const auto rightSize = findSocket(pair.right);
    if(!leftSize && !rightSize) return std::nullopt;
    if(leftSize && !rightSize) return leftSize;
    if(rightSize && !leftSize) return rightSize;
    return std::make_pair(std::min(leftSize->first, rightSize->first), std::min(leftSize->second, rightSize->second));
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

Depth::Selection Depth::selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                       float targetFps,
                                       const std::vector<Algorithm>& supportedAlgorithms,
                                       const std::vector<DeviceModelZoo>& supportedModels) {
    if(targetFps <= 0.f) {
        targetFps = 30.f;
    }
    const float requiredFps = targetFps * kSelectionFpsSafetyMargin;

    // 1. Priority scan honoring the requested FPS and (optional) resolution.
    if(const auto picked = pickBackendProfile(resolution, requiredFps, supportedAlgorithms, supportedModels)) {
        return {picked->algorithm, picked->config};
    }

    // 2. No backend can serve the resolution at the requested FPS: pick the algorithm whose
    //    catalog row covers the resolution with the highest available maxFps.
    if(resolution.has_value()) {
        if(const auto picked = pickHighestFpsForResolution(*resolution, supportedAlgorithms, supportedModels)) {
            return {picked->algorithm, picked->config};
        }
    }

    // 3. Last-resort fallback: drop the FPS gate, honor resolution if possible, else fastest StereoDepth preset.
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
        // Non-RVC4 has no catalog scan; pick the algorithm and populate the preset that
        // actually goes into the backend's build() (so getResolvedPreset() matches reality).
        const Algorithm chosen = (algorithmOverride_ == Algorithm::AUTO)
                                     ? (supportsAlgorithm(supported, Algorithm::TOF) ? Algorithm::TOF : Algorithm::STEREO)
                                     : algorithmOverride_;
        Config config = std::monostate{};
        if(chosen == Algorithm::STEREO) {
            config = StereoDepth::PresetMode::DEFAULT;
        }
        resolved_ = {chosen, config};
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

    // Resolution comes from (in order): build() override, upstream stereo cameras' requested output
    // size (falling back to configured/native sensor size), or device features. Falling back to
    // device features lets AUTO see the camera resolution even when the user wires only the Depth
    // node, so e.g. a 2592x1944 sensor steers selection toward GPUStereo via the resolution-fit
    // fallback in ``selectBackend``.
    std::optional<std::pair<uint32_t, uint32_t>> resolution = stereoSizeOverride_;
    if(!resolution) {
        resolution = stereoSizeFromExistingCameras(left, right);
    }
    if(!resolution) {
        resolution = stereoSizeFromDeviceFeatures(device, pair);
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
       && (resolved_.algorithm == Algorithm::STEREO || resolved_.algorithm == Algorithm::NEURAL_ASSISTED_STEREO
           || resolved_.algorithm == Algorithm::GPU_STEREO)) {
        bool anyProfileFits = false;
        uint32_t maxW = 0;
        uint32_t maxH = 0;
        for(const auto& profile : kBackendProfiles) {
            if(profile.algorithm != resolved_.algorithm) continue;
            maxW = std::max(maxW, profile.maxWidth);
            maxH = std::max(maxH, profile.maxHeight);
            if(resolutionFits(profile, *resolution)) {
                anyProfileFits = true;
                break;
            }
        }
        DAI_CHECK_V(anyProfileFits,
                    "Depth: resolution exceeds the maximum supported by the requested algorithm "
                    "(largest profile: {}x{}).",
                    maxW,
                    maxH);
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
            const auto model = neuralModelFromConfig(resolved_.config);
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
            (*stereoBackend_)->build(*leftOut, *rightOut, stereoPresetFromConfig(resolved_.config));
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
