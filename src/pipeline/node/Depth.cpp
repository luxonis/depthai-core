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

constexpr DeviceModelZoo DEFAULT_NEURAL_DEPTH_MODEL = DeviceModelZoo::NEURAL_DEPTH_SMALL;
constexpr DeviceModelZoo DEFAULT_NAS_NEURAL_MODEL = DeviceModelZoo::NEURAL_DEPTH_NANO;
constexpr bool DEFAULT_NAS_RECTIFY = true;
constexpr float DEFAULT_TARGET_FPS = 30.f;
constexpr float SELECTION_FPS_SAFETY_MARGIN = 0.85f;
constexpr float NEURAL_TENSOR_COVER_SCALE = 1.4142135f;

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

inline const std::array<BackendProfile, 17> BACKEND_PROFILES = {{
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

const char* algorithmName(Depth::Algorithm algorithm) {
    switch(algorithm) {
        case Depth::Algorithm::AUTO:
            return "AUTO";
        case Depth::Algorithm::STEREO:
            return "STEREO";
        case Depth::Algorithm::NEURAL:
            return "NEURAL";
        case Depth::Algorithm::NEURAL_ASSISTED_STEREO:
            return "NEURAL_ASSISTED_STEREO";
        case Depth::Algorithm::TOF:
            return "TOF";
        case Depth::Algorithm::GPU_STEREO:
            return "GPU_STEREO";
    }
    return "UNKNOWN";
}

/** Validate an explicit (algorithm, config) pair against the algorithm's expected Config type and device support. */
void validateExplicitConfig(Depth::Algorithm algorithm,
                            const Depth::Config& config,
                            const std::vector<Depth::Algorithm>& supportedAlgorithms,
                            const std::vector<DeviceModelZoo>& supportedModels) {
    DAI_CHECK_V(algorithm != Depth::Algorithm::AUTO, "Depth: a config can only be pinned together with a concrete algorithm (not AUTO).");
    DAI_CHECK_V(supportsAlgorithm(supportedAlgorithms, algorithm),
                "Depth selected algorithm {} is not supported on this device.",
                algorithmName(algorithm));

    switch(algorithm) {
        case Depth::Algorithm::NEURAL: {
            const auto* model = std::get_if<DeviceModelZoo>(&config);
            DAI_CHECK_V(model != nullptr, "Depth config for NEURAL must be a DeviceModelZoo model.");
            DAI_CHECK_V(isModelOnDevice(*model, supportedModels), "Depth selected NeuralDepth model is not available on this device.");
            break;
        }
        case Depth::Algorithm::STEREO:
            DAI_CHECK_V(std::holds_alternative<StereoDepth::PresetMode>(config), "Depth config for STEREO must be a StereoDepth::PresetMode.");
            break;
        case Depth::Algorithm::NEURAL_ASSISTED_STEREO:
        case Depth::Algorithm::GPU_STEREO:
        case Depth::Algorithm::TOF:
            DAI_CHECK_V(std::holds_alternative<std::monostate>(config), "Depth config for {} must be empty (std::monostate).", algorithmName(algorithm));
            break;
        case Depth::Algorithm::AUTO:
            break;
    }
}

DeviceModelZoo neuralModelFromConfig(const Depth::Config& config) {
    if(const auto* m = std::get_if<DeviceModelZoo>(&config)) {
        return *m;
    }
    return DEFAULT_NEURAL_DEPTH_MODEL;
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
        const uint32_t minW = static_cast<uint32_t>(static_cast<float>(profile.maxWidth) / NEURAL_TENSOR_COVER_SCALE);
        const uint32_t maxW = static_cast<uint32_t>(static_cast<float>(profile.maxWidth) * NEURAL_TENSOR_COVER_SCALE);
        const uint32_t minH = static_cast<uint32_t>(static_cast<float>(profile.maxHeight) / NEURAL_TENSOR_COVER_SCALE);
        const uint32_t maxH = static_cast<uint32_t>(static_cast<float>(profile.maxHeight) * NEURAL_TENSOR_COVER_SCALE);
        return resolution.first >= minW && resolution.first <= maxW && resolution.second >= minH && resolution.second <= maxH;
    }
    return resolution.first <= profile.maxWidth && resolution.second <= profile.maxHeight;
}

bool profileModelSupported(const BackendProfile& profile, const std::vector<DeviceModelZoo>& supportedModels) {
    if(const auto* model = std::get_if<DeviceModelZoo>(&profile.config)) {
        return isModelOnDevice(*model, supportedModels);
    }
    return true;
}

template <typename Predicate>
std::optional<BackendProfile> findBackendProfile(Predicate&& predicate) {
    for(const auto& profile : BACKEND_PROFILES) {
        if(predicate(profile)) {
            return profile;
        }
    }
    return std::nullopt;
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
    if(!profileModelSupported(profile, supportedModels)) {
        return false;
    }
    if(resolution.has_value() && !resolutionFits(profile, *resolution)) {
        return false;
    }
    return true;
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
    for(const auto& profile : BACKEND_PROFILES) {
        if(!supportsAlgorithm(supportedAlgorithms, profile.algorithm)) continue;
        if(!profileModelSupported(profile, supportedModels)) continue;
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
    const auto picked = findBackendProfile([&](const BackendProfile& profile) {
        return profile.algorithm == Depth::Algorithm::NEURAL && profile.maxFps >= requiredFps && profileModelSupported(profile, supportedModels)
               && (!resolution.has_value() || resolutionFits(profile, *resolution));
    });
    if(picked) {
        return std::get<DeviceModelZoo>(picked->config);
    }
    return std::nullopt;
}

/** StereoDepth preset lookup for the explicit-algorithm path (skips non-STEREO rows). */
std::optional<StereoDepth::PresetMode> pickStereoPreset(float requiredFps,
                                                         std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    const auto picked = findBackendProfile([&](const BackendProfile& profile) {
        return profile.algorithm == Depth::Algorithm::STEREO && profile.maxFps >= requiredFps
               && (!resolution.has_value() || resolutionFits(profile, *resolution));
    });
    if(picked) {
        return std::get<StereoDepth::PresetMode>(picked->config);
    }
    return std::nullopt;
}

bool anyProfileFits(Depth::Algorithm algorithm, std::pair<uint32_t, uint32_t> resolution) {
    return findBackendProfile([&](const BackendProfile& profile) { return profile.algorithm == algorithm && resolutionFits(profile, resolution); })
        .has_value();
}

std::pair<uint32_t, uint32_t> maxResolutionForAlgorithm(Depth::Algorithm algorithm) {
    std::pair<uint32_t, uint32_t> maxResolution{0, 0};
    for(const auto& profile : BACKEND_PROFILES) {
        if(profile.algorithm != algorithm) continue;
        maxResolution.first = std::max(maxResolution.first, profile.maxWidth);
        maxResolution.second = std::max(maxResolution.second, profile.maxHeight);
    }
    return maxResolution;
}

float targetFpsWithDefault(float targetFps) {
    return targetFps > 0.f ? targetFps : DEFAULT_TARGET_FPS;
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

void Depth::requireNotBuilt(const char* method) const {
    DAI_CHECK_V(!graphBuilt_, "{} must be called before the graph is wired (before first depth()/confidence() access).", method);
}

std::shared_ptr<Depth> Depth::build(std::optional<float> fps) {
    requireNotBuilt("Depth::build(fps)");
    stereoOutputFps_ = fps;  // Applied to Camera outputs when a stereo-based backend is wired.
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm, std::optional<float> fps) {
    return build(algorithm, fps, std::nullopt);
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm,
                                      std::optional<float> fps,
                                      std::optional<std::pair<uint32_t, uint32_t>> stereoSize) {
    requireNotBuilt("Depth::build(algorithm, fps, stereoSize)");
    algorithmOverride_ = algorithm;
    stereoOutputFps_ = fps;
    stereoSizeOverride_ = stereoSize;
    configOverride_.reset();
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm,
                                      Config config,
                                      std::optional<float> fps,
                                      std::optional<std::pair<uint32_t, uint32_t>> stereoSize) {
    requireNotBuilt("Depth::build(algorithm, config, fps, stereoSize)");
    algorithmOverride_ = algorithm;
    configOverride_ = std::move(config);
    stereoOutputFps_ = fps;
    stereoSizeOverride_ = stereoSize;
    neuralModelOverride_.reset();
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::setAlgorithm(Algorithm algorithm) {
    requireNotBuilt("Depth::setAlgorithm");
    algorithmOverride_ = algorithm;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::setConfig(Config config) {
    requireNotBuilt("Depth::setConfig");
    configOverride_ = std::move(config);
    neuralModelOverride_.reset();
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(DeviceModelZoo neuralModel) {
    requireNotBuilt("Depth::build(neuralModel)");
    neuralModelOverride_ = neuralModel;
    configOverride_.reset();
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
    const auto [maxWidth, maxHeight] = maxResolutionForAlgorithm(Algorithm::STEREO);
    return width > maxWidth || height > maxHeight;
}

Depth::Selection Depth::selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                       float targetFps,
                                       const std::vector<Algorithm>& supportedAlgorithms,
                                       const std::vector<DeviceModelZoo>& supportedModels) {
    targetFps = targetFpsWithDefault(targetFps);
    const float requiredFps = targetFps * SELECTION_FPS_SAFETY_MARGIN;

    // 1. Priority scan honoring the requested FPS and (optional) resolution.
    if(const auto picked = findBackendProfile([&](const BackendProfile& profile) {
           return profileMatches(profile, supportedAlgorithms, supportedModels, requiredFps, resolution);
       })) {
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

    if(configOverride_) {
        const auto supportedModels = algorithmOverride_ == Algorithm::NEURAL ? device->getSupportedDeviceModels() : std::vector<DeviceModelZoo>{};
        validateExplicitConfig(algorithmOverride_, *configOverride_, supported, supportedModels);
        resolved_ = {algorithmOverride_, *configOverride_};
        return;
    }

    if(algorithmOverride_ != Algorithm::AUTO) {
        DAI_CHECK_V(supportsAlgorithm(supported, algorithmOverride_),
                    "Depth algorithm {} is not supported on this device.",
                    algorithmName(algorithmOverride_));
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
    targetFps = targetFpsWithDefault(targetFps);

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
        const auto [maxW, maxH] = maxResolutionForAlgorithm(resolved_.algorithm);
        DAI_CHECK_V(anyProfileFits(resolved_.algorithm, *resolution),
                    "Depth: resolution exceeds the maximum supported by the requested algorithm "
                    "(largest profile: {}x{}).",
                    maxW,
                    maxH);
    }

    switch(resolved_.algorithm) {
        case Algorithm::NEURAL:
            if(neuralModelOverride_) {
                resolved_.config = *neuralModelOverride_;
            } else if(const auto model = pickNeuralModel(targetFps * SELECTION_FPS_SAFETY_MARGIN, resolution, supportedModels)) {
                resolved_.config = *model;
            } else {
                resolved_.config = DEFAULT_NEURAL_DEPTH_MODEL;
            }
            break;
        case Algorithm::STEREO:
            if(const auto preset = pickStereoPreset(targetFps * SELECTION_FPS_SAFETY_MARGIN, resolution)) {
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
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), std::nullopt, stereoOutputFps_);
            nasBackend_->build(*leftOut, *rightOut, DEFAULT_NAS_NEURAL_MODEL, DEFAULT_NAS_RECTIFY);
            break;
        }
        case Algorithm::GPU_STEREO: {
            gpuStereoBackend_ = std::make_unique<Subnode<GPUStereo>>(*this, "gpuStereo");
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), std::nullopt, stereoOutputFps_);
            (*gpuStereoBackend_)->setRectification(true).build(*leftOut, *rightOut);
            break;
        }
        case Algorithm::NEURAL: {
            const auto model = neuralModelFromConfig(resolved_.config);
            const auto is = NeuralDepth::getInputSize(model);
            const std::pair<uint32_t, uint32_t> monoSize{static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
            neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth");
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), monoSize, stereoOutputFps_);
            (*neuralBackend_)->build(*leftOut, *rightOut, model);
            break;
        }
        case Algorithm::STEREO: {
            stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoDepth");
            auto [leftOut, rightOut] = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), std::nullopt, stereoOutputFps_);
            (*stereoBackend_)->build(*leftOut, *rightOut, stereoPresetFromConfig(resolved_.config));
            break;
        }
    }

    bindBackendOutputs(active);
    graphBuilt_ = true;
}

// --- Stereo camera wiring ---

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
