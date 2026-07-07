#include "depthai/pipeline/node/Depth.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <optional>
#include <utility>
#include <variant>

#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

// Anonymous helpers: device capability probes and stereo camera discovery.
namespace {

constexpr DeviceModelZoo DEFAULT_NEURAL_DEPTH_MODEL = DeviceModelZoo::NEURAL_DEPTH_MEDIUM;
constexpr DeviceModelZoo DEFAULT_NAS_NEURAL_MODEL = DeviceModelZoo::NEURAL_DEPTH_NANO;
constexpr bool DEFAULT_NAS_RECTIFY = true;
constexpr float DEFAULT_TARGET_FPS = 30.f;
constexpr float SELECTION_FPS_SAFETY_MARGIN = 0.9f;
constexpr float NEURAL_TENSOR_COVER_SCALE = 1.4142135f;
constexpr uint32_t STEREO_DEPTH_WIDTH_ALIGNMENT = 16;

// Backend profiles for AUTO selection: config, max resolution, and max FPS per algorithm.
// Neural FPS values: https://docs.luxonis.com/overview/toplevel-features/depth/ (RVC4 Neural Stereo table).
// Rows are in priority order (NEURAL, NEURAL_ASSISTED_STEREO, STEREO, GPU_STEREO).
struct BackendProfile {
    Depth::Algorithm algorithm;
    Depth::Config config;
    std::pair<uint32_t, uint32_t> maxSize;
    float maxFps;
};

constexpr BackendProfile neural(DeviceModelZoo model, uint32_t width, uint32_t height, float maxFps) {
    return {Depth::Algorithm::NEURAL, model, {width, height}, maxFps};
}

constexpr BackendProfile stereo(StereoDepth::PresetMode preset, float maxFps) {
    return {Depth::Algorithm::STEREO, preset, {1280, 1280}, maxFps};
}

constexpr BackendProfile noConfig(Depth::Algorithm algorithm, uint32_t width, uint32_t height, float maxFps) {
    return {algorithm, std::monostate{}, {width, height}, maxFps};
}

inline constexpr std::array BACKEND_PROFILES = {
    neural(DeviceModelZoo::NEURAL_DEPTH_1248X780, 1248, 780, 8.5f),
    neural(DeviceModelZoo::NEURAL_DEPTH_1056X660, 1056, 660, 12.5f),
    neural(DeviceModelZoo::NEURAL_DEPTH_960X600, 960, 600, 14.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_864X540, 864, 540, 18.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_768X480, 768, 480, 22.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_576X360, 576, 360, 38.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_480X300, 480, 300, 56.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_384X240, 384, 240, 60.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_288X180, 288, 180, 60.f),
    neural(DeviceModelZoo::NEURAL_DEPTH_192X120, 192, 120, 60.f),

    noConfig(Depth::Algorithm::NEURAL_ASSISTED_STEREO, 1280, 800, 55.f),

    stereo(StereoDepth::PresetMode::DEFAULT, 16.f),
    stereo(StereoDepth::PresetMode::FAST_ACCURACY, 30.f),
    stereo(StereoDepth::PresetMode::FAST_DENSITY, 30.f),

    noConfig(Depth::Algorithm::GPU_STEREO, 2592, 1944, 5.f),
    noConfig(Depth::Algorithm::GPU_STEREO, 1280, 800, 30.f),
    noConfig(Depth::Algorithm::GPU_STEREO, 640, 400, 55.f),
};

bool modelPassesOptionalFilter(DeviceModelZoo model, const std::vector<DeviceModelZoo>& modelFilter) {
    return modelFilter.empty() || std::find(modelFilter.begin(), modelFilter.end(), model) != modelFilter.end();
}

bool isModelSupportedByDevice(DeviceModelZoo model, const std::vector<DeviceModelZoo>& supportedModels) {
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

const char* presetModeName(StereoDepth::PresetMode preset) {
    switch(preset) {
        case StereoDepth::PresetMode::FAST_ACCURACY:
            return "FAST_ACCURACY";
        case StereoDepth::PresetMode::FAST_DENSITY:
            return "FAST_DENSITY";
        case StereoDepth::PresetMode::DEFAULT:
            return "DEFAULT";
        case StereoDepth::PresetMode::FACE:
            return "FACE";
        case StereoDepth::PresetMode::HIGH_DETAIL:
            return "HIGH_DETAIL";
        case StereoDepth::PresetMode::ROBOTICS:
            return "ROBOTICS";
        case StereoDepth::PresetMode::DENSITY:
            return "DENSITY";
        case StereoDepth::PresetMode::ACCURACY:
            return "ACCURACY";
    }
    return "UNKNOWN";
}

const char* deviceModelZooName(DeviceModelZoo model) {
    switch(model) {
        case DeviceModelZoo::NEURAL_DEPTH_1248X780:
            return "NEURAL_DEPTH_EXTRA_LARGE";
        case DeviceModelZoo::NEURAL_DEPTH_768X480:
            return "NEURAL_DEPTH_LARGE";
        case DeviceModelZoo::NEURAL_DEPTH_576X360:
            return "NEURAL_DEPTH_MEDIUM";
        case DeviceModelZoo::NEURAL_DEPTH_480X300:
            return "NEURAL_DEPTH_SMALL";
        case DeviceModelZoo::NEURAL_DEPTH_384X240:
            return "NEURAL_DEPTH_NANO";
        case DeviceModelZoo::NEURAL_DEPTH_1056X660:
            return "NEURAL_DEPTH_1056X660";
        case DeviceModelZoo::NEURAL_DEPTH_960X600:
            return "NEURAL_DEPTH_960X600";
        case DeviceModelZoo::NEURAL_DEPTH_864X540:
            return "NEURAL_DEPTH_864X540";
        case DeviceModelZoo::NEURAL_DEPTH_288X180:
            return "NEURAL_DEPTH_288X180";
        case DeviceModelZoo::NEURAL_DEPTH_192X120:
            return "NEURAL_DEPTH_192X120";
    }
    return "UNKNOWN";
}

const char* configName(const Depth::Config& config) {
    if(std::holds_alternative<std::monostate>(config)) {
        return "none";
    }
    if(const auto* model = std::get_if<DeviceModelZoo>(&config)) {
        return deviceModelZooName(*model);
    }
    if(const auto* preset = std::get_if<StereoDepth::PresetMode>(&config)) {
        return presetModeName(*preset);
    }
    return "UNKNOWN";
}

void validateExplicitConfig(Depth::Algorithm algorithm,
                            const Depth::Config& config,
                            const std::vector<Depth::Algorithm>& supportedAlgorithms,
                            const std::vector<DeviceModelZoo>& supportedModels) {
    DAI_CHECK_V(algorithm != Depth::Algorithm::AUTO, "Depth: a config can only be pinned together with a concrete algorithm (not AUTO).");
    DAI_CHECK_V(supportsAlgorithm(supportedAlgorithms, algorithm), "Depth selected algorithm {} is not supported on this device.", algorithmName(algorithm));

    switch(algorithm) {
        case Depth::Algorithm::NEURAL: {
            const auto* model = std::get_if<DeviceModelZoo>(&config);
            DAI_CHECK_V(model != nullptr, "Depth config for NEURAL must be a DeviceModelZoo model.");
            DAI_CHECK_V(isModelSupportedByDevice(*model, supportedModels), "Depth selected NeuralDepth model is not available on this device.");
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

// NEURAL uses a tensor-cover band around the model size; other algorithms use a simple max cap.
bool resolutionFits(const BackendProfile& profile, std::pair<uint32_t, uint32_t> resolution) {
    const auto [maxWidth, maxHeight] = profile.maxSize;
    if(profile.algorithm == Depth::Algorithm::NEURAL) {
        const uint32_t minW = static_cast<uint32_t>(static_cast<float>(maxWidth) / NEURAL_TENSOR_COVER_SCALE);
        const uint32_t maxW = static_cast<uint32_t>(static_cast<float>(maxWidth) * NEURAL_TENSOR_COVER_SCALE);
        const uint32_t minH = static_cast<uint32_t>(static_cast<float>(maxHeight) / NEURAL_TENSOR_COVER_SCALE);
        const uint32_t maxH = static_cast<uint32_t>(static_cast<float>(maxHeight) * NEURAL_TENSOR_COVER_SCALE);
        return resolution.first >= minW && resolution.first <= maxW && resolution.second >= minH && resolution.second <= maxH;
    }
    return resolution.first <= maxWidth && resolution.second <= maxHeight;
}

bool profilePassesModelFilter(const BackendProfile& profile, const std::vector<DeviceModelZoo>& modelFilter) {
    if(const auto* model = std::get_if<DeviceModelZoo>(&profile.config)) {
        return modelPassesOptionalFilter(*model, modelFilter);
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

bool profileMatches(const BackendProfile& profile,
                    const std::vector<Depth::Algorithm>& supportedAlgorithms,
                    const std::vector<DeviceModelZoo>& modelFilter,
                    float requiredFps,
                    std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    if(!supportsAlgorithm(supportedAlgorithms, profile.algorithm)) {
        return false;
    }
    if(profile.maxFps < requiredFps) {
        return false;
    }
    if(!profilePassesModelFilter(profile, modelFilter)) {
        return false;
    }
    if(resolution.has_value() && !resolutionFits(profile, *resolution)) {
        return false;
    }
    return true;
}

// Pick the highest-FPS profile that fits the resolution when the priority scan finds no exact match.
std::optional<BackendProfile> pickHighestFpsForResolution(std::pair<uint32_t, uint32_t> resolution,
                                                          const std::vector<Depth::Algorithm>& supportedAlgorithms,
                                                          const std::vector<DeviceModelZoo>& modelFilter) {
    std::optional<BackendProfile> best;
    for(const auto& profile : BACKEND_PROFILES) {
        if(!supportsAlgorithm(supportedAlgorithms, profile.algorithm)) continue;
        if(!profilePassesModelFilter(profile, modelFilter)) continue;
        if(!resolutionFits(profile, resolution)) continue;
        if(!best || profile.maxFps > best->maxFps) {
            best = profile;
        }
    }
    return best;
}

std::optional<DeviceModelZoo> pickNeuralModel(float requiredFps,
                                              std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                              const std::vector<DeviceModelZoo>& modelFilter) {
    const auto picked = findBackendProfile([&](const BackendProfile& profile) {
        return profile.algorithm == Depth::Algorithm::NEURAL && profile.maxFps >= requiredFps && profilePassesModelFilter(profile, modelFilter)
               && (!resolution.has_value() || resolutionFits(profile, *resolution));
    });
    if(picked) {
        return std::get<DeviceModelZoo>(picked->config);
    }
    return std::nullopt;
}

std::optional<StereoDepth::PresetMode> pickStereoPreset(float requiredFps, std::optional<std::pair<uint32_t, uint32_t>> resolution) {
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
    return findBackendProfile([&](const BackendProfile& profile) { return profile.algorithm == algorithm && resolutionFits(profile, resolution); }).has_value();
}

std::pair<uint32_t, uint32_t> maxResolutionForAlgorithm(Depth::Algorithm algorithm) {
    std::pair<uint32_t, uint32_t> maxResolution{0, 0};
    for(const auto& profile : BACKEND_PROFILES) {
        if(profile.algorithm != algorithm) continue;
        maxResolution.first = std::max(maxResolution.first, profile.maxSize.first);
        maxResolution.second = std::max(maxResolution.second, profile.maxSize.second);
    }
    return maxResolution;
}

float targetFpsWithDefault(float targetFps) {
    return targetFps > 0.f ? targetFps : DEFAULT_TARGET_FPS;
}

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

std::optional<std::pair<uint32_t, uint32_t>> stereoSizeFromExistingCameras(const std::shared_ptr<Camera>& left, const std::shared_ptr<Camera>& right) {
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

StereoPair requireFirstStereoPair(const std::shared_ptr<Device>& device) {
    const auto pairs = device->getStereoPairs();
    DAI_CHECK_V(!pairs.empty(), "Device has no stereo camera pair for Depth node.");
    return pairs[0];
}

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

void validateStereoDepthResolution(uint32_t width, uint32_t height) {
    DAI_CHECK_V(width % STEREO_DEPTH_WIDTH_ALIGNMENT == 0,
                "Depth: StereoDepth requires width to be a multiple of {} (requested {}x{}).",
                STEREO_DEPTH_WIDTH_ALIGNMENT,
                width,
                height);
}

struct WiringInputs {
    float targetFps{};
    std::optional<std::pair<uint32_t, uint32_t>> resolution{};
};

WiringInputs gatherWiringInputs(const std::shared_ptr<Device>& device,
                                const StereoPair& pair,
                                const std::shared_ptr<Camera>& left,
                                const std::shared_ptr<Camera>& right,
                                std::optional<float> stereoOutputFps,
                                std::optional<std::pair<uint32_t, uint32_t>> stereoSizeOverride) {
    WiringInputs inputs{};
    inputs.targetFps = stereoOutputFps.value_or(0.f);
    if(inputs.targetFps <= 0.f && left && right) {
        inputs.targetFps = std::max(left->getMaxRequestedFps(), right->getMaxRequestedFps());
    }
    inputs.targetFps = targetFpsWithDefault(inputs.targetFps);

    inputs.resolution = stereoSizeOverride;
    if(!inputs.resolution) {
        inputs.resolution = stereoSizeFromExistingCameras(left, right);
    }
    if(!inputs.resolution) {
        inputs.resolution = stereoSizeFromDeviceFeatures(device, pair);
    }
    return inputs;
}

std::optional<StereoDepth::PresetMode> pickFastestStereoPreset(std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    const BackendProfile* best = nullptr;
    for(const auto& profile : BACKEND_PROFILES) {
        if(profile.algorithm != Depth::Algorithm::STEREO) {
            continue;
        }
        if(resolution.has_value() && !resolutionFits(profile, *resolution)) {
            continue;
        }
        if(best == nullptr || profile.maxFps > best->maxFps) {
            best = &profile;
        }
    }
    if(best != nullptr) {
        return std::get<StereoDepth::PresetMode>(best->config);
    }
    return std::nullopt;
}

StereoDepth::PresetMode resolveStereoPreset(float targetFps, std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    if(const auto preset = pickStereoPreset(targetFps * SELECTION_FPS_SAFETY_MARGIN, resolution)) {
        return *preset;
    }
    if(const auto preset = pickFastestStereoPreset(resolution)) {
        return *preset;
    }
    return StereoDepth::PresetMode::FAST_ACCURACY;
}

}  // namespace

// --- Construction ---

Depth::Depth() : DeviceNodeGroup(nullptr) {}

// --- Build methods ---

Node::Output& Depth::RequestedOutput::OutputHandle::get() const {
    return (parent_.*resolver_)();
}

Node::Output* Depth::RequestedOutput::OutputHandle::operator->() const {
    return &get();
}

Depth::RequestedOutput::OutputHandle::operator Node::Output&() const {
    return get();
}

std::shared_ptr<Depth> Depth::build() {
    requireNotBuilt("Depth::build()");
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm) {
    requireNotBuilt("Depth::build(algorithm)");
    algorithmOverride_ = algorithm;
    configOverride_.reset();
    return std::static_pointer_cast<Depth>(shared_from_this());
}

Depth::RequestedOutput* Depth::requestOutput(std::optional<float> fps) {
    requireNotBuilt("Depth::requestOutput(fps)");
    stereoSizeOverride_.reset();
    stereoOutputFps_ = fps;
    if(!requestedOutput_) {
        requestedOutput_ = std::unique_ptr<RequestedOutput>(new RequestedOutput(*this));
    }
    return requestedOutput_.get();
}

Depth::RequestedOutput* Depth::requestOutput(const std::pair<uint32_t, uint32_t>& size, std::optional<float> fps) {
    requireNotBuilt("Depth::requestOutput(size, fps)");
    stereoSizeOverride_ = size;
    stereoOutputFps_ = fps;
    if(!requestedOutput_) {
        requestedOutput_ = std::unique_ptr<RequestedOutput>(new RequestedOutput(*this));
    }
    return requestedOutput_.get();
}

std::shared_ptr<Depth> Depth::build(Algorithm algorithm, Config config) {
    requireNotBuilt("Depth::build(algorithm, config)");
    algorithmOverride_ = algorithm;
    configOverride_ = std::move(config);
    return std::static_pointer_cast<Depth>(shared_from_this());
}

// --- Outputs ---

Node::Output& Depth::resolveDepthOutput() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(depthOut_ != nullptr, "Depth backend output missing.");
    return *depthOut_;
}

Node::Output& Depth::resolveConfidenceOutput() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(confidenceOut_ != nullptr, "Depth backend confidence output missing.");
    return *confidenceOut_;
}

// --- Setters ---

std::shared_ptr<Depth> Depth::setAlgorithm(Algorithm algorithm) {
    requireNotBuilt("Depth::setAlgorithm");
    algorithmOverride_ = algorithm;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::setConfig(Config config) {
    requireNotBuilt("Depth::setConfig");
    configOverride_ = std::move(config);
    return std::static_pointer_cast<Depth>(shared_from_this());
}

std::shared_ptr<Depth> Depth::setAlignTo(Node::Output& alignTo) {
    requireNotBuilt("Depth::setAlignTo");
    alignToOutput_ = &alignTo;
    return std::static_pointer_cast<Depth>(shared_from_this());
}

// --- Internal ---

void Depth::requireNotBuilt(const char* method) const {
    DAI_CHECK_V(!graphBuilt_, "{} must be called before the graph is wired (before first requested output access).", method);
}

std::vector<Depth::Algorithm> Depth::getSupportedAlgorithms(const std::shared_ptr<Device>& device, const std::vector<DeviceModelZoo>& supportedModels) const {
    std::vector<Algorithm> supported = {Algorithm::STEREO};

    const bool isRvc4 = device->getPlatform() == Platform::RVC4;
    if(isRvc4 && device->isNeuralDepthSupported()) {
        // NEURAL is only usable when the device actually has a model-zoo depth model available.
        if(!supportedModels.empty()) {
            supported.push_back(Algorithm::NEURAL);
        }
        supported.push_back(Algorithm::NEURAL_ASSISTED_STEREO);
    }
    if(isRvc4 && device->isGpuStereoSupported() && !device->getStereoPairs().empty()) {
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
                                      const std::vector<DeviceModelZoo>& modelFilter,
                                      bool requireFpsAndResolutionMatch) {
    targetFps = targetFpsWithDefault(targetFps);
    const float requiredFps = targetFps * SELECTION_FPS_SAFETY_MARGIN;

    // 1. Priority scan honoring the requested FPS and (optional) resolution.
    if(const auto picked = findBackendProfile(
           [&](const BackendProfile& profile) { return profileMatches(profile, supportedAlgorithms, modelFilter, requiredFps, resolution); })) {
        return {picked->algorithm, picked->config};
    }

    // When the user explicitly pinned both FPS and resolution, do not silently fall back to a
    // backend that cannot serve them; surface a clear error instead.
    DAI_CHECK_V(!requireFpsAndResolutionMatch,
                "Depth: no available algorithm can serve the requested resolution {}x{} at {} FPS.",
                resolution.has_value() ? resolution->first : 0,
                resolution.has_value() ? resolution->second : 0,
                targetFps);

    // 2. No backend can serve the resolution at the requested FPS: pick the algorithm whose
    //    catalog row covers the resolution with the highest available maxFps.
    if(resolution.has_value()) {
        if(const auto picked = pickHighestFpsForResolution(*resolution, supportedAlgorithms, modelFilter)) {
            return {picked->algorithm, picked->config};
        }
    }

    // 3. Last-resort fallback: fastest StereoDepth preset that fits resolution, if any.
    if(supportsAlgorithm(supportedAlgorithms, Algorithm::STEREO)) {
        if(const auto preset = pickFastestStereoPreset(resolution)) {
            return {Algorithm::STEREO, *preset};
        }
    }
    return {Algorithm::STEREO, StereoDepth::PresetMode::FAST_ACCURACY};
}

void Depth::resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline) {
    const bool isRvc4 = device->getPlatform() == Platform::RVC4;

    // getSupportedDeviceModels() is an RVC4-only RPC; on other platforms there is no model catalog.
    const auto supportedModels = isRvc4 ? device->getSupportedDeviceModels() : std::vector<DeviceModelZoo>{};
    const auto supported = getSupportedAlgorithms(device, supportedModels);

    if(configOverride_) {
        validateExplicitConfig(
            algorithmOverride_, *configOverride_, supported, algorithmOverride_ == Algorithm::NEURAL ? supportedModels : std::vector<DeviceModelZoo>{});
        resolved_ = {algorithmOverride_, *configOverride_};
        if(resolved_.algorithm == Algorithm::STEREO && stereoSizeOverride_) {
            validateStereoDepthResolution(stereoSizeOverride_->first, stereoSizeOverride_->second);
        }
        return;
    }

    if(algorithmOverride_ != Algorithm::AUTO) {
        DAI_CHECK_V(supportsAlgorithm(supported, algorithmOverride_), "Depth algorithm {} is not supported on this device.", algorithmName(algorithmOverride_));
    }

    if(!isRvc4) {
        const Algorithm chosen =
            (algorithmOverride_ == Algorithm::AUTO) ? (supportsAlgorithm(supported, Algorithm::TOF) ? Algorithm::TOF : Algorithm::STEREO) : algorithmOverride_;
        Config config = std::monostate{};
        if(chosen == Algorithm::STEREO) {
            const auto pair = requireFirstStereoPair(device);
            const auto [left, right] = findCamerasForPair(pipeline, pair);
            const auto inputs = gatherWiringInputs(device, pair, left, right, stereoOutputFps_, stereoSizeOverride_);
            if(inputs.resolution) {
                validateStereoDepthResolution(inputs.resolution->first, inputs.resolution->second);
            }
            config = resolveStereoPreset(inputs.targetFps, inputs.resolution);
        }
        resolved_ = {chosen, config};
        return;
    }

    const auto pair = requireFirstStereoPair(device);
    const auto [left, right] = findCamerasForPair(pipeline, pair);
    const auto inputs = gatherWiringInputs(device, pair, left, right, stereoOutputFps_, stereoSizeOverride_);
    const float targetFps = inputs.targetFps;
    const auto& resolution = inputs.resolution;

    if(algorithmOverride_ == Algorithm::AUTO) {
        // Only enforce an exact FPS+resolution match when the user pinned both before requesting the output.
        const bool userPinnedFpsAndResolution = stereoOutputFps_.has_value() && stereoSizeOverride_.has_value();
        resolved_ = selectBackend(resolution, targetFps, supported, supportedModels, userPinnedFpsAndResolution);
        if(resolved_.algorithm == Algorithm::STEREO && resolution) {
            validateStereoDepthResolution(resolution->first, resolution->second);
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

    if(resolved_.algorithm == Algorithm::STEREO && resolution) {
        validateStereoDepthResolution(resolution->first, resolution->second);
    }

    switch(resolved_.algorithm) {
        case Algorithm::NEURAL:
            if(const auto model = pickNeuralModel(targetFps * SELECTION_FPS_SAFETY_MARGIN, resolution, supportedModels)) {
                resolved_.config = *model;
            } else {
                resolved_.config = DEFAULT_NEURAL_DEPTH_MODEL;
            }
            break;
        case Algorithm::STEREO:
            resolved_.config = resolveStereoPreset(targetFps, resolution);
            break;
        case Algorithm::NEURAL_ASSISTED_STEREO:
        case Algorithm::GPU_STEREO:
        case Algorithm::TOF:
        case Algorithm::AUTO:
            break;
    }
}

Depth::StereoWiring Depth::ensureStereoOutputs(Pipeline& pipeline,
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

    const std::pair<uint32_t, uint32_t> resolution{left->getMaxRequestedWidth(), left->getMaxRequestedHeight()};
    DAI_CHECK_V(resolution.first > 0 && resolution.second > 0, "Depth: failed to determine wired stereo resolution.");
    const float maxCameraFps = std::max(left->getMaxRequestedFps(), right->getMaxRequestedFps());
    return {lo, ro, resolution, maxCameraFps};
}

void Depth::wireAlignment(Algorithm active, const std::shared_ptr<Device>& device) {
    if(alignToOutput_ == nullptr) {
        return;
    }
    DAI_CHECK_V(depthOut_ != nullptr, "Depth: cannot align an unbound depth output.");

    const bool isRvc4 = device->getPlatform() == Platform::RVC4;

    // RVC2 + StereoDepth: alignment is handled natively on-device via StereoDepth's inputAlignTo,
    // which aligns the depth output without an extra node.
    if(!isRvc4 && active == Algorithm::STEREO) {
        alignToOutput_->link((*stereoBackend_)->inputAlignTo);
        return;
    }

    // Every other case (all RVC4 backends, and non-StereoDepth backends on RVC2 such as ToF) routes
    // the backend depth through an ImageAlign node. On RVC2 there is no hardware ImageAlign path for
    // these backends, so the node runs on host.
    imageAlignBackend_ = std::make_unique<Subnode<ImageAlign>>(*this, "imageAlign");
    if(!isRvc4) {
        (*imageAlignBackend_)->setRunOnHost(true);
    }
    depthOut_->link((*imageAlignBackend_)->input);
    alignToOutput_->link((*imageAlignBackend_)->inputAlignTo);
    depthOut_ = &(*imageAlignBackend_)->outputAligned;
}

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
    DAI_CHECK_V(device != nullptr,
                "Depth node requires a device (set on create, when added to a pipeline with a default device, or from pipeline at first wiring).");

    resolveWiring(device, pipeline);
    const Algorithm active = resolved_.algorithm;

    std::optional<float> wiredFps = stereoOutputFps_;
    std::optional<std::pair<uint32_t, uint32_t>> wiredResolution;

    switch(active) {
        case Algorithm::AUTO:
            DAI_CHECK_V(false, "Depth: AUTO must be resolved before wiring.");
            break;
        case Algorithm::TOF:
            tofBackend_ = ToF::create(device);
            add(tofBackend_);
            tofBackend_->build(CameraBoardSocket::AUTO, ImageFiltersPresetMode::TOF_MID_RANGE, stereoOutputFps_);
            depthOut_ = &tofBackend_->depth;
            confidenceOut_ = &tofBackend_->amplitude;  // ToF "confidence" is amplitude.
            break;
        case Algorithm::NEURAL_ASSISTED_STEREO: {
            nasBackend_ = std::make_shared<NeuralAssistedStereo>(device);
            add(nasBackend_);
            const auto stereo = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), stereoSizeOverride_, stereoOutputFps_);
            nasBackend_->build(*stereo.left, *stereo.right, DEFAULT_NAS_NEURAL_MODEL, DEFAULT_NAS_RECTIFY);
            depthOut_ = &nasBackend_->depth;
            confidenceOut_ = &(*nasBackend_->stereoDepth).confidenceMap;
            wiredResolution = stereo.resolution;
            if((!wiredFps || *wiredFps <= 0.f) && stereo.maxCameraFps > 0.f) {
                wiredFps = stereo.maxCameraFps;
            }
            break;
        }
        case Algorithm::GPU_STEREO: {
            gpuStereoBackend_ = std::make_unique<Subnode<GPUStereo>>(*this, "gpuStereo");
            const auto stereo = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), stereoSizeOverride_, stereoOutputFps_);
            (*gpuStereoBackend_)->setRectification(true).build(*stereo.left, *stereo.right);
            depthOut_ = &(**gpuStereoBackend_).depth;
            confidenceOut_ = &(**gpuStereoBackend_).confidenceMap;
            wiredResolution = stereo.resolution;
            if((!wiredFps || *wiredFps <= 0.f) && stereo.maxCameraFps > 0.f) {
                wiredFps = stereo.maxCameraFps;
            }
            break;
        }
        case Algorithm::NEURAL: {
            const auto model = neuralModelFromConfig(resolved_.config);
            const auto is = NeuralDepth::getInputSize(model);
            const std::pair<uint32_t, uint32_t> monoSize{static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
            neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth");
            const auto stereo = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), monoSize, stereoOutputFps_);
            (*neuralBackend_)->build(*stereo.left, *stereo.right, model);
            depthOut_ = &(**neuralBackend_).depth;
            confidenceOut_ = &(**neuralBackend_).confidence;
            wiredResolution = stereo.resolution;
            if((!wiredFps || *wiredFps <= 0.f) && stereo.maxCameraFps > 0.f) {
                wiredFps = stereo.maxCameraFps;
            }
            break;
        }
        case Algorithm::STEREO: {
            stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoDepth");
            const auto stereo = ensureStereoOutputs(pipeline, requireFirstStereoPair(device), stereoSizeOverride_, stereoOutputFps_);
            validateStereoDepthResolution(stereo.resolution.first, stereo.resolution.second);
            (*stereoBackend_)->build(*stereo.left, *stereo.right, stereoPresetFromConfig(resolved_.config));
            depthOut_ = &(**stereoBackend_).depth;
            confidenceOut_ = &(**stereoBackend_).confidenceMap;
            wiredResolution = stereo.resolution;
            if((!wiredFps || *wiredFps <= 0.f) && stereo.maxCameraFps > 0.f) {
                wiredFps = stereo.maxCameraFps;
            }
            break;
        }
    }

    wireAlignment(active, device);

    if(!wiredFps || *wiredFps <= 0.f) {
        wiredFps = DEFAULT_TARGET_FPS;
    }

    if(active == Algorithm::TOF) {
        pimpl->logger->info("Depth wired: algorithm={}, config={}, fps={:.1f}, resolution=sensor-native",
                            algorithmName(resolved_.algorithm),
                            configName(resolved_.config),
                            *wiredFps);
    } else {
        DAI_CHECK_V(wiredResolution.has_value(), "Depth: missing wired stereo resolution.");
        pimpl->logger->info("Depth wired: algorithm={}, config={}, fps={:.1f}, resolution={}x{}",
                            algorithmName(resolved_.algorithm),
                            configName(resolved_.config),
                            *wiredFps,
                            wiredResolution->first,
                            wiredResolution->second);
    }

    graphBuilt_ = true;
}

}  // namespace node
}  // namespace dai
