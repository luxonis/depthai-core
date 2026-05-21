/**
 * @file Depth.cpp
 * @brief Implementation of the composite Depth node: algorithm selection, lazy backend wiring,
 *        stereo camera provisioning, and unified depth/confidence outputs.
 */

#include "depthai/pipeline/node/Depth.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>
#include <optional>
#include <string>
#include <utility>

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

// Anonymous helpers: device capability probes, board-revision parsing, and stereo camera discovery.
// Used by selectAlgorithm(), getSupportedAlgorithms(), and ensureStereoOutputs().
namespace {

// Depth algorithm selection is device-dependent:
// - TOF requires a connected ToF sensor.
// - NEURAL_ASSISTED_STEREO / GPU_STEREO require RVC4 (GPUStereo also requires board R9+).
// - AUTO prefers NEURAL on RVC4, TOF on RVC2 with ToF, otherwise STEREO.

/** Minimum EEPROM major revision (P9/R9) for GPUStereo hardware on RVC4. */
constexpr int kGpuStereoMinBoardRevisionMajor = 9;

/** Fallback NeuralDepth model when device zoo is unavailable. */
constexpr DeviceModelZoo kDefaultNeuralDepthModel = DeviceModelZoo::NEURAL_DEPTH_SMALL;
/** Default NeuralAssistedStereo assist model. */
constexpr DeviceModelZoo kDefaultNasNeuralModel = DeviceModelZoo::NEURAL_DEPTH_NANO;
constexpr bool kDefaultNasRectify = true;

/** RVC4 depth algorithm limits (tune when official perf tables are available). */
constexpr uint32_t kStereoDepthMaxWidth = 1280;
constexpr uint32_t kStereoDepthMaxHeight = 1280;
constexpr float kNasMaxFps = 60.f;
constexpr float kGpuStereoMaxFps = 30.f;
constexpr float kSelectionFpsSafetyMargin = 0.85f;

bool deviceHasGpuStereoHardware(const std::shared_ptr<Device>& device);

/**
 * NeuralDepth profile entry: the user-visible "profile" for NEURAL is the @ref DeviceModelZoo model.
 * Per-axis user-frame bounds are ~tensor/sqrt(2) (min) and ~tensor*sqrt(2) (max), so each model
 * covers a comfortable resize range without aggressive up- or down-sampling.
 * Ordered largest-tensor first (preferred when several rows match).
 */
struct NeuralDepthProfile {
    DeviceModelZoo model;
    uint32_t maxUserWidth;
    uint32_t maxUserHeight;
    uint32_t minUserWidth;
    uint32_t minUserHeight;
    float maxFps;
};
constexpr std::array<NeuralDepthProfile, 10> kNeuralDepthProfiles = {{
    {DeviceModelZoo::NEURAL_DEPTH_1248X780, 1765, 1103, 882, 551, 1.6f},
    {DeviceModelZoo::NEURAL_DEPTH_1056X660, 1493, 933, 746, 466, 3.0f},
    {DeviceModelZoo::NEURAL_DEPTH_960X600, 1357, 848, 679, 424, 5.0f},
    {DeviceModelZoo::NEURAL_DEPTH_864X540, 1221, 763, 610, 381, 8.0f},
    {DeviceModelZoo::NEURAL_DEPTH_768X480, 1085, 678, 543, 339, 10.0f},
    {DeviceModelZoo::NEURAL_DEPTH_576X360, 814, 509, 407, 254, 24.0f},
    {DeviceModelZoo::NEURAL_DEPTH_480X300, 678, 424, 339, 212, 40.0f},
    {DeviceModelZoo::NEURAL_DEPTH_384X240, 543, 339, 271, 170, 55.0f},
    {DeviceModelZoo::NEURAL_DEPTH_288X180, 407, 254, 203, 127, 55.0f},
    {DeviceModelZoo::NEURAL_DEPTH_192X120, 271, 170, 136, 85, 55.0f},
}};

/**
 * StereoDepth profile entry: the user-visible "profile" for STEREO is the @ref StereoDepth::PresetMode.
 * Resolution cap is the same for every preset (kStereoDepthMaxWidth x kStereoDepthMaxHeight); only
 * the FPS budget differs. Ordered quality-first; ties picked in declaration order.
 */
struct StereoDepthProfile {
    StereoDepth::PresetMode preset;
    float maxFps;
};
constexpr std::array<StereoDepthProfile, 8> kStereoDepthProfiles = {{
    {StereoDepth::PresetMode::ACCURACY, 15.f},
    {StereoDepth::PresetMode::HIGH_DETAIL, 15.f},
    {StereoDepth::PresetMode::DENSITY, 30.f},
    {StereoDepth::PresetMode::DEFAULT, 30.f},
    {StereoDepth::PresetMode::FACE, 30.f},
    {StereoDepth::PresetMode::ROBOTICS, 30.f},
    {StereoDepth::PresetMode::FAST_DENSITY, 60.f},
    {StereoDepth::PresetMode::FAST_ACCURACY, 60.f},
}};

bool fitsNeuralProfile(const NeuralDepthProfile& p, uint32_t userW, uint32_t userH) {
    if(userW < p.minUserWidth || userH < p.minUserHeight) {
        return false;
    }
    if(userW > p.maxUserWidth || userH > p.maxUserHeight) {
        return false;
    }
    return true;
}

bool neuralModelSupported(DeviceModelZoo model, const std::vector<DeviceModelZoo>& supportedModels) {
    return supportedModels.empty()
           || std::find(supportedModels.begin(), supportedModels.end(), model) != supportedModels.end();
}

const NeuralDepthProfile* pickNeuralProfile(uint32_t userW,
                                            uint32_t userH,
                                            float requiredFps,
                                            const std::vector<DeviceModelZoo>& supportedModels) {
    for(const auto& p : kNeuralDepthProfiles) {
        if(!fitsNeuralProfile(p, userW, userH)) continue;
        if(!neuralModelSupported(p.model, supportedModels)) continue;
        if(p.maxFps < requiredFps) continue;
        return &p;
    }
    return nullptr;
}

const StereoDepthProfile* pickStereoProfile(float requiredFps) {
    for(const auto& p : kStereoDepthProfiles) {
        if(p.maxFps >= requiredFps) {
            return &p;
        }
    }
    return nullptr;
}

/** RVC4 AUTO selection result: algorithm + (algorithm-specific) profile. */
struct DepthAutoPick {
    Depth::Algorithm algorithm = Depth::Algorithm::STEREO;
    /** Valid only when @c algorithm is @c NEURAL. */
    DeviceModelZoo neuralModel = kDefaultNeuralDepthModel;
    /** Valid only when @c algorithm is @c STEREO. */
    StereoDepth::PresetMode stereoPreset = StereoDepth::PresetMode::DEFAULT;
};

DepthAutoPick pickAutoBackend(uint32_t userW,
                              uint32_t userH,
                              float targetFps,
                              const std::shared_ptr<Device>& device,
                              const std::vector<DeviceModelZoo>& supportedModels) {
    const float requiredFps = targetFps * kSelectionFpsSafetyMargin;
    const bool overStereoMax = userW > kStereoDepthMaxWidth || userH > kStereoDepthMaxHeight;
    const bool neuralAllowed = device->isNeuralDepthSupported();
    const bool gpuAllowed = deviceHasGpuStereoHardware(device);

    // Preference order: NEURAL -> NEURAL_ASSISTED_STEREO -> STEREO -> GPU_STEREO.
    if(neuralAllowed) {
        if(const auto* p = pickNeuralProfile(userW, userH, requiredFps, supportedModels)) {
            return {Depth::Algorithm::NEURAL, p->model, StereoDepth::PresetMode::DEFAULT};
        }
    }
    if(neuralAllowed && !overStereoMax && kNasMaxFps >= requiredFps) {
        return {Depth::Algorithm::NEURAL_ASSISTED_STEREO, kDefaultNasNeuralModel, StereoDepth::PresetMode::DEFAULT};
    }
    if(!overStereoMax) {
        if(const auto* p = pickStereoProfile(requiredFps)) {
            return {Depth::Algorithm::STEREO, kDefaultNeuralDepthModel, p->preset};
        }
    }
    if(gpuAllowed && kGpuStereoMaxFps >= requiredFps) {
        return {Depth::Algorithm::GPU_STEREO, kDefaultNeuralDepthModel, StereoDepth::PresetMode::DEFAULT};
    }

    // FPS budget unmet anywhere: drop FPS constraint, keep preference + resolution rules.
    if(neuralAllowed) {
        for(const auto& p : kNeuralDepthProfiles) {
            if(fitsNeuralProfile(p, userW, userH) && neuralModelSupported(p.model, supportedModels)) {
                return {Depth::Algorithm::NEURAL, p.model, StereoDepth::PresetMode::DEFAULT};
            }
        }
    }
    if(!overStereoMax) {
        return {Depth::Algorithm::STEREO,
                kDefaultNeuralDepthModel,
                kStereoDepthProfiles.back().preset};  // fastest stereo preset
    }
    if(gpuAllowed) {
        return {Depth::Algorithm::GPU_STEREO, kDefaultNeuralDepthModel, StereoDepth::PresetMode::DEFAULT};
    }
    return {Depth::Algorithm::STEREO, kDefaultNeuralDepthModel, kStereoDepthProfiles.back().preset};
}

/** Error text when confidence() is called with GPUStereo as the resolved backend. */
constexpr const char* kGpuStereoConfidenceUnavailable =
    "Depth::confidence() is unavailable when the GPUStereo backend is selected (Algorithm::GPU_STEREO). "
    "GPUStereo does not provide a confidence map; do not link or create an output queue on confidence(). Use depth() "
    "only, or choose NEURAL, STEREO, NEURAL_ASSISTED_STEREO, or TOF.";

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

/** Read consecutive decimal digits at @p i; returns nullopt if none. */
std::optional<int> readDecimalMajorAt(const std::string& s, std::size_t& i) {
    if(i >= s.size() || std::isdigit(static_cast<unsigned char>(s[i])) == 0) {
        return std::nullopt;
    }
    int major = 0;
    while(i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])) != 0) {
        major = major * 10 + (s[i] - '0');
        ++i;
    }
    return major;
}

/**
 * Parse major board revision from EEPROM ``boardRev``.
 * RVC4 uses ``P{n}D*`` (e.g. ``P9D1``, ``P10D0``); older boards use ``R{n}*`` (e.g. ``R9``, ``R2.1``).
 */
std::optional<int> parseBoardRevisionMajor(const std::string& boardRev) {
    if(boardRev.empty()) {
        return std::nullopt;
    }
    std::size_t i = 0;
    while(i < boardRev.size() && std::isspace(static_cast<unsigned char>(boardRev[i])) != 0) {
        ++i;
    }
    if(i >= boardRev.size()) {
        return std::nullopt;
    }
    const char lead = static_cast<char>(std::tolower(static_cast<unsigned char>(boardRev[i])));
    ++i;
    if(lead == 'p') {
        // RVC4 pattern: P{major}D{minor} (e.g. P9D1, P10D0).
        auto major = readDecimalMajorAt(boardRev, i);
        if(major && i < boardRev.size() && std::tolower(static_cast<unsigned char>(boardRev[i])) == 'd') {
            return major;
        }
        return std::nullopt;
    }
    if(lead == 'r') {
        // Legacy pattern: R{major} with optional fractional suffix (e.g. R9, R2.1).
        while(i < boardRev.size() && std::isspace(static_cast<unsigned char>(boardRev[i])) != 0) {
            ++i;
        }
        return readDecimalMajorAt(boardRev, i);
    }
    return std::nullopt;
}

/** Fallback when EEPROM boardRev is missing (e.g. product name contains ``OAK4-D R9``). */
std::optional<int> inferBoardRevisionMajorFromProduct(const std::string& productName) {
    for(std::size_t pos = 0; pos < productName.size(); ++pos) {
        if(pos > 0) {
            const char prev = productName[pos - 1];
            if(std::isalnum(static_cast<unsigned char>(prev)) != 0) {
                continue;
            }
        }
        if(std::tolower(static_cast<unsigned char>(productName[pos])) != 'r') {
            continue;
        }
        std::size_t i = pos + 1;
        if(i >= productName.size() || std::isdigit(static_cast<unsigned char>(productName[i])) == 0) {
            continue;
        }
        int major = 0;
        while(i < productName.size() && std::isdigit(static_cast<unsigned char>(productName[i])) != 0) {
            major = major * 10 + (productName[i] - '0');
            ++i;
        }
        return major;
    }
    return std::nullopt;
}

/** Best-effort board major revision: EEPROM boardRev, then product name, then device name. */
std::optional<int> deviceBoardRevisionMajor(const std::shared_ptr<Device>& device) {
    if(device == nullptr) {
        return std::nullopt;
    }
    // Prefer calibration EEPROM (authoritative on shipped devices).
    try {
        if(auto major = parseBoardRevisionMajor(device->readCalibrationOrDefault().getEepromData().boardRev)) {
            return major;
        }
    } catch(...) {
    }
    // Fall back to human-readable product strings when EEPROM is missing or unreadable.
    try {
        if(auto major = inferBoardRevisionMajorFromProduct(device->getProductName())) {
            return major;
        }
    } catch(...) {
    }
    try {
        return inferBoardRevisionMajorFromProduct(device->getDeviceName());
    } catch(...) {
    }
    return std::nullopt;
}

/** RVC4 with stereo pair and board revision R9+ (GPU block present). */
bool deviceHasGpuStereoHardware(const std::shared_ptr<Device>& device) {
    if(device == nullptr || device->getPlatform() != Platform::RVC4) {
        return false;
    }
    if(device->getStereoPairs().empty()) {
        return false;
    }
    const auto major = deviceBoardRevisionMajor(device);
    if(!major) {
        return false;
    }
    return *major >= kGpuStereoMinBoardRevisionMajor;
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
    // Baseline backends available on all platforms that expose a stereo pair.
    std::vector<Algorithm> supported = {Algorithm::STEREO, Algorithm::NEURAL};

    if(device->getPlatform() == Platform::RVC4) {
        supported.push_back(Algorithm::NEURAL_ASSISTED_STEREO);
        if(deviceHasGpuStereoHardware(device)) {
            supported.push_back(Algorithm::GPU_STEREO);
        }
    }
    if(cameraFeaturesIncludeTof(device->getConnectedCameraFeatures())) {
        supported.push_back(Algorithm::TOF);
    }

    return supported;
}

// --- Algorithm resolution ---

Depth::Algorithm Depth::selectAlgorithm(const std::shared_ptr<Device>& device) const {
    const auto supported = getSupportedAlgorithms(device);

    if(algorithmOverride_ == Algorithm::AUTO) {
        // AUTO: pick the preferred default for the platform when hardware allows it.
        if(device->getPlatform() == Platform::RVC4) {
            return Algorithm::NEURAL;
        }
        if(device->getPlatform() == Platform::RVC2
           && std::find(supported.begin(), supported.end(), Algorithm::TOF) != supported.end()) {
            return Algorithm::TOF;
        }
        return Algorithm::STEREO;
    }

    // Fixed algorithm: return immediately when the device advertises support.
    if(std::find(supported.begin(), supported.end(), algorithmOverride_) != supported.end()) {
        return algorithmOverride_;
    }

    // Unsupported fixed choice: emit a targeted error before the generic fallback.
    switch(algorithmOverride_) {
        case Algorithm::NEURAL_ASSISTED_STEREO:
            DAI_CHECK_V(false, "NeuralAssistedStereo is only supported on RVC4.");
            break;
        case Algorithm::GPU_STEREO:
            DAI_CHECK_V(device->getPlatform() == Platform::RVC4, "GPUStereo is only supported on RVC4.");
            DAI_CHECK_V(deviceHasGpuStereoHardware(device),
                        "GPUStereo requires an RVC4 device with a stereo camera pair and board revision R9 or newer.");
            break;
        case Algorithm::TOF:
            DAI_CHECK_V(false, "Depth Algorithm::TOF requires a connected ToF camera (e.g. OAK-D ToF / OAK-TOF series).");
            break;
        case Algorithm::AUTO:
        case Algorithm::STEREO:
        case Algorithm::NEURAL:
            // Always listed in getSupportedAlgorithms(); unreachable if we get here.
            break;
    }

    DAI_CHECK_V(false, "Depth algorithm is not supported on this device.");
    return Algorithm::STEREO;
}

bool Depth::exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height) {
    return width > kStereoDepthMaxWidth || height > kStereoDepthMaxHeight;
}

DeviceModelZoo Depth::selectNeuralDepthModel(uint32_t userWidth,
                                              uint32_t userHeight,
                                              float targetFps,
                                              const std::vector<DeviceModelZoo>& supportedModels) {
    if(userWidth == 0 || userHeight == 0) {
        userWidth = 640;
        userHeight = 400;
    }
    if(targetFps <= 0.f) {
        targetFps = 30.f;
    }
    const float requiredFps = targetFps * kSelectionFpsSafetyMargin;

    if(const auto* p = pickNeuralProfile(userWidth, userHeight, requiredFps, supportedModels)) {
        return p->model;
    }

    // FPS budget unmet: keep resolution constraint, return the fastest model that still fits.
    const NeuralDepthProfile* best = nullptr;
    for(const auto& p : kNeuralDepthProfiles) {
        if(!fitsNeuralProfile(p, userWidth, userHeight)) continue;
        if(!neuralModelSupported(p.model, supportedModels)) continue;
        if(best == nullptr || p.maxFps > best->maxFps) best = &p;
    }
    if(best) return best->model;

    // Resolution outside every band: pick any supported model as last resort.
    for(const auto& p : kNeuralDepthProfiles) {
        if(neuralModelSupported(p.model, supportedModels)) return p.model;
    }
    return kDefaultNeuralDepthModel;
}

StereoDepth::PresetMode Depth::selectStereoDepthPreset(float targetFps) {
    if(targetFps <= 0.f) {
        targetFps = 30.f;
    }
    if(const auto* p = pickStereoProfile(targetFps * kSelectionFpsSafetyMargin)) {
        return p->preset;
    }
    return kStereoDepthProfiles.back().preset;
}

void Depth::resolveWiring(const std::shared_ptr<Device>& device, Pipeline& pipeline) {
    resolvedAlgorithm_ = selectAlgorithm(device);
    resolvedNeuralModel_ = neuralModelOverride_.value_or(kDefaultNeuralDepthModel);
    resolvedStereoPreset_ = StereoDepth::PresetMode::DEFAULT;

    if(device->getPlatform() != Platform::RVC4) {
        return;
    }

    const auto pair = requireFirstStereoPair(device);
    const auto [left, right] = findCamerasForPair(pipeline, pair);
    std::optional<std::pair<uint32_t, uint32_t>> stereoSize = stereoSizeOverride_;
    if(!stereoSize) {
        stereoSize = stereoSizeFromExistingCameras(left, right);
    }

    float targetFps = stereoOutputFps_.value_or(0.f);
    if(targetFps <= 0.f && left && right) {
        targetFps = std::max(left->getMaxRequestedFps(), right->getMaxRequestedFps());
    }
    if(targetFps <= 0.f) {
        targetFps = 30.f;
    }

    const uint32_t userW = stereoSize ? stereoSize->first : 640;
    const uint32_t userH = stereoSize ? stereoSize->second : 400;
    const auto supportedModels = device->getSupportedDeviceModels();

    if(algorithmOverride_ == Algorithm::AUTO) {
        const auto pick = pickAutoBackend(userW, userH, targetFps, device, supportedModels);
        resolvedAlgorithm_ = pick.algorithm;
        // NAS always uses NEURAL_DEPTH_NANO and sets its own stereo preset internally — no profile to pick.
        resolvedNeuralModel_ = neuralModelOverride_.value_or(pick.neuralModel);
        resolvedStereoPreset_ = pick.stereoPreset;
        if(resolvedAlgorithm_ == Algorithm::GPU_STEREO) {
            DAI_CHECK_V(deviceHasGpuStereoHardware(device),
                        "Depth: GPUStereo is required for this resolution/FPS but board R9+ hardware is unavailable.");
        }
        return;
    }

    // Explicit algorithm: validate resolution and pick the algorithm-specific profile.
    if(resolvedAlgorithm_ == Algorithm::STEREO || resolvedAlgorithm_ == Algorithm::NEURAL_ASSISTED_STEREO) {
        DAI_CHECK_V(!exceedsStereoDepthMaxResolution(userW, userH),
                    "Depth: resolution exceeds StereoDepth maximum 1280x1280; use GPUStereo or lower resolution.");
    }
    if(resolvedAlgorithm_ == Algorithm::STEREO) {
        resolvedStereoPreset_ = selectStereoDepthPreset(targetFps);
    }
    if(resolvedAlgorithm_ == Algorithm::NEURAL && !neuralModelOverride_) {
        resolvedNeuralModel_ = selectNeuralDepthModel(userW, userH, targetFps, supportedModels);
    }
    if(resolvedAlgorithm_ == Algorithm::NEURAL_ASSISTED_STEREO) {
        resolvedNeuralModel_ = kDefaultNasNeuralModel;  // NAS always uses NEURAL_DEPTH_NANO
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
    const Algorithm active = resolvedAlgorithm_;

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
            nasBackend_->build(*leftOut, *rightOut, resolvedNeuralModel_, kDefaultNasRectify);
            break;
        }
        case Algorithm::GPU_STEREO: {
            gpuStereoBackend_ = std::make_unique<Subnode<GPUStereo>>(*this, "gpuStereo");
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, std::nullopt);
            (*gpuStereoBackend_)->setRectification(true).build(*leftOut, *rightOut);
            break;
        }
        case Algorithm::NEURAL: {
            const auto is = NeuralDepth::getInputSize(resolvedNeuralModel_);
            const std::pair<uint32_t, uint32_t> monoSize{static_cast<uint32_t>(is.first), static_cast<uint32_t>(is.second)};
            neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "neuralDepth");
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, monoSize);
            (*neuralBackend_)->build(*leftOut, *rightOut, resolvedNeuralModel_);
            break;
        }
        case Algorithm::STEREO: {
            stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "stereoDepth");
            auto [leftOut, rightOut] = stereoCameraOutputs(pipeline, device, std::nullopt);
            (*stereoBackend_)->build(*leftOut, *rightOut, resolvedStereoPreset_);
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

/** Wire composite outputs to the active backend; GPUStereo leaves confidenceOut_ null. */
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
            confidenceOut_ = nullptr;
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

bool Depth::hasConfidence() const {
    if(!graphBuilt_) {
        // buildInternal is non-const; const API still needs wiring before querying the backend.
        const_cast<Depth*>(this)->buildInternal();
    }
    return confidenceOut_ != nullptr;
}

Node::Output& Depth::confidence() {
    if(!graphBuilt_) {
        buildInternal();
    }
    if(confidenceOut_ == nullptr) {
        // GPUStereo is the only backend without a confidence-like stream.
        throw std::runtime_error(kGpuStereoConfidenceUnavailable);
    }
    return *confidenceOut_;
}

}  // namespace node
}  // namespace dai
