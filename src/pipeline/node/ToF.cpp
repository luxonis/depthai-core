#include "depthai/pipeline/node/ToF.hpp"

#include <algorithm>
#include <cmath>

#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/ToFSensorMode.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "spdlog/fmt/fmt.h"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

namespace {

bool configHasIppFieldsSet(const ToFConfig& config) {
    return config.enableBilateralFilter.has_value() || config.bilateralStdFactor.has_value() || config.bilateralFilterKernelSize.has_value()
           || config.enableTemporalNoiseReduction.has_value() || config.tnrMaxGain.has_value() || config.tnrStdFactor.has_value()
           || config.enableFlyingPixelCorrection.has_value() || config.fpDepthThreshold.has_value() || config.fpMinDepthOccurrence.has_value()
           || config.enableRadialToPerp.has_value();
}

bool configHasDecoderFieldsSet(const ToFConfig& config) {
    return config.median != filters::params::MedianFilter::MEDIAN_OFF || config.enablePhaseShuffleTemporalFilter != true || config.enableBurstMode != false
           || config.enableDistortionCorrection != true || config.phaseUnwrappingLevel != 4 || config.enableFPPNCorrection.has_value()
           || config.enableOpticalCorrection.has_value() || config.enableTemperatureCorrection.has_value() || config.enableWiggleCorrection.has_value()
           || config.enablePhaseUnwrapping.has_value();
}

CameraBoardSocket resolveToFBoardSocket(Device& device, CameraBoardSocket boardSocket) {
    auto cameraFeatures = device.getConnectedCameraFeatures();
    bool found = false;

    if(boardSocket == CameraBoardSocket::AUTO) {
        auto socketHasToF = [&](CameraBoardSocket socket) {
            for(const auto& cf : cameraFeatures) {
                if(cf.socket != socket) {
                    continue;
                }
                return std::find(cf.supportedTypes.begin(), cf.supportedTypes.end(), CameraSensorType::TOF) != cf.supportedTypes.end();
            }
            return false;
        };

        // Prefer the conventional sockets in order, then fall back to the first ToF-capable
        // socket on any other wiring (contract: AUTO picks the first socket with CameraSensorType::TOF).
        auto preferredSockets = {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_D};
        for(auto socket : preferredSockets) {
            if(socketHasToF(socket)) {
                boardSocket = socket;
                found = true;
                break;
            }
        }
        if(!found) {
            for(const auto& cf : cameraFeatures) {
                if(std::find(cf.supportedTypes.begin(), cf.supportedTypes.end(), CameraSensorType::TOF) != cf.supportedTypes.end()) {
                    boardSocket = cf.socket;
                    found = true;
                    break;
                }
            }
        }
    } else {
        for(const auto& cf : cameraFeatures) {
            if(cf.socket == boardSocket) {
                if(std::find(cf.supportedTypes.begin(), cf.supportedTypes.end(), CameraSensorType::TOF) == cf.supportedTypes.end()) {
                    throw std::runtime_error(fmt::format("Camera on socket {} doesn't have a ToF sensor.", static_cast<int>(boardSocket)));
                }
                found = true;
                break;
            }
        }
    }

    if(!found) {
        throw std::runtime_error("Camera socket not found on the connected device");
    }

    return boardSocket;
}

}  // namespace

ToFBase::ToFBase(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ToFBase, ToFProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

ToFBase::Properties& ToFBase::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

std::shared_ptr<ToFBase> ToFBase::finalizeBuild(CameraBoardSocket boardSocket, std::optional<float> fps) {
    if(isBuilt) {
        throw std::runtime_error("ToF node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    boardSocket = resolveToFBoardSocket(*device, boardSocket);

    for(const auto& cf : device->getConnectedCameraFeatures()) {
        if(cf.socket == boardSocket) {
            maxWidth = cf.width;
            maxHeight = cf.height;
            break;
        }
    }

    properties.boardSocket = boardSocket;
    properties.fps = fps.value_or(ToFProperties::AUTO);

    isBuilt = true;
    return std::static_pointer_cast<ToFBase>(shared_from_this());
}

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, ImageFiltersPresetMode presetMode, std::optional<float> fps) {
    initialConfig->setProfilePreset(presetMode);
    return finalizeBuild(boardSocket, fps);
}

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, std::optional<float> fps) {
    return finalizeBuild(boardSocket, fps);
}

ToF::ToF(const std::shared_ptr<Device>& device)
    : DeviceNodeGroup(device),
      initialConfig{tofBase->initialConfig},
      rawDepth{tofBase->depth},
      depth((device && device->getPlatform() == Platform::RVC4) ? static_cast<Output&>(tofBase->depth)
                                                                : static_cast<Output&>(imageFilters->output)),
      amplitude{tofBase->amplitude},
      intensity{tofBase->intensity},
      confidence((device && device->getPlatform() == Platform::RVC4) ? static_cast<Output&>(tofBase->confidence)
                                                                    : static_cast<Output&>(tofBase->amplitude)),
      phase{tofBase->phase},
      rawInput{tofBase->rawInput},
      raw{tofBase->raw},
      inputConfig{tofBase->inputConfig},
      tofBaseInputConfig{tofBase->inputConfig},
      imageFiltersInputConfig{imageFilters->inputConfig},
      tofBaseNode{*tofBase},
      imageFiltersNode{*imageFilters},
      isRvc4Platform(device && device->getPlatform() == Platform::RVC4) {}

ToF::~ToF() = default;

void ToF::logWarnOnce(bool& warned, const char* message) const {
    if(!warned) {
        warned = true;
        Logging::getInstance().logger.warn(message);
    }
}

void ToF::buildInternal() {
    tofBase->buildInternal();
    if(!isRvc4Platform) {
        imageFilters->buildInternal();
        tofBase->depth.link(imageFilters->input);
    }
}

void ToF::validateBuildFps(const std::optional<float>& fps) {
    if(!fps.has_value()) {
        return;
    }

    const float value = fps.value();
    if(value <= 0.f) {
        throw std::runtime_error("ToF fps must be greater than 0");
    }

    if(std::abs(value - std::round(value)) > 1e-3f) {
        throw std::runtime_error("ToF fps must be an integer value (matches ToFProperties/Camera fps field)");
    }

    if(isRvc4Platform && (value < 10.f || value > 30.f)) {
        logWarnOnce(warnedFpsRangeRvc4, "RVC4 ToF fps outside VD55H1 recommended range 10-30");
    }
}

std::shared_ptr<ToF> ToF::build(const ToFBuildOptions& options) {
    if(isGroupBuilt) {
        throw std::runtime_error("ToF node is already built");
    }
    if(!device) {
        throw std::runtime_error("Connect device before ToF.build()");
    }

    validateBuildFps(options.fps);

    if(isRvc4Platform) {
        buildOptions = options;

        tofBase->build(options.boardSocket, options.fps);
        buildOptions.boardSocket = tofBase->getBoardSocket();
        if(options.preset.has_value()) {
            tofBase->initialConfig->setToFPreset(*options.preset);
        }
    } else {
        buildOptions = options;
        // Keep the build surface unified across platforms: map ToFPreset to the equivalent
        // host ImageFiltersPresetMode on RVC2 instead of silently dropping it to MID_RANGE.
        auto presetMode = options.preset.has_value() ? toFPresetToImageFiltersPreset(*options.preset) : ImageFiltersPresetMode::TOF_MID_RANGE;
        tofBase->build(options.boardSocket, presetMode, options.fps);
        imageFilters->build(presetMode);
        buildOptions.boardSocket = tofBase->getBoardSocket();
    }

    isGroupBuilt = true;
    warnIfMisconfiguredInitialConfig();
    return std::static_pointer_cast<ToF>(shared_from_this());
}

std::shared_ptr<ToF> ToF::build(CameraBoardSocket boardSocket, ImageFiltersPresetMode presetMode, std::optional<float> fps) {
    if(isGroupBuilt) {
        throw std::runtime_error("ToF node is already built");
    }

    if(isRvc4Platform) {
        ToFBuildOptions options;
        options.boardSocket = boardSocket;
        options.fps = fps;
        options.preset = imageFiltersPresetToToFPreset(presetMode);
        return build(options);
    }

    if(!device) {
        throw std::runtime_error("Connect device before ToF.build()");
    }

    validateBuildFps(fps);

    tofBase->build(boardSocket, presetMode, fps);
    imageFilters->build(presetMode);

    buildOptions.boardSocket = tofBase->getBoardSocket();
    buildOptions.fps = fps;
    isGroupBuilt = true;
    warnIfMisconfiguredInitialConfig();
    return std::static_pointer_cast<ToF>(shared_from_this());
}

void ToF::buildStage1() {
    // No cross-node validation needed at this stage; retained for lifecycle symmetry.
}

void ToF::buildStage2() {
    if(!isRvc4Platform || !isGroupBuilt) {
        return;
    }

    if(rawInput.isConnected()) {
        Logging::getInstance().logger.info("ToF: Using external rawInput; auto-camera skipped");
    } else {
        auto pipeline = getParentPipeline();
        const auto mode = ToFSensorMode::F3_FULL;  // BETA: VD55H1 capture mode fixed
        const auto rawResolution = getToFSensorModeRawResolution(mode);
        autoCamera = pipeline.create<Camera>();
        autoCamera->setSensorType(CameraSensorType::TOF);
        autoCamera->build(buildOptions.boardSocket, rawResolution, buildOptions.fps);
        autoCamera->raw.link(rawInput);
    }

    if(!rawInput.isConnected()) {
        throw std::runtime_error("ToF on RVC4 requires rawInput connection (auto-camera failed or disabled)");
    }
}

std::shared_ptr<Camera> ToF::getCamera() const {
    return autoCamera;
}

CameraBoardSocket ToF::getBoardSocket() const {
    return tofBase->getBoardSocket();
}

std::pair<uint32_t, uint32_t> ToF::getOutputResolution() const {
    if(!isGroupBuilt) {
        throw std::runtime_error("ToF node must be built before calling getOutputResolution()");
    }
    if(!isRvc4Platform) {
        throw std::runtime_error("getOutputResolution() is RVC4-only (VD55H1 ToFSensorMode); on RVC2 use depth frame dimensions at runtime");
    }
    const auto mode = ToFSensorMode::F3_FULL;  // BETA: VD55H1 capture mode fixed
    return getToFSensorModeOutputResolution(mode);
}

std::pair<uint32_t, uint32_t> ToF::getRawResolution() const {
    if(!isGroupBuilt) {
        throw std::runtime_error("ToF node must be built before calling getRawResolution()");
    }
    if(!isRvc4Platform) {
        throw std::runtime_error("getRawResolution() is RVC4-only (VD55H1 raw superframe)");
    }
    const auto mode = ToFSensorMode::F3_FULL;  // BETA: VD55H1 capture mode fixed
    return getToFSensorModeRawResolution(mode);
}

std::pair<uint32_t, uint32_t> ToF::getSensorResolution() const {
    logWarnOnce(warnedSensorResolutionDeprecated, "getSensorResolution() is deprecated; use getOutputResolution() for IPP output size or getRawResolution() for Camera raw superframe size");
    return getOutputResolution();
}

void ToF::warnIfMisconfiguredInitialConfig() const {
    const auto& config = *tofBase->initialConfig;
    if(isRvc4Platform) {
        if(configHasDecoderFieldsSet(config)) {
            logWarnOnce(warnedDecoderFieldsRvc4, "RVC2 decoder fields in ToFConfig are ignored on RVC4");
        }
    } else if(configHasIppFieldsSet(config)) {
        logWarnOnce(warnedIppFieldsRvc2, "IPP fields in ToFConfig are ignored on RVC2");
    }
}

CameraBoardSocket ToFBase::getBoardSocket() const {
    if(!isBuilt) {
        throw std::runtime_error("ToF node must be built before calling getBoardSocket()");
    }
    return properties.boardSocket;
}

}  // namespace node
}  // namespace dai
