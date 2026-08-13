#include "depthai/pipeline/node/ToF.hpp"

#include <utility>
#include <vector>

#include "depthai/common/CameraSensorType.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "spdlog/fmt/fmt.h"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

bool usesImageFilters(const std::shared_ptr<Device>& device) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    return device && device->getPlatform() == Platform::RVC2;
#else
    (void)device;
    return false;
#endif
}

bool usesAutoCamera(const std::shared_ptr<const Device>& device) {
    return device && device->getPlatform() == Platform::RVC4;
}

ImageFiltersPresetMode profileToPresetMode(ToFConfig::Profile profile) {
    switch(profile) {
        case ToFConfig::Profile::LOW_RANGE:
            return ImageFiltersPresetMode::TOF_LOW_RANGE;
        case ToFConfig::Profile::MID_RANGE:
            return ImageFiltersPresetMode::TOF_MID_RANGE;
        case ToFConfig::Profile::HIGH_RANGE:
            return ImageFiltersPresetMode::TOF_HIGH_RANGE;
    }

    throw std::runtime_error("Unknown ToF profile");
}

ToFConfig::Profile presetModeToProfile(ImageFiltersPresetMode presetMode) {
    switch(presetMode) {
        case ImageFiltersPresetMode::TOF_LOW_RANGE:
            return ToFConfig::Profile::LOW_RANGE;
        case ImageFiltersPresetMode::TOF_MID_RANGE:
            return ToFConfig::Profile::MID_RANGE;
        case ImageFiltersPresetMode::TOF_HIGH_RANGE:
            return ToFConfig::Profile::HIGH_RANGE;
    }

    throw std::runtime_error("Unknown ImageFilters preset mode");
}

std::vector<CameraBoardSocket> getAllToFSockets(const std::vector<CameraFeatures>& cameraFeatures) {
    std::vector<CameraBoardSocket> tofSockets;
    for(const auto& cf : cameraFeatures) {
        for(const auto& sensorType : cf.supportedTypes) {
            if(sensorType == CameraSensorType::TOF) {
                tofSockets.push_back(cf.socket);
                break;
            }
        }
    }
    return tofSockets;
}

bool isSocketAlreadyConnected(const Pipeline& pipeline, CameraBoardSocket boardSocket) {
    for(const auto& node : pipeline.getAllNodes()) {
        auto camera = std::dynamic_pointer_cast<Camera>(node);
        if(camera) {
            try {
                if(camera->getBoardSocket() == boardSocket) {
                    return true;
                }
            } catch(const std::exception&) {
            }
        }

        auto tofBase = std::dynamic_pointer_cast<ToFBase>(node);
        if(tofBase) {
            try {
                if(tofBase->getBoardSocket() == boardSocket) {
                    return true;
                }
            } catch(const std::exception&) {
            }
        }

        auto tof = std::dynamic_pointer_cast<ToF>(node);
        if(tof) {
            try {
                if(tof->tofBaseNode.getBoardSocket() == boardSocket) {
                    return true;
                }
            } catch(const std::exception&) {
            }
        }
    }
    return false;
}

}  // namespace

void ToF::postBuildStage() {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    auto& logger = pimpl->logger;
    if(device->getPlatform() == Platform::RVC2) {
        if(!confidence.getConnections().empty()) {
            if(logger) logger->warn("Confidence is not supported on this platform and will stream aplitude instead.");
        }
    }
    if(device->getPlatform() == Platform::RVC4) {
        const bool hasRawDepthConnections = !rawDepth.getConnections().empty() || !rawDepth.getQueueConnections().empty();
        if(hasRawDepthConnections) {
            throw std::runtime_error("ToF on RVC4 does not support rawDepth output.");
        }
        const bool hasPhaseConnections = !phase.getConnections().empty() || !phase.getQueueConnections().empty();
        if(hasPhaseConnections) {
            throw std::runtime_error("ToF on RVC4 does not support Phase output.");
        }
    }
#endif
}

ToF::ToF(const std::shared_ptr<Device>& device)
    : DeviceNodeGroup(device)
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
      ,
      imageFilters{usesImageFilters(device) ? std::make_unique<Subnode<ImageFilters>>(*this, "imageFilter") : nullptr}
#endif
      ,
      autoCamera{usesAutoCamera(device) ? std::make_unique<Subnode<Camera>>(*this, "autoCamera") : nullptr}
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
      ,
      rawDepth{device && device->getPlatform() == Platform::RVC2 ? tofBase->depth : rawDepthPlaceholder}
    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
      ,
      depth{usesImageFilters(device) ? (*imageFilters)->output : tofBase->depth}
    #else
      ,
      depth{tofBase->depth}
    #endif
#endif
{
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(imageFilters) {
        imageFiltersNode = &**imageFilters;
    #ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
        imageFiltersInputConfig = &(*imageFilters)->inputConfig;
    #endif
    }
#endif
}

ToFBase::ToFBase(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ToFBase, ToFProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

ToFBase::Properties& ToFBase::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void ToF::buildInternal() {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(usesImageFilters(getDevice())) {
        tofBase->depth.link((*imageFilters)->input);
    }
#endif
}

void ToF::buildAutoCamera() {
    if(!autoCamera || !usesAutoCamera(getDevice())) {
        return;
    }

    auto& camera = **autoCamera;
    camera.setSensorType(CameraSensorType::TOF);
    camera.build(tofBase->getBoardSocket(),
                 std::pair<uint32_t, uint32_t>{1344, 7244},
                 tofBase->properties.fps > 0 ? std::optional<float>(tofBase->properties.fps) : std::nullopt);
    camera.raw.link(tofBase->rawInput);
}

std::shared_ptr<ToF> ToF::build(dai::CameraBoardSocket boardSocket, dai::ImageFiltersPresetMode presetMode, std::optional<float> fps) {
    return build(boardSocket, presetModeToProfile(presetMode), fps);
}

std::shared_ptr<ToF> ToF::build(dai::CameraBoardSocket boardSocket, dai::ToFConfig::Profile profile, std::optional<float> fps) {
    tofBase->build(boardSocket, profile, fps);
    buildAutoCamera();

    const auto presetMode = profileToPresetMode(profile);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(usesImageFilters(getDevice()) && imageFilters) {
        (*imageFilters)->build(presetMode);
    }
#endif
    return std::static_pointer_cast<ToF>(shared_from_this());
}

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, ToFConfig::Profile profile, std::optional<float> fps) {
    if(isBuilt) {
        throw std::runtime_error("ToF node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    const auto cameraFeatures = device->getConnectedCameraFeatures();
    const auto tofSockets = getAllToFSockets(cameraFeatures);
    DAI_CHECK_V(!tofSockets.empty(), "No ToF sensors found on the connected device.");
    const auto pipeline = getParentPipeline();

    if(boardSocket == CameraBoardSocket::AUTO) {
        bool foundAvailableSocket = false;
        for(const auto& tofSocket : tofSockets) {
            if(!isSocketAlreadyConnected(pipeline, tofSocket)) {
                properties.boardSocket = tofSocket;
                foundAvailableSocket = true;
                break;
            }
        }
        DAI_CHECK_V(foundAvailableSocket, "All ToF-capable sensors are already used by other Camera or ToF nodes in this pipeline.");
    } else {
        DAI_CHECK_V(!isSocketAlreadyConnected(pipeline, boardSocket),
                    "Camera socket {} is already used by another Camera node in this pipeline.",
                    static_cast<int>(boardSocket));
        const bool isToFSocket = std::find(tofSockets.begin(), tofSockets.end(), boardSocket) != tofSockets.end();
        DAI_CHECK_V(isToFSocket, "Camera socket {} is not a ToF-capable socket on the connected device.", static_cast<int>(boardSocket));
        properties.boardSocket = boardSocket;
    }

    initialConfig->setProfilePreset(profile);

    properties.fps = fps.value_or(ToFProperties::AUTO);

    isBuilt = true;
    return std::static_pointer_cast<ToFBase>(shared_from_this());
}

ToF::~ToF() = default;

CameraBoardSocket ToFBase::getBoardSocket() const {
    if(!isBuilt) {
        throw std::runtime_error("ToF node must be built before calling getBoardSocket()");
    }
    return properties.boardSocket;
}

std::shared_ptr<ToFBase> ToFBase::setOutputUndistortion(bool enable) {
    properties.enableUndistortion = enable;
    return std::static_pointer_cast<ToFBase>(shared_from_this());
}

std::shared_ptr<ToF> ToF::setOutputUndistortion(bool enable) {
    tofBase->setOutputUndistortion(enable);
    return std::static_pointer_cast<ToF>(shared_from_this());
}

}  // namespace node
}  // namespace dai
