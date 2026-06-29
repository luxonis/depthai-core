#include "depthai/pipeline/node/ToF.hpp"

#include <utility>

#include "depthai/common/CameraSensorType.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "spdlog/fmt/fmt.h"

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

bool usesAutoCamera(const std::shared_ptr<Device>& device) {
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
        if(!intensity.getConnections().empty()) {
            if(logger) logger->warn("Intensity is not supported on this platform and will stream aplitude instead.");
        }
        if(!rawDepth.getConnections().empty()) {
            if(logger) logger->warn("RawDepth is not supported on this platform and will not stream the data.");
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

std::shared_ptr<ToF> ToF::build(dai::CameraBoardSocket boardSocket, dai::ImageFiltersPresetMode presetMode, std::optional<float> fps) {
    return build(boardSocket, presetModeToProfile(presetMode), fps);
}

std::shared_ptr<ToF> ToF::build(dai::CameraBoardSocket boardSocket, dai::ToFConfig::Profile profile, std::optional<float> fps) {
    tofBase->build(boardSocket, profile, fps);

    const auto presetMode = profileToPresetMode(profile);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(usesImageFilters(getDevice()) && imageFilters) {
        (*imageFilters)->build(presetMode);
    }
#endif

    if(autoCamera) {
        (*autoCamera)->setSensorType(CameraSensorType::TOF);
        (*autoCamera)->build(tofBase->getBoardSocket(),
                             std::pair<uint32_t, uint32_t>{1344, 7244},
                             tofBase->properties.fps > 0 ? std::optional<float>(tofBase->properties.fps) : std::nullopt);

        (*autoCamera)->raw.link(tofBase->rawInput);
    }
    return std::static_pointer_cast<ToF>(shared_from_this());
}

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, ToFConfig::Profile profile, std::optional<float> fps) {
    if(isBuilt) {
        throw std::runtime_error("ToF node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    auto cameraFeatures = device->getConnectedCameraFeatures();
    bool found = boardSocket != CameraBoardSocket::AUTO;
    if(boardSocket == CameraBoardSocket::AUTO) {
        for(const auto& cf : cameraFeatures) {
            for(const auto& sensorType : cf.supportedTypes) {
                if(sensorType == CameraSensorType::TOF) {
                    boardSocket = cf.socket;
                    found = true;
                    break;
                }
            }
            if(found) {
                break;
            }
        }
    }

    if(!found) {
        throw std::runtime_error("Camera socket not found on the connected device");
    }

    initialConfig->setProfilePreset(profile);

    properties.boardSocket = boardSocket;
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

}  // namespace node
}  // namespace dai
