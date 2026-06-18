#include "depthai/pipeline/node/ToF.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ToFBase::ToFBase(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ToFBase, ToFProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

ToFBase::Properties& ToFBase::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, ToFConfig::Profile profile, std::optional<float> fps) {
    if(isBuilt) {
        throw std::runtime_error("ToF node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    auto cameraFeatures = device->getConnectedCameraFeatures();
    // First handle the case if the boardSocket is set to AUTO
    if(boardSocket == CameraBoardSocket::AUTO) {
        for(const auto& cf : cameraFeatures) {
            for(const auto& sensorType : cf.supportedTypes) {
                if(sensorType == CameraSensorType::TOF) {
                    boardSocket = cf.socket;
                    found = true;
                    break;
                }
            }
        }
    }

    if(!found) {
        throw std::runtime_error("Camera socket not found on the connected device");
    }

    // Set profile preset for ToFConfig
    initialConfig->profile = profile;

    properties.boardSocket = boardSocket;
    properties.fps = fps.value_or(ToFProperties::AUTO);

    isBuilt = true;
    return std::static_pointer_cast<ToFBase>(shared_from_this());
}

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, ImageFiltersPresetMode presetMode, std::optional<float> fps) {
    ToFConfig::Profile profile;
    switch(presetMode) {
        case TOF_LOW_RANGE:
            profile = ToFConfig::LOW_RANGE;
            break;

        case TOF_MID_RANGE:
            profile = ToFConfig::MID_RANGE;
            break;

        case TOF_HIGH_RANGE:
            profile = ToFConfig::HIGHT_RANGE;
            break;
    }
    if(platform == Platform::RVC2) {
        initialConfig->setProfilePresetprofile(presetMode);
    }
    build(boardSocket, profile, fps);

    return std::static_pointer_cast<ToFBase>(shared_from_this());
}

ToF::~ToF() = default;

// Get current board socket
CameraBoardSocket ToFBase::getBoardSocket() const {
    if(!isBuilt) {
        throw std::runtime_error("ToF node must be built before calling getBoardSocket()");
    }
    return properties.boardSocket;
}

}  // namespace node
}  // namespace dai
