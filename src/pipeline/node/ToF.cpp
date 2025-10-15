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

std::shared_ptr<ToFBase> ToFBase::build(CameraBoardSocket boardSocket, ImageFiltersPresetMode presetMode, std::optional<float> fps) {
    if(isBuilt) {
        throw std::runtime_error("ToF node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    auto cameraFeatures = device->getConnectedCameraFeatures();
    // First handle the case if the boardSocket is set to AUTO
    if(boardSocket == CameraBoardSocket::AUTO) {
        auto defaultSockets = {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_D};
        for(auto socket : defaultSockets) {
            bool found = false;
            for(const auto& cf : cameraFeatures) {
                if(cf.socket == socket) {
                    for(const auto& sensorType : cf.supportedTypes) {
                        if(sensorType == CameraSensorType::TOF) {
                            found = true;
                            break;
                        }
                    }
                }
            }
            if(found) {
                boardSocket = socket;
                break;
            }
        }
    }

    // Check if the board socket is valid
    bool found = false;
    for(const auto& cf : cameraFeatures) {
        if(cf.socket == boardSocket) {
            found = true;
            maxWidth = cf.width;
            maxHeight = cf.height;
            break;
        }
    }
    if(!found) {
        throw std::runtime_error("Camera socket not found on the connected device");
    }

    // Set profile preset for ToFConfig
    initialConfig->setProfilePreset(presetMode);

    properties.boardSocket = boardSocket;
    properties.fps = fps.value_or(ToFProperties::AUTO);

    isBuilt = true;
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
