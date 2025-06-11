#include "depthai/pipeline/node/Thermal.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

Thermal::Thermal(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, Thermal, ThermalProperties>(std::move(props)) {}

Thermal::Properties& Thermal::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void Thermal::setFps(float fps) {
    properties.fps = fps;
}

std::shared_ptr<Thermal> Thermal::build(CameraBoardSocket boardSocket, float fps) {
    if(isBuilt) {
        throw std::runtime_error("Thermal node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    auto cameraFeatures = device->getConnectedCameraFeatures();
    // First handle the case if the boardSocket is set to AUTO
    bool found = false;
    if(boardSocket == CameraBoardSocket::AUTO) {
        auto defaultSockets = {CameraBoardSocket::CAM_E};
        for(auto socket : defaultSockets) {
            for(const auto& cf : cameraFeatures) {
                if(cf.socket == socket) {
                    for(const auto& sensorType : cf.supportedTypes) {
                        if(sensorType == CameraSensorType::THERMAL) {
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
    } else {  // Try to find the manually specified socket and verify it has a thermal sensor
        for(const auto& cf : cameraFeatures) {
            if(cf.socket == boardSocket) {
                if(std::find(cf.supportedTypes.begin(), cf.supportedTypes.end(), CameraSensorType::THERMAL) == cf.supportedTypes.end()) {
                    throw std::runtime_error(fmt::format("Camera on socket {} doesn't have a thermal sensor.", (int)boardSocket));
                }
                found = true;
                break;
            }
        }
    }

    if(!found) {
        throw std::runtime_error("Camera socket not found on the connected device");
    }

    properties.boardSocket = boardSocket;
    properties.fps = fps;
    isBuilt = true;
    return std::static_pointer_cast<Thermal>(shared_from_this());
}

// Get current board socket
CameraBoardSocket Thermal::getBoardSocket() const {
    if(!isBuilt) {
        throw std::runtime_error("Thermal node must be built before calling getBoardSocket()");
    }
    return properties.boardSocket;
}

}  // namespace node
}  // namespace dai
