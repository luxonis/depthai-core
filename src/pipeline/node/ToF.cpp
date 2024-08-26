#include "depthai/pipeline/node/ToF.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ToF::ToF(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, ToF, ToFProperties>(std::move(props)) {}

ToF::Properties& ToF::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

std::shared_ptr<ToF> ToF::build(CameraBoardSocket boardSocket) {
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

    properties.boardSocket = boardSocket;
    isBuilt = true;
    return std::static_pointer_cast<ToF>(shared_from_this());
}

// Get current board socket
CameraBoardSocket ToF::getBoardSocket() const {
    if(!isBuilt) {
        throw std::runtime_error("ToF node must be built before calling getBoardSocket()");
    }
    return properties.boardSocket;
}

}  // namespace node
}  // namespace dai
