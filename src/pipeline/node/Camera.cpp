#include "depthai/pipeline/node/Camera.hpp"
// std
#include <fstream>
#include <memory>
#include <stdexcept>

// libraries
#include <spimpl.h>

// depthai internal
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

class Camera::Impl {
   public:
    struct OutputRequest {
        int32_t id{};
        ImgFrameCapability capability;
        bool onHost{false};
    };

    int32_t nextOutputRequestId = 0;

    std::vector<OutputRequest> outputRequests;

    Node::Output* requestOutput(Camera& parent, const Capability& genericCapability, bool onHost) {
        if(!parent.isBuilt) {
            throw std::runtime_error("Camera node must be built before requesting outputs from it");
        }

        if(const auto* capability = ImgFrameCapability::get(genericCapability)) {
            const auto requestId = nextOutputRequestId;
            outputRequests.push_back({requestId, *capability, onHost});
            ++nextOutputRequestId;
            parent.properties.outputRequests.push_back(*capability);
            return &parent.dynamicOutputs[std::to_string(requestId)];
        }
        return nullptr;
    }

    void buildStage1(Camera& parent) {
        for(const auto& outputRequest : outputRequests) {
            DAI_CHECK(!parent.dynamicOutputs[std::to_string(outputRequest.id)].getQueueConnections().empty()
                          || !parent.dynamicOutputs[std::to_string(outputRequest.id)].getConnections().empty(),
                      "Always call output->createOutputQueue() or output->link(*) after calling dai::node::Camera::requestOutput()");
        }
    }
};

Camera::Camera() : pimpl(spimpl::make_impl<Impl>()) {}

Camera::Camera(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(std::move(props)), initialControl(properties.initialControl), pimpl(spimpl::make_impl<Impl>()) {}

Camera::Camera(std::shared_ptr<Device>& defaultDevice)
    : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(defaultDevice), pimpl(spimpl::make_impl<Impl>()) {}

std::shared_ptr<Camera> Camera::build(CameraBoardSocket boardSocket,
                                      std::optional<std::pair<uint32_t, uint32_t>> sensorResolution,
                                      std::optional<float> sensorFps) {
    if(isBuilt) {
        throw std::runtime_error("Camera node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    auto cameraFeaturesVector = device->getConnectedCameraFeatures();
    // First handle the case if the boardSocket is set to AUTO
    if(boardSocket == CameraBoardSocket::AUTO) {
        auto defaultSockets = {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C};
        for(auto socket : defaultSockets) {
            bool found = false;
            for(const auto& cf : cameraFeaturesVector) {
                if(cf.socket == socket) {
                    found = true;
                    break;
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
    for(const auto& cf : cameraFeaturesVector) {
        if(cf.socket == boardSocket) {
            found = true;
            cameraFeatures = cf;
            break;
        }
    }
    auto fps = sensorFps.value_or(-1.0f);
    // Check if the sensor resolution is valid, if specified
    if(sensorResolution.has_value()) {
        if(static_cast<int32_t>(sensorResolution->first) > cameraFeatures.width || static_cast<int32_t>(sensorResolution->second) > cameraFeatures.height) {
            throw std::runtime_error("Invalid sensor resolution specified, maximum supported resolution is " + std::to_string(cameraFeatures.width) + "x"
                                     + std::to_string(cameraFeatures.height));
        }
        found = false;
        for(const auto& config : cameraFeatures.configs) {
            auto signedWidth = static_cast<int32_t>(sensorResolution->first);
            auto signedHeight = static_cast<int32_t>(sensorResolution->second);
            if(config.width == signedWidth && config.height == signedHeight) {
                found = true;
                properties.resolutionWidth = signedWidth;
                properties.resolutionHeight = signedHeight;
                if(fps > config.maxFps || (fps < config.minFps && fps != -1.0f)) {
                    throw std::runtime_error("Invalid sensor FPS specified, supported range is " + std::to_string(config.minFps) + " - "
                                             + std::to_string(config.maxFps));
                }
                break;
            }
        }
        if(!found) {
            throw std::runtime_error(
                "Invalid sensor resolution specified - check the supported resolutions for the connected device with device.getConnectedCameraFeatures()");
        }
    }

    if(fps != -1.0f) {
        properties.fps = fps;
    }

    if(!found) {
        throw std::runtime_error("Camera socket not found on the connected device");
    }

    properties.boardSocket = boardSocket;
    isBuilt = true;
    return std::static_pointer_cast<Camera>(shared_from_this());
}

Camera::Properties& Camera::getProperties() {
    properties.initialControl = initialControl;
    return properties;
}

// Get current board socket
CameraBoardSocket Camera::getBoardSocket() const {
    if(!isBuilt) {
        throw std::runtime_error("Camera node must be built before calling getBoardSocket()");
    }
    return properties.boardSocket;
}

uint32_t Camera::getMaxWidth() const {
    if(!isBuilt) {
        throw std::runtime_error("Camera node must be built before calling getMaxWidth()");
    }
    return properties.resolutionWidth > 0 ? properties.resolutionWidth : cameraFeatures.width;
}

uint32_t Camera::getMaxHeight() const {
    if(!isBuilt) {
        throw std::runtime_error("Camera node must be built before calling getMaxHeight()");
    }
    return properties.resolutionHeight > 0 ? properties.resolutionHeight : cameraFeatures.height;
}

Node::Output* Camera::requestFullResolutionOutput(std::optional<ImgFrame::Type> type, std::optional<float> fps, bool useHighestResolution) {
    if(!isBuilt) {
        throw std::runtime_error("Camera node must be built before requesting outputs from it");
    }
    if(!device) {
        throw std::runtime_error("Invalid device pointer");
    }

    ImgFrameCapability cap;
    if(useHighestResolution) {
        cap.size.fixed({cameraFeatures.width, cameraFeatures.height});
    } else {
        int32_t maxWidth = 0;
        int32_t maxHeight = 0;
        // Loop over all the configs and find the highest resolution that is not higher than 5000x4000
        for(const auto& config : cameraFeatures.configs) {
            // Only consider the full FOV configurations
            auto denormalizedFov = config.fov.denormalize(cameraFeatures.width, cameraFeatures.height);
            if((static_cast<int>(denormalizedFov.width) != cameraFeatures.width) || (static_cast<int>(denormalizedFov.height) != cameraFeatures.height)
               || denormalizedFov.x != 0 || denormalizedFov.y != 0) {
                continue;
            }
            if(config.width <= 5000 && config.height <= 4000) {
                if(config.width > maxWidth || config.height > maxHeight) {
                    maxWidth = config.width;
                    maxHeight = config.height;
                }
            }
        }
        if(maxWidth == 0 || maxHeight == 0) {
            throw std::runtime_error("No valid full resolution configuration found for the connected camera");
        }
        cap.size.fixed({maxWidth, maxHeight});
    }
    if(fps.has_value()) {
        cap.fps.fixed(fps.value());
    }
    cap.type = type;
    return pimpl->requestOutput(*this, cap, false);
}

Node::Output* Camera::requestOutput(std::pair<uint32_t, uint32_t> size,
                                    std::optional<ImgFrame::Type> type,
                                    ImgResizeMode resizeMode,
                                    std::optional<float> fps,
                                    std::optional<bool> enableUndistortion) {
    ImgFrameCapability cap;
    cap.size.fixed(size);

    if(fps.has_value()) {
        cap.fps.fixed(fps.value());
    }

    cap.type = type;
    cap.resizeMode = resizeMode;
    cap.enableUndistortion = enableUndistortion;
    return pimpl->requestOutput(*this, cap, false);
}

Node::Output* Camera::requestOutput(const Capability& capability, bool onHost) {
    return pimpl->requestOutput(*this, capability, onHost);
}

void Camera::buildStage1() {
    return pimpl->buildStage1(*this);
}

bool Camera::isSourceNode() const {
    return true;
}

NodeRecordParams Camera::getNodeRecordParams() const {
    if(properties.boardSocket == CameraBoardSocket::AUTO) {
        throw std::runtime_error("For record and replay functionality, board socket must be specified (Camera).");
    }
    NodeRecordParams params;
    params.video = true;
    params.name = "Camera" + toString(properties.boardSocket);
    return params;
}

/*
Camera::Output& Camera::getRecordOutput() {
    return isp;
}
Camera::Input& Camera::getReplayInput() {
    return mockIsp;
}
*/

}  // namespace node
}  // namespace dai
