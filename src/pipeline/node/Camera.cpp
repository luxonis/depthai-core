#include "depthai/pipeline/node/Camera.hpp"
// std
#include <fstream>
#include <memory>
#include <stdexcept>
#include <utility>

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

std::shared_ptr<Camera> Camera::build(CameraBoardSocket boardSocket) {
    if(isBuilt) {
        throw std::runtime_error("Camera node is already built");
    }
    if(!device) {
        throw std::runtime_error("Device pointer is not valid");
    }

    auto cameraFeatures = device->getConnectedCameraFeatures();
    // First handle the case if the boardSocket is set to AUTO
    if(boardSocket == CameraBoardSocket::AUTO) {
        auto defaultSockets = {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C};
        for(auto socket : defaultSockets) {
            bool found = false;
            for(const auto& cf : cameraFeatures) {
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
    return maxWidth;
}

uint32_t Camera::getMaxHeight() const {
    if(!isBuilt) {
        throw std::runtime_error("Camera node must be built before calling getMaxHeight()");
    }
    return maxHeight;
}

Node::Output* Camera::requestFullResolutionOutput(ImgFrame::Type type, float fps) {
    if(!isBuilt) {
        throw std::runtime_error("Camera node must be built before requesting outputs from it");
    }
    if(!device) {
        throw std::runtime_error("Invalid device pointer");
    }

    if(maxHeight == 0 || maxWidth == 0) {
        throw std::runtime_error(fmt::format("Invalid max width or height - {}x{}", maxWidth, maxHeight));
    }
    ImgFrameCapability cap;
    cap.size.fixed({maxWidth, maxHeight});
    cap.fps.fixed(fps);
    cap.type = type;
    return pimpl->requestOutput(*this, cap, false);
}

Node::Output* Camera::requestOutput(std::pair<uint32_t, uint32_t> size, std::optional<ImgFrame::Type> type, ImgResizeMode resizeMode, float fps) {
    ImgFrameCapability cap;
    cap.size.fixed(size);
    cap.fps.fixed(fps);
    cap.type = type;
    cap.resizeMode = resizeMode;
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
Camera::Input& Camera::getReplayInput() {
    return mockIsp;
}

std::pair<size_t, size_t> Camera::getMaxRequestedSize() const {
    uint32_t maxWidth = 0;
    uint32_t maxHeight = 0;
    for(const auto& outputRequest : pimpl->outputRequests) {
        if(outputRequest.capability.size.value) {
            if(const auto* size = std::get_if<std::pair<uint32_t, uint32_t>>(&(*outputRequest.capability.size.value))) {
                maxWidth = std::max(maxWidth, size->first);
                maxHeight = std::max(maxHeight, size->second);
            } else {
                throw std::runtime_error("Unsupported size value");
            }
        }
    }
    if(maxWidth == 0 || maxHeight == 0) {
        throw std::runtime_error("Invalid max width or height");
    }
    return std::make_pair(maxWidth, maxHeight);
};
float Camera::getMaxRequestedFps() const {
    float maxFps = 0;
    for(const auto& outputRequest : pimpl->outputRequests) {
        if(outputRequest.capability.fps.value) {
            if(const auto* fps = std::get_if<float>(&(*outputRequest.capability.fps.value))) {
                maxFps = std::max(maxFps, *fps);
            } else if(const auto* fps = std::get_if<std::pair<float, float>>(&(*outputRequest.capability.fps.value))) {
                maxFps = std::max(maxFps, std::get<1>(*fps));
            } else if(const auto* fps = std::get_if<std::vector<float>>(&(*outputRequest.capability.fps.value))) {
                DAI_CHECK(fps->size() > 0, "When passing a vector to ImgFrameCapability->fps, please pass a non empty vector!");
                maxFps = std::max(maxFps, (*fps)[0]);
            } else {
                throw std::runtime_error("Unsupported fps value");
            }
        }
    }
    return maxFps == 0 ? 30 : maxFps;
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
