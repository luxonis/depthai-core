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

Camera::Camera(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(std::move(props)), pimpl(spimpl::make_impl<Impl>()) {}

Camera::Camera(std::shared_ptr<Device>& defaultDevice)
    : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(defaultDevice), pimpl(spimpl::make_impl<Impl>()) {}

std::shared_ptr<Camera> Camera::build() {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.imageOrientation = CameraImageOrientation::AUTO;
    properties.colorOrder = CameraProperties::ColorOrder::BGR;
    properties.interleaved = true;
    properties.previewHeight = 300;
    properties.previewWidth = 300;
    properties.fps = 30.0;
    properties.previewKeepAspectRatio = true;
    isBuild = true;
    return std::static_pointer_cast<Camera>(shared_from_this());
}

Camera::Properties& Camera::getProperties() {
    properties.initialControl = initialControl;
    return properties;
}

// Set board socket to use
void Camera::setBoardSocket(dai::CameraBoardSocket boardSocket) {
    properties.boardSocket = boardSocket;
}

// Get current board socket
CameraBoardSocket Camera::getBoardSocket() const {
    return properties.boardSocket;
}

void Camera::setCamera(std::string name) {
    properties.cameraName = name;
}

std::string Camera::getCamera() const {
    return properties.cameraName;
}

Node::Output* Camera::requestOutput(std::pair<uint32_t, uint32_t> size, ImgFrame::Type encoding, ImgResizeMode resizeMode, uint32_t fps) {
    ImgFrameCapability cap;
    cap.size.fixed(size);
    cap.fps.fixed(fps);
    cap.encoding = encoding;
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

utility::NodeRecordParams Camera::getNodeRecordParams() const {
    if(properties.boardSocket == CameraBoardSocket::AUTO) {
        throw std::runtime_error("For record and replay functionality, board socket must be specified (Camera).");
    }
    utility::NodeRecordParams params;
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
