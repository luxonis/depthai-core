#include "depthai/pipeline/node/MonoCamera.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

MonoCamera::MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawControl(std::make_shared<RawCameraControl>()), initialControl(rawControl) {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.resolution = MonoCameraProperties::SensorResolution::THE_720_P;
    properties.fps = 30.0;
}

std::string MonoCamera::getName() const {
    return "MonoCamera";
}

std::vector<Node::Output> MonoCamera::getOutputs() {
    return {out, raw};
}

std::vector<Node::Input> MonoCamera::getInputs() {
    return {inputControl};
}

nlohmann::json MonoCamera::getProperties() {
    nlohmann::json j;
    properties.initialControl = *rawControl;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> MonoCamera::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

// Set board socket to use
void MonoCamera::setBoardSocket(dai::CameraBoardSocket boardSocket) {
    properties.boardSocket = boardSocket;
}

// Get current board socket
CameraBoardSocket MonoCamera::getBoardSocket() const {
    return properties.boardSocket;
}

// Set which color camera to use
void MonoCamera::setCamId(int64_t id) {
    // cast to board socket
    switch(id) {
        case 0:
            properties.boardSocket = CameraBoardSocket::RGB;
            break;
        case 1:
            properties.boardSocket = CameraBoardSocket::LEFT;
            break;
        case 2:
            properties.boardSocket = CameraBoardSocket::RIGHT;
            break;
        default:
            throw std::invalid_argument(fmt::format("CamId value: {} is invalid.", id));
            break;
    }
}

// Get which color camera to use
int64_t MonoCamera::getCamId() const {
    return (int64_t)properties.boardSocket;
}

// Set camera image orientation
void MonoCamera::setImageOrientation(CameraImageOrientation imageOrientation) {
    properties.imageOrientation = imageOrientation;
}

// Get camera image orientation
CameraImageOrientation MonoCamera::getImageOrientation() const {
    // TODO: in case of AUTO, see if possible to return actual value determined by device?
    return properties.imageOrientation;
}

void MonoCamera::setResolution(MonoCameraProperties::SensorResolution resolution) {
    properties.resolution = resolution;
}

MonoCameraProperties::SensorResolution MonoCamera::getResolution() const {
    return properties.resolution;
}

void MonoCamera::setFps(float fps) {
    properties.fps = fps;
}

float MonoCamera::getFps() const {
    // if AUTO
    if(properties.fps == -1 || properties.fps == 0) {
        return 30.0f;
    }

    // else return fps
    return properties.fps;
}

std::tuple<int, int> MonoCamera::getResolutionSize() const {
    switch(properties.resolution) {
        case MonoCameraProperties::SensorResolution::THE_400_P:
            return {640, 400};
            break;

        case MonoCameraProperties::SensorResolution::THE_720_P:
            return {1280, 720};
            break;

        case MonoCameraProperties::SensorResolution::THE_800_P:
            return {1280, 800};
            break;
    }
    return {1280, 720};
}

int MonoCamera::getResolutionWidth() const {
    return std::get<0>(getResolutionSize());
}

int MonoCamera::getResolutionHeight() const {
    return std::get<1>(getResolutionSize());
}

}  // namespace node
}  // namespace dai
