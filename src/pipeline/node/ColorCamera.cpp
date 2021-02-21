#include "depthai/pipeline/node/ColorCamera.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ColorCamera::ColorCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawControl(std::make_shared<RawCameraControl>()), initialControl(rawControl) {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.imageOrientation = CameraImageOrientation::AUTO;
    properties.colorOrder = ColorCameraProperties::ColorOrder::BGR;
    properties.interleaved = true;
    properties.previewHeight = 300;
    properties.previewWidth = 300;
    properties.resolution = ColorCameraProperties::SensorResolution::THE_1080_P;
    properties.fps = 30.0;
    properties.previewKeepAspectRatio = true;
}

std::string ColorCamera::getName() const {
    return "ColorCamera";
}

std::vector<Node::Output> ColorCamera::getOutputs() {
    return {video, preview, still};
}

std::vector<Node::Input> ColorCamera::getInputs() {
    return {inputConfig, inputControl};
}

nlohmann::json ColorCamera::getProperties() {
    nlohmann::json j;
    properties.initialControl = *rawControl;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> ColorCamera::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

// Set board socket to use
void ColorCamera::setBoardSocket(dai::CameraBoardSocket boardSocket) {
    properties.boardSocket = boardSocket;
}

// Get current board socket
CameraBoardSocket ColorCamera::getBoardSocket() const {
    return properties.boardSocket;
}

// Set which color camera to use
void ColorCamera::setCamId(int64_t id) {
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
int64_t ColorCamera::getCamId() const {
    return (int64_t)properties.boardSocket;
}

// Set camera image orientation
void ColorCamera::setImageOrientation(CameraImageOrientation imageOrientation) {
    properties.imageOrientation = imageOrientation;
}

// Get camera image orientation
CameraImageOrientation ColorCamera::getImageOrientation() const {
    // TODO: in case of AUTO, see if possible to return actual value determined by device?
    return properties.imageOrientation;
}

// setColorOrder - RGB or BGR
void ColorCamera::setColorOrder(ColorCameraProperties::ColorOrder colorOrder) {
    properties.colorOrder = colorOrder;
}

// getColorOrder - returns color order
ColorCameraProperties::ColorOrder ColorCamera::getColorOrder() const {
    return properties.colorOrder;
}

// setInterleaved
void ColorCamera::setInterleaved(bool interleaved) {
    properties.interleaved = interleaved;
}

// getInterleaved
bool ColorCamera::getInterleaved() const {
    return properties.interleaved;
}

// setFp16
void ColorCamera::setFp16(bool fp16) {
    properties.fp16 = fp16;
}

// getFp16
bool ColorCamera::getFp16() const {
    return properties.fp16;
}

// set preview output size
void ColorCamera::setPreviewSize(int width, int height) {
    properties.previewWidth = width;
    properties.previewHeight = height;
}

// set video output size
void ColorCamera::setVideoSize(int width, int height) {
    properties.videoWidth = width;
    properties.videoHeight = height;
}

// set still output size
void ColorCamera::setStillSize(int width, int height) {
    properties.stillWidth = width;
    properties.stillHeight = height;
}

void ColorCamera::setResolution(ColorCameraProperties::SensorResolution resolution) {
    properties.resolution = resolution;
}
ColorCameraProperties::SensorResolution ColorCamera::getResolution() const {
    return properties.resolution;
}

void ColorCamera::setFps(float fps) {
    properties.fps = fps;
}

float ColorCamera::getFps() const {
    // if AUTO
    if(properties.fps == ColorCameraProperties::AUTO || properties.fps == 0) {
        return 30.0f;
    }

    // else return fps
    return properties.fps;
}

// Returns preview size
std::tuple<int, int> ColorCamera::getPreviewSize() const {
    return {properties.previewWidth, properties.previewHeight};
}

int ColorCamera::getPreviewWidth() const {
    return properties.previewWidth;
}

int ColorCamera::getPreviewHeight() const {
    return properties.previewHeight;
}

// Returns video size
std::tuple<int, int> ColorCamera::getVideoSize() const {
    if(properties.videoWidth == ColorCameraProperties::AUTO || properties.videoHeight == ColorCameraProperties::AUTO) {
        // calculate based on auto
        int maxVideoWidth = 1920;
        int maxVideoHeight = 1080;

        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_4_K
           || properties.resolution == ColorCameraProperties::SensorResolution::THE_12_MP) {
            maxVideoWidth = 3840;
            maxVideoHeight = 2160;
        }

        return {maxVideoWidth, maxVideoHeight};
    }

    return {properties.videoWidth, properties.videoHeight};
}

int ColorCamera::getVideoWidth() const {
    return std::get<0>(getVideoSize());
}

int ColorCamera::getVideoHeight() const {
    return std::get<1>(getVideoSize());
}

// Returns still size
std::tuple<int, int> ColorCamera::getStillSize() const {
    // Calculate from AUTO
    if(properties.stillWidth == ColorCameraProperties::AUTO || properties.stillHeight == ColorCameraProperties::AUTO) {
        int maxStillWidth = 1920;
        int maxStillHeight = 1080;
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_4_K) {
            maxStillWidth = 3840;
            maxStillHeight = 2160;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_12_MP) {
            maxStillWidth = 4032;  // Note not 4056 as full sensor resolution
            maxStillHeight = 3040;
        }
        return {maxStillWidth, maxStillHeight};
    }

    // Else return size set
    return {properties.stillWidth, properties.stillHeight};
}

int ColorCamera::getStillWidth() const {
    return std::get<0>(getStillSize());
}

int ColorCamera::getStillHeight() const {
    return std::get<1>(getStillSize());
}

// Returns sensor size
std::tuple<int, int> ColorCamera::getResolutionSize() const {
    switch(properties.resolution) {
        case ColorCameraProperties::SensorResolution::THE_1080_P:
            return {1920, 1080};
            break;

        case ColorCameraProperties::SensorResolution::THE_4_K:
            return {3840, 2160};
            break;

        case ColorCameraProperties::SensorResolution::THE_12_MP:
            return {4056, 3040};
            break;
    }

    return {1920, 1080};
}

int ColorCamera::getResolutionWidth() const {
    return std::get<0>(getResolutionSize());
}

int ColorCamera::getResolutionHeight() const {
    return std::get<1>(getResolutionSize());
}

void ColorCamera::sensorCenterCrop() {
    properties.sensorCropX = ColorCameraProperties::AUTO;
    properties.sensorCropY = ColorCameraProperties::AUTO;
}

void ColorCamera::setSensorCrop(float x, float y) {
    if(x < 0 || x >= 1) {
        throw std::invalid_argument("Sensor crop x must be specified as normalized value [0:1)");
    }
    if(y < 0 || y >= 1) {
        throw std::invalid_argument("Sensor crop y must be specified as normalized value [0:1)");
    }
    properties.sensorCropX = x;
    properties.sensorCropY = y;
}

std::tuple<float, float> ColorCamera::getSensorCrop() const {
    // AUTO - center crop by default
    if(properties.sensorCropX == ColorCameraProperties::AUTO || properties.sensorCropY == ColorCameraProperties::AUTO) {
        float x = ((getResolutionWidth() - getVideoWidth()) / 2) / getResolutionWidth();
        float y = ((getResolutionHeight() - getVideoHeight()) / 2) / getResolutionHeight();
        return {x, y};
    }
    return {properties.sensorCropX, properties.sensorCropY};
}

float ColorCamera::getSensorCropX() const {
    return std::get<0>(getSensorCrop());
}

float ColorCamera::getSensorCropY() const {
    return std::get<1>(getSensorCrop());
}

void ColorCamera::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

bool ColorCamera::getWaitForConfigInput() {
    return properties.inputConfigSync;
}

void ColorCamera::setPreviewKeepAspectRatio(bool keep) {
    properties.previewKeepAspectRatio = keep;
}

bool ColorCamera::getPreviewKeepAspectRatio() {
    return properties.previewKeepAspectRatio;
}

}  // namespace node
}  // namespace dai
