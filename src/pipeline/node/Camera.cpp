#include "depthai/pipeline/node/Camera.hpp"

#include <cmath>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

Camera::Camera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Camera(par, nodeId, std::make_unique<Camera::Properties>()) {}
Camera::Camera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Camera, CameraProperties>(par, nodeId, std::move(props)), rawControl(std::make_shared<RawCameraControl>()), initialControl(rawControl) {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.imageOrientation = CameraImageOrientation::AUTO;
    properties.colorOrder = CameraProperties::ColorOrder::BGR;
    properties.interleaved = true;
    properties.previewHeight = 300;
    properties.previewWidth = 300;
    properties.fps = 30.0;
    properties.previewKeepAspectRatio = true;

    setInputRefs({&inputConfig, &inputControl});
    setOutputRefs({&video, &preview, &still, &isp, &raw, &frameEvent});
}

Camera::Properties& Camera::getProperties() {
    properties.initialControl = *rawControl;
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

// Set which color camera to use
void Camera::setCamId(int64_t id) {
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
        case 3:
            properties.boardSocket = CameraBoardSocket::CAM_D;
            break;
        default:
            throw std::invalid_argument(fmt::format("CamId value: {} is invalid.", id));
            break;
    }
}

// Get which color camera to use
int64_t Camera::getCamId() const {
    return (int64_t)properties.boardSocket;
}

// Set camera image orientation
void Camera::setImageOrientation(CameraImageOrientation imageOrientation) {
    properties.imageOrientation = imageOrientation;
}

// Get camera image orientation
CameraImageOrientation Camera::getImageOrientation() const {
    // TODO: in case of AUTO, see if possible to return actual value determined by device?
    return properties.imageOrientation;
}

// setColorOrder - RGB or BGR
void Camera::setColorOrder(CameraProperties::ColorOrder colorOrder) {
    properties.colorOrder = colorOrder;
}

// getColorOrder - returns color order
CameraProperties::ColorOrder Camera::getColorOrder() const {
    return properties.colorOrder;
}

// setInterleaved
void Camera::setInterleaved(bool interleaved) {
    properties.interleaved = interleaved;
}

// getInterleaved
bool Camera::getInterleaved() const {
    return properties.interleaved;
}

// setFp16
void Camera::setFp16(bool fp16) {
    properties.fp16 = fp16;
}

// getFp16
bool Camera::getFp16() const {
    return properties.fp16;
}

// set preview output size
void Camera::setPreviewSize(int width, int height) {
    properties.previewWidth = width;
    properties.previewHeight = height;
}

void Camera::setPreviewSize(std::tuple<int, int> size) {
    setPreviewSize(std::get<0>(size), std::get<1>(size));
}

// set video output size
void Camera::setVideoSize(int width, int height) {
    properties.videoWidth = width;
    properties.videoHeight = height;
}

void Camera::setVideoSize(std::tuple<int, int> size) {
    setVideoSize(std::get<0>(size), std::get<1>(size));
}

// set still output size
void Camera::setSensorSize(int width, int height) {
    properties.resolutionWidth = width;
    properties.resolutionHeight = height;
}

void Camera::setSensorSize(std::tuple<int, int> size) {
    setSensorSize(std::get<0>(size), std::get<1>(size));
}

// set still output size
void Camera::setStillSize(int width, int height) {
    properties.stillWidth = width;
    properties.stillHeight = height;
}

void Camera::setStillSize(std::tuple<int, int> size) {
    setStillSize(std::get<0>(size), std::get<1>(size));
}

void Camera::setIspScale(int horizNum, int horizDenom, int vertNum, int vertDenom) {
    properties.ispScale.horizNumerator = horizNum;
    properties.ispScale.horizDenominator = horizDenom;
    properties.ispScale.vertNumerator = vertNum;
    properties.ispScale.vertDenominator = vertDenom;
}

void Camera::setIspScale(int numerator, int denominator) {
    setIspScale(numerator, denominator, numerator, denominator);
}

void Camera::setIspScale(std::tuple<int, int> scale) {
    setIspScale(std::get<0>(scale), std::get<1>(scale));
}

void Camera::setIspScale(std::tuple<int, int> horizScale, std::tuple<int, int> vertScale) {
    setIspScale(std::get<0>(horizScale), std::get<1>(horizScale), std::get<0>(vertScale), std::get<1>(vertScale));
}

// void Camera::setResolution(CameraProperties::SensorResolution resolution) {
//     properties.resolution = resolution;
// }
// CameraProperties::SensorResolution Camera::getResolution() const {
//     return properties.resolution;
// }

void Camera::setFps(float fps) {
    properties.fps = fps;
}

float Camera::getFps() const {
    // if AUTO
    if(properties.fps == CameraProperties::AUTO || properties.fps == 0) {
        return 30.0f;
    }

    // else return fps
    return properties.fps;
}

// Returns preview size
std::tuple<int, int> Camera::getPreviewSize() const {
    return {properties.previewWidth, properties.previewHeight};
}

int Camera::getPreviewWidth() const {
    return properties.previewWidth;
}

int Camera::getPreviewHeight() const {
    return properties.previewHeight;
}

// Returns video size
std::tuple<int, int> Camera::getVideoSize() const {
    // TODO(themarpe) - revisit
    return {properties.videoWidth, properties.videoHeight};
}

int Camera::getVideoWidth() const {
    return std::get<0>(getVideoSize());
}

int Camera::getVideoHeight() const {
    return std::get<1>(getVideoSize());
}

// Returns still size
std::tuple<int, int> Camera::getStillSize() const {
    // TODO(themarpe) - revisit
    // Else return size set
    return {properties.stillWidth, properties.stillHeight};
}

int Camera::getStillWidth() const {
    return std::get<0>(getStillSize());
}

int Camera::getStillHeight() const {
    return std::get<1>(getStillSize());
}

// Returns sensor size
std::tuple<int, int> Camera::getResolutionSize() const {
    // TODO(themarpe) - revisit
    return {properties.resolutionWidth, properties.resolutionHeight};
}

int Camera::getResolutionWidth() const {
    return std::get<0>(getResolutionSize());
}

int Camera::getResolutionHeight() const {
    return std::get<1>(getResolutionSize());
}

int Camera::getScaledSize(int input, int num, int denom) const {
    return (input * num - 1) / denom + 1;
}

int Camera::getIspWidth() const {
    int inW = getResolutionWidth();
    int num = properties.ispScale.horizNumerator;
    int den = properties.ispScale.horizDenominator;
    if(num > 0 && den > 0) {
        return getScaledSize(inW, num, den);
    }
    return inW;
}

int Camera::getIspHeight() const {
    int inH = getResolutionHeight();
    int num = properties.ispScale.vertNumerator;
    int den = properties.ispScale.vertDenominator;
    if(num > 0 && den > 0) {
        return getScaledSize(inH, num, den);
    }
    return inH;
}

std::tuple<int, int> Camera::getIspSize() const {
    return {getIspWidth(), getIspHeight()};
}

void Camera::sensorCenterCrop() {
    properties.sensorCropX = CameraProperties::AUTO;
    properties.sensorCropY = CameraProperties::AUTO;
}

void Camera::setSensorCrop(float x, float y) {
    if(x < 0 || x >= 1) {
        throw std::invalid_argument("Sensor crop x must be specified as normalized value [0:1)");
    }
    if(y < 0 || y >= 1) {
        throw std::invalid_argument("Sensor crop y must be specified as normalized value [0:1)");
    }
    properties.sensorCropX = x;
    properties.sensorCropY = y;
}

std::tuple<float, float> Camera::getSensorCrop() const {
    return {properties.sensorCropX, properties.sensorCropY};
}

float Camera::getSensorCropX() const {
    return std::get<0>(getSensorCrop());
}

float Camera::getSensorCropY() const {
    return std::get<1>(getSensorCrop());
}

void Camera::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool Camera::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void Camera::setPreviewKeepAspectRatio(bool keep) {
    properties.previewKeepAspectRatio = keep;
}

bool Camera::getPreviewKeepAspectRatio() {
    return properties.previewKeepAspectRatio;
}

void Camera::setNumFramesPool(int raw, int isp, int preview, int video, int still) {
    properties.numFramesPoolRaw = raw;
    properties.numFramesPoolIsp = isp;
    properties.numFramesPoolPreview = preview;
    properties.numFramesPoolVideo = video;
    properties.numFramesPoolStill = still;
}

void Camera::setPreviewNumFramesPool(int num) {
    properties.numFramesPoolPreview = num;
}
void Camera::setVideoNumFramesPool(int num) {
    properties.numFramesPoolVideo = num;
}
void Camera::setStillNumFramesPool(int num) {
    properties.numFramesPoolStill = num;
}
void Camera::setRawNumFramesPool(int num) {
    properties.numFramesPoolRaw = num;
}
void Camera::setIspNumFramesPool(int num) {
    properties.numFramesPoolIsp = num;
}

int Camera::getPreviewNumFramesPool() {
    return properties.numFramesPoolPreview;
}
int Camera::getVideoNumFramesPool() {
    return properties.numFramesPoolVideo;
}
int Camera::getStillNumFramesPool() {
    return properties.numFramesPoolStill;
}
int Camera::getRawNumFramesPool() {
    return properties.numFramesPoolRaw;
}
int Camera::getIspNumFramesPool() {
    return properties.numFramesPoolIsp;
}

}  // namespace node
}  // namespace dai
