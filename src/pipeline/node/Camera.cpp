#include "depthai/pipeline/node/Camera.hpp"

#include <cmath>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

Camera::Camera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId), rawControl(std::make_shared<RawCameraControl>()), initialControl(rawControl) {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.imageOrientation = CameraImageOrientation::AUTO;
    properties.colorOrder = CameraProperties::ColorOrder::BGR;
    properties.interleaved = true;
    properties.previewHeight = 300;
    properties.previewWidth = 300;
    properties.resolution = CameraProperties::SensorResolution::THE_1080_P;
    properties.fps = 30.0;
    properties.previewKeepAspectRatio = true;

    inputs = {&inputConfig, &inputControl};
    outputs = {&video, &preview, &still, &isp, &raw};
}

std::string Camera::getName() const {
    return "Camera";
}

nlohmann::json Camera::getProperties() {
    nlohmann::json j;
    properties.initialControl = *rawControl;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> Camera::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

// Set board socket to use
void Camera::setBoardSocket(dai::CameraBoardSocket boardSocket) {
    properties.boardSocket = boardSocket;
}

// Get current board socket
CameraBoardSocket Camera::getBoardSocket() const {
    return properties.boardSocket;
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

void Camera::setResolution(CameraProperties::SensorResolution resolution) {
    properties.resolution = resolution;
}
CameraProperties::SensorResolution Camera::getResolution() const {
    return properties.resolution;
}

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
    if(properties.videoWidth == CameraProperties::AUTO || properties.videoHeight == CameraProperties::AUTO) {
        // calculate based on auto
        int maxVideoWidth = 1920;
        int maxVideoHeight = 1080;

        if(properties.resolution == CameraProperties::SensorResolution::THE_4_K
           || properties.resolution == CameraProperties::SensorResolution::THE_12_MP
           || properties.resolution == CameraProperties::SensorResolution::THE_13_MP) {
            maxVideoWidth = 3840;
            maxVideoHeight = 2160;
        }

        if(properties.resolution == CameraProperties::SensorResolution::THE_1200_P) {
            maxVideoHeight = 1200;
        }

        if(properties.resolution == CameraProperties::SensorResolution::THE_5_MP) {
            maxVideoWidth = 2592;
            maxVideoHeight = 1944;
        }

        // Take into the account the ISP scaling
        int numW = properties.ispScale.horizNumerator;
        int denW = properties.ispScale.horizDenominator;
        if(numW > 0 && denW > 0) {
            maxVideoWidth = getScaledSize(maxVideoWidth, numW, denW);
        }

        int numH = properties.ispScale.vertNumerator;
        int denH = properties.ispScale.vertDenominator;
        if(numH > 0 && denH > 0) {
            maxVideoHeight = getScaledSize(maxVideoHeight, numH, denH);
        }

        return {maxVideoWidth, maxVideoHeight};
    }

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
    // Calculate from AUTO
    if(properties.stillWidth == CameraProperties::AUTO || properties.stillHeight == CameraProperties::AUTO) {
        int maxStillWidth = 1920;
        int maxStillHeight = 1080;
        if(properties.resolution == dai::CameraProperties::SensorResolution::THE_1200_P) {
            maxStillHeight = 1200;
        }
        if(properties.resolution == dai::CameraProperties::SensorResolution::THE_4_K) {
            maxStillWidth = 3840;
            maxStillHeight = 2160;
        }
        if(properties.resolution == dai::CameraProperties::SensorResolution::THE_5_MP) {
            maxStillWidth = 2592;
            maxStillHeight = 1944;
        }
        if(properties.resolution == dai::CameraProperties::SensorResolution::THE_12_MP) {
            maxStillWidth = 4032;  // Note not 4056 as full sensor resolution
            maxStillHeight = 3040;
        }
        if(properties.resolution == dai::CameraProperties::SensorResolution::THE_13_MP) {
            maxStillWidth = 4192;  // Note not 4208 as full sensor resolution
            maxStillHeight = 3120;
        }

        // Take into the account the ISP scaling
        int numW = properties.ispScale.horizNumerator;
        int denW = properties.ispScale.horizDenominator;
        if(numW > 0 && denW > 0) {
            maxStillWidth = getScaledSize(maxStillWidth, numW, denW);
        }

        int numH = properties.ispScale.vertNumerator;
        int denH = properties.ispScale.vertDenominator;
        if(numH > 0 && denH > 0) {
            maxStillHeight = getScaledSize(maxStillHeight, numH, denH);
        }

        return {maxStillWidth, maxStillHeight};
    }

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
    switch(properties.resolution) {
        case CameraProperties::SensorResolution::THE_1080_P:
            return {1920, 1080};
            break;

        case CameraProperties::SensorResolution::THE_1200_P:
            return {1920, 1200};
            break;

        case CameraProperties::SensorResolution::THE_4_K:
            return {3840, 2160};
            break;

        case CameraProperties::SensorResolution::THE_5_MP:
            return {2592, 1944};
            break;

        case CameraProperties::SensorResolution::THE_12_MP:
            return {4056, 3040};
            break;

        case CameraProperties::SensorResolution::THE_13_MP:
            return {4208, 3120};
            break;

        case CameraProperties::SensorResolution::THE_720_P:
            return {1280, 720};
            break;

        case CameraProperties::SensorResolution::THE_800_P:
            return {1280, 800};
            break;

        case CameraProperties::SensorResolution::THE_400_P:
            return {640, 400};
            break;

        case CameraProperties::SensorResolution::THE_480_P:
            return {640, 480};
            break;
    }

    return {1920, 1080};
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
    // AUTO - center crop by default
    if(properties.sensorCropX == CameraProperties::AUTO || properties.sensorCropY == CameraProperties::AUTO) {
        float x = std::floor(((getResolutionWidth() - getVideoWidth()) / 2.0f) / getResolutionWidth());
        float y = std::floor(((getResolutionHeight() - getVideoHeight()) / 2.0f) / getResolutionHeight());
        return {x, y};
    }
    return {properties.sensorCropX, properties.sensorCropY};
}

float Camera::getSensorCropX() const {
    return std::get<0>(getSensorCrop());
}

float Camera::getSensorCropY() const {
    return std::get<1>(getSensorCrop());
}

void Camera::setWaitForConfigInput(bool wait) {
    properties.inputConfigSync = wait;
}

bool Camera::getWaitForConfigInput() {
    return properties.inputConfigSync;
}

void Camera::setPreviewKeepAspectRatio(bool keep) {
    properties.previewKeepAspectRatio = keep;
}

bool Camera::getPreviewKeepAspectRatio() {
    return properties.previewKeepAspectRatio;
}

}  // namespace node
}  // namespace dai
