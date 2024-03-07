#include "depthai/pipeline/node/ColorCamera.hpp"

#include <cmath>

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ColorCamera::ColorCamera(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, ColorCamera, ColorCameraProperties>(std::move(props)) {}

void ColorCamera::build() {
    // Set some default properties
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.imageOrientation = CameraImageOrientation::AUTO;
    properties.previewType = ImgFrame::Type::BGR888i;
    properties.previewHeight = 300;
    properties.previewWidth = 300;
    properties.resolution = ColorCameraProperties::SensorResolution::THE_1080_P;
    properties.fps = 30.0;
    properties.previewKeepAspectRatio = true;
}

ColorCamera::Properties& ColorCamera::getProperties() {
    properties.initialControl = initialControl;
    return properties;
}

// Set board socket to use
void ColorCamera::setBoardSocket(dai::CameraBoardSocket boardSocket) {
    properties.boardSocket = boardSocket;
}

// Get current board socket
CameraBoardSocket ColorCamera::getBoardSocket() const {
    return properties.boardSocket;
}

void ColorCamera::setCamera(std::string name) {
    properties.cameraName = name;
}

std::string ColorCamera::getCamera() const {
    return properties.cameraName;
}

// Set which color camera to use
void ColorCamera::setCamId(int64_t camId) {
    // cast to board socket
    switch(camId) {
        case 0:
            properties.boardSocket = CameraBoardSocket::CAM_A;
            break;
        case 1:
            properties.boardSocket = CameraBoardSocket::CAM_B;
            break;
        case 2:
            properties.boardSocket = CameraBoardSocket::CAM_C;
            break;
        case 3:
            properties.boardSocket = CameraBoardSocket::CAM_D;
            break;
        default:
            throw std::invalid_argument(fmt::format("CamId value: {} is invalid.", camId));
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    bool isInterleaved = ImgFrame::isInterleaved(properties.previewType);
    bool isFp16 = getFp16();
#pragma GCC diagnostic pop
    switch(colorOrder) {
        case ColorCameraProperties::ColorOrder::BGR:
            if(isInterleaved) {
                if(isFp16) {
                    properties.previewType = ImgFrame::Type::BGRF16F16F16i;
                } else {
                    properties.previewType = ImgFrame::Type::BGR888i;
                }
            } else {
                if(isFp16) {
                    properties.previewType = ImgFrame::Type::BGRF16F16F16p;
                } else {
                    properties.previewType = ImgFrame::Type::BGR888p;
                }
            }
            break;
        case ColorCameraProperties::ColorOrder::RGB:
            if(isInterleaved) {
                if(isFp16) {
                    properties.previewType = ImgFrame::Type::RGBF16F16F16i;
                } else {
                    properties.previewType = ImgFrame::Type::RGB888i;
                }
            } else {
                if(isFp16) {
                    properties.previewType = ImgFrame::Type::RGBF16F16F16p;
                } else {
                    properties.previewType = ImgFrame::Type::RGB888p;
                }
            }
            break;
    }
}

// getColorOrder - returns color order
ColorCameraProperties::ColorOrder ColorCamera::getColorOrder() const {
    if(properties.previewType == ImgFrame::Type::RGB888i || properties.previewType == ImgFrame::Type::RGB888p
       || properties.previewType == ImgFrame::Type::RGBF16F16F16i || properties.previewType == ImgFrame::Type::RGBF16F16F16p) {
        return ColorCameraProperties::ColorOrder::RGB;
    } else if(properties.previewType == ImgFrame::Type::BGR888i || properties.previewType == ImgFrame::Type::BGR888p
              || properties.previewType == ImgFrame::Type::BGRF16F16F16i || properties.previewType == ImgFrame::Type::BGRF16F16F16p) {
        return ColorCameraProperties::ColorOrder::BGR;
    } else {
        throw std::runtime_error("Don't call getColorOrder() for wrong previewType");
    }
}

// setInterleaved
void ColorCamera::setInterleaved(bool interleaved) {
    if(ImgFrame::isInterleaved(properties.previewType)) {
        if(!interleaved) {
            properties.previewType = ImgFrame::toPlanar(properties.previewType);
        }
    } else {
        if(interleaved) {
            properties.previewType = ImgFrame::toInterleaved(properties.previewType);
        }
    }
}

// getInterleaved
bool ColorCamera::getInterleaved() const {
    return ImgFrame::isInterleaved(properties.previewType);
}

/// Set type of preview output image
void ColorCamera::setPreviewType(ImgFrame::Type type) {
    properties.previewType = type;
}

/// Get the preview type
ImgFrame::Type ColorCamera::getPreviewType() const {
    return properties.previewType;
}

// setFp16
void ColorCamera::setFp16(bool fp16) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    auto order = getColorOrder();
    auto interleaved = getInterleaved();
#pragma GCC diagnostic pop
    if(fp16) {
        if(order == ColorCameraProperties::ColorOrder::BGR) {
            if(interleaved) {
                properties.previewType = ImgFrame::Type::BGRF16F16F16i;
            } else {
                properties.previewType = ImgFrame::Type::BGRF16F16F16p;
            }
        } else {
            if(interleaved) {
                properties.previewType = ImgFrame::Type::RGBF16F16F16i;
            } else {
                properties.previewType = ImgFrame::Type::RGBF16F16F16p;
            }
        }
    } else {
        if(order == ColorCameraProperties::ColorOrder::BGR) {
            if(interleaved) {
                properties.previewType = ImgFrame::Type::BGR888i;
            } else {
                properties.previewType = ImgFrame::Type::BGR888p;
            }
        } else {
            if(interleaved) {
                properties.previewType = ImgFrame::Type::RGB888i;
            } else {
                properties.previewType = ImgFrame::Type::RGB888p;
            }
        }
    }
}

// getFp16
bool ColorCamera::getFp16() const {
    if(properties.previewType == ImgFrame::Type::BGRF16F16F16i || properties.previewType == ImgFrame::Type::BGRF16F16F16p
       || properties.previewType == ImgFrame::Type::RGBF16F16F16i || properties.previewType == ImgFrame::Type::RGBF16F16F16p) {
        return true;
    } else {
        return false;
    }
}

// set preview output size
void ColorCamera::setPreviewSize(int width, int height) {
    properties.previewWidth = width;
    properties.previewHeight = height;
}

void ColorCamera::setPreviewSize(std::tuple<int, int> size) {
    setPreviewSize(std::get<0>(size), std::get<1>(size));
}

// set video output size
void ColorCamera::setVideoSize(int width, int height) {
    properties.videoWidth = width;
    properties.videoHeight = height;
}

void ColorCamera::setVideoSize(std::tuple<int, int> size) {
    setVideoSize(std::get<0>(size), std::get<1>(size));
}

// set still output size
void ColorCamera::setStillSize(int width, int height) {
    properties.stillWidth = width;
    properties.stillHeight = height;
}

void ColorCamera::setStillSize(std::tuple<int, int> size) {
    setStillSize(std::get<0>(size), std::get<1>(size));
}

void ColorCamera::setIspScale(int horizNum, int horizDenom, int vertNum, int vertDenom) {
    properties.ispScale.horizNumerator = horizNum;
    properties.ispScale.horizDenominator = horizDenom;
    properties.ispScale.vertNumerator = vertNum;
    properties.ispScale.vertDenominator = vertDenom;
}

void ColorCamera::setIspScale(int numerator, int denominator) {
    setIspScale(numerator, denominator, numerator, denominator);
}

void ColorCamera::setIspScale(std::tuple<int, int> scale) {
    setIspScale(std::get<0>(scale), std::get<1>(scale));
}

void ColorCamera::setIspScale(std::tuple<int, int> horizScale, std::tuple<int, int> vertScale) {
    setIspScale(std::get<0>(horizScale), std::get<1>(horizScale), std::get<0>(vertScale), std::get<1>(vertScale));
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

void ColorCamera::setIsp3aFps(int isp3aFps) {
    properties.isp3aFps = isp3aFps;
}

void ColorCamera::setFrameEventFilter(const std::vector<dai::FrameEvent>& events) {
    properties.eventFilter = events;
}

std::vector<dai::FrameEvent> ColorCamera::getFrameEventFilter() const {
    return properties.eventFilter;
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
           || properties.resolution == ColorCameraProperties::SensorResolution::THE_12_MP
           || properties.resolution == ColorCameraProperties::SensorResolution::THE_4000X3000
           || properties.resolution == ColorCameraProperties::SensorResolution::THE_13_MP
           || properties.resolution == ColorCameraProperties::SensorResolution::THE_5312X6000
           || properties.resolution == ColorCameraProperties::SensorResolution::THE_48_MP) {
            maxVideoWidth = 3840;
            maxVideoHeight = 2160;
        }

        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_1200_P) {
            maxVideoHeight = 1200;
        }

        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_5_MP) {
            maxVideoWidth = 2592;
            maxVideoHeight = 1944;
        }

        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_720_P) {
            maxVideoWidth = 1280;
            maxVideoHeight = 720;
        }

        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_800_P) {
            maxVideoWidth = 1280;
            maxVideoHeight = 800;
        }

        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_1440X1080) {
            maxVideoWidth = 1440;
        }
        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_2024X1520) {
            maxVideoWidth = 2024;
        }
        if(properties.resolution == ColorCameraProperties::SensorResolution::THE_1352X1012) {
            maxVideoWidth = 1352;
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
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_1200_P) {
            maxStillHeight = 1200;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_4_K) {
            maxStillWidth = 3840;
            maxStillHeight = 2160;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_5_MP) {
            maxStillWidth = 2592;
            maxStillHeight = 1944;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_4000X3000) {
            maxStillWidth = 4000;
            maxStillHeight = 3000;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_12_MP) {
            maxStillWidth = 4032;  // Note not 4056 as full sensor resolution
            maxStillHeight = 3040;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_13_MP) {
            maxStillWidth = 4192;  // Note not 4208 as full sensor resolution
            maxStillHeight = 3120;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_5312X6000) {
            maxStillWidth = 5312;
            maxStillHeight = 6000;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_48_MP) {
            maxStillWidth = 8000;
            maxStillHeight = 6000;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_1440X1080) {
            maxStillWidth = 1440;
            maxStillHeight = 1080;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_2024X1520) {
            maxStillWidth = 2024;
            maxStillHeight = 1520;
        }
        if(properties.resolution == dai::ColorCameraProperties::SensorResolution::THE_1352X1012) {
            maxStillWidth = 1352;
            maxStillHeight = 1012;
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

        case ColorCameraProperties::SensorResolution::THE_1200_P:
            return {1920, 1200};
            break;

        case ColorCameraProperties::SensorResolution::THE_4_K:
            return {3840, 2160};
            break;

        case ColorCameraProperties::SensorResolution::THE_5_MP:
            return {2592, 1944};
            break;

        case ColorCameraProperties::SensorResolution::THE_4000X3000:
            return {4000, 3000};
            break;

        case ColorCameraProperties::SensorResolution::THE_12_MP:
            return {4056, 3040};
            break;

        case ColorCameraProperties::SensorResolution::THE_13_MP:
            return {4208, 3120};
            break;

        case ColorCameraProperties::SensorResolution::THE_720_P:
            return {1280, 720};
            break;

        case ColorCameraProperties::SensorResolution::THE_800_P:
            return {1280, 800};
            break;

        case ColorCameraProperties::SensorResolution::THE_5312X6000:
            return {5312, 6000};
            break;

        case ColorCameraProperties::SensorResolution::THE_48_MP:
            return {8000, 6000};
            break;

        case ColorCameraProperties::SensorResolution::THE_240X180:
            return {240, 180};
            break;

        case ColorCameraProperties::SensorResolution::THE_1280X962:
            return {1280, 962};
            break;

        case ColorCameraProperties::SensorResolution::THE_2000X1500:
            return {2000, 1500};
            break;

        case ColorCameraProperties::SensorResolution::THE_2028X1520:
            return {2028, 1520};
            break;

        case ColorCameraProperties::SensorResolution::THE_2104X1560:
            return {2104, 1560};
            break;

        case ColorCameraProperties::SensorResolution::THE_1440X1080:
            return {1440, 1080};
            break;

        case ColorCameraProperties::SensorResolution::THE_2024X1520:
            return {2024, 1520};
            break;

        case ColorCameraProperties::SensorResolution::THE_1352X1012:
            return {1352, 1012};
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

int ColorCamera::getScaledSize(int input, int num, int denom) const {
    return (input * num - 1) / denom + 1;
}

int ColorCamera::getIspWidth() const {
    int inW = getResolutionWidth();
    int num = properties.ispScale.horizNumerator;
    int den = properties.ispScale.horizDenominator;
    if(num > 0 && den > 0) {
        return getScaledSize(inW, num, den);
    }
    return inW;
}

int ColorCamera::getIspHeight() const {
    int inH = getResolutionHeight();
    int num = properties.ispScale.vertNumerator;
    int den = properties.ispScale.vertDenominator;
    if(num > 0 && den > 0) {
        return getScaledSize(inH, num, den);
    }
    return inH;
}

std::tuple<int, int> ColorCamera::getIspSize() const {
    return {getIspWidth(), getIspHeight()};
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
        float x = std::floor(((getResolutionWidth() - getVideoWidth()) / 2.0f)) / getResolutionWidth();
        float y = std::floor(((getResolutionHeight() - getVideoHeight()) / 2.0f)) / getResolutionHeight();
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
    inputConfig.setWaitForMessage(wait);
}

bool ColorCamera::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void ColorCamera::setPreviewKeepAspectRatio(bool keep) {
    properties.previewKeepAspectRatio = keep;
}

bool ColorCamera::getPreviewKeepAspectRatio() {
    return properties.previewKeepAspectRatio;
}

void ColorCamera::setNumFramesPool(int numRaw, int numIsp, int numPreview, int numVideo, int numStill) {
    properties.numFramesPoolRaw = numRaw;
    properties.numFramesPoolIsp = numIsp;
    properties.numFramesPoolPreview = numPreview;
    properties.numFramesPoolVideo = numVideo;
    properties.numFramesPoolStill = numStill;
}

void ColorCamera::setPreviewNumFramesPool(int num) {
    properties.numFramesPoolPreview = num;
}
void ColorCamera::setVideoNumFramesPool(int num) {
    properties.numFramesPoolVideo = num;
}
void ColorCamera::setStillNumFramesPool(int num) {
    properties.numFramesPoolStill = num;
}
void ColorCamera::setRawNumFramesPool(int num) {
    properties.numFramesPoolRaw = num;
}
void ColorCamera::setIspNumFramesPool(int num) {
    properties.numFramesPoolIsp = num;
}

int ColorCamera::getPreviewNumFramesPool() {
    return properties.numFramesPoolPreview;
}
int ColorCamera::getVideoNumFramesPool() {
    return properties.numFramesPoolVideo;
}
int ColorCamera::getStillNumFramesPool() {
    return properties.numFramesPoolStill;
}
int ColorCamera::getRawNumFramesPool() {
    return properties.numFramesPoolRaw;
}
int ColorCamera::getIspNumFramesPool() {
    return properties.numFramesPoolIsp;
}

void ColorCamera::setMeshSource(ColorCamera::Properties::WarpMeshSource source) {
    properties.warpMeshSource = source;
}
ColorCamera::Properties::WarpMeshSource ColorCamera::getMeshSource() const {
    return properties.warpMeshSource;
}

void ColorCamera::loadMeshData(const std::vector<std::uint8_t> data) {
    if(data.size() <= 0) {
        throw std::runtime_error("Camera | mesh data must not be empty");
    }

    Asset meshAsset;
    std::string assetKey;
    meshAsset.alignment = 64;

    meshAsset.data = data;
    assetKey = "warpMesh";
    properties.warpMeshUri = assetManager.set(assetKey, meshAsset)->getRelativeUri();

    setMeshSource(ColorCamera::Properties::WarpMeshSource::URI);
}

// void ColorCamera::loadMeshFile(const dai::Path& warpMesh) {
//     std::ifstream streamMesh(warpMesh, std::ios::binary);
//     if(!streamMesh.is_open()) {
//         throw std::runtime_error(fmt::format("Camera | Cannot open mesh at path: {}", warpMesh.u8string()));
//     }
//     std::vector<std::uint8_t> data = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamMesh), {});

//     loadMeshData(data);
// }

void ColorCamera::setMeshStep(int width, int height) {
    properties.warpMeshStepWidth = width;
    properties.warpMeshStepHeight = height;
}
void ColorCamera::setMeshSize(int width, int height) {
    properties.warpMeshWidth = width;
    properties.warpMeshHeight = height;
}
std::tuple<int, int> ColorCamera::getMeshStep() const {
    return {properties.warpMeshStepWidth, properties.warpMeshStepHeight};
}
std::tuple<int, int> ColorCamera::getMeshSize() const {
    return {properties.warpMeshWidth, properties.warpMeshHeight};
}

void ColorCamera::setCalibrationAlpha(float alpha) {
    properties.calibAlpha = alpha;
}

float ColorCamera::getCalibrationAlpha() const {
    return properties.calibAlpha;
}

void ColorCamera::setRawOutputPacked(bool packed) {
    properties.rawPacked = packed;
}

bool ColorCamera::isSourceNode() const {
    return true;
}

utility::NodeRecordParams ColorCamera::getNodeRecordParams() const {
    if(properties.boardSocket == CameraBoardSocket::AUTO) {
        throw std::runtime_error("For record and replay functionality, board socket must be specified (Camera).");
    }
    utility::NodeRecordParams params;
    params.name = "Camera" + toString(properties.boardSocket);
    return params;
}

ColorCamera::Output& ColorCamera::getRecordOutput() {
    return isp;
}
ColorCamera::Input& ColorCamera::getReplayInput() {
    return mockIsp;
}

}  // namespace node
}  // namespace dai
