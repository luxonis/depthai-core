#include "depthai/pipeline/node/Camera.hpp"
// std
#include <fstream>
#include <stdexcept>
#include <variant>

// libraries
#include <spimpl.h>

// depthai internal
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

class Camera::Impl {
   public:
    enum class OutputType { PREVIEW, VIDEO, RAW };

    static OutputType getOutputType(ImgFrame::Type format) {
        using E = ImgFrame::Type;
        switch(format) {
            case E::YUV422i:  // interleaved 8 bit
            case E::YUV444p:  // planar 4:4:4 format
            case E::YUV420p:  // planar 4:2:0 format
            case E::YUV422p:  // planar 8 bit
            case E::YUV400p:  // 8-bit greyscale
            case E::YUV444i:
            case E::NV12:
            case E::NV21:
                return OutputType::VIDEO;
            case E::RGBA8888:       // RGBA interleaved stored in 32 bit word
            case E::RGB161616:      // Planar 16 bit RGB data
            case E::RGB888p:        // Planar 8 bit RGB data
            case E::BGR888p:        // Planar 8 bit BGR data
            case E::RGB888i:        // Interleaved 8 bit RGB data
            case E::BGR888i:        // Interleaved 8 bit BGR data
            case E::RGBF16F16F16p:  // Planar FP16 RGB data
            case E::BGRF16F16F16p:  // Planar FP16 BGR data
            case E::RGBF16F16F16i:  // Interleaved FP16 RGB data
            case E::BGRF16F16F16i:  // Interleaved FP16 BGR data
            case E::GRAY8:          // 8 bit grayscale (1 plane)
            case E::GRAYF16:        // FP16 grayscale (normalized)
                return OutputType::PREVIEW;
            case E::LUT2:   // 1 bit  per pixel, Lookup table (used for graphics layers)
            case E::LUT4:   // 2 bits per pixel, Lookup table (used for graphics layers)
            case E::LUT16:  // 4 bits per pixel, Lookup table (used for graphics layers)
                DAI_CHECK(false, "LUT formats on camera NOT IMPLEMENTED YET");
                break;
            case E::RAW16:  // save any raw type (8, 10, 12bit) on 16 bits
            case E::RAW14:  // 14bit value in 16bit storage
            case E::RAW12:  // 12bit value in 16bit storage
            case E::RAW10:  // 10bit value in 16bit storage
            case E::RAW8:
            case E::PACK10:  // SIPP 10bit packed format
            case E::PACK12:  // SIPP 12bit packed format
            case E::RAW32:   // 32 bits raw
                return OutputType::RAW;
            case E::BITSTREAM:  // used for video encoder bitstream
            case E::HDR:
                DAI_CHECK_IN(false);
                break;
            case E::NONE:
                DAI_CHECK_IN(false);
                break;
            default:
                DAI_CHECK_IN(false);
                break;
        }
    }
};

Camera::Camera(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(std::move(props)) {}

void Camera::build() {
    properties.boardSocket = CameraBoardSocket::AUTO;
    properties.imageOrientation = CameraImageOrientation::AUTO;
    properties.colorOrder = CameraProperties::ColorOrder::BGR;
    properties.interleaved = true;
    properties.previewHeight = 300;
    properties.previewWidth = 300;
    properties.fps = 30.0;
    properties.previewKeepAspectRatio = true;
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

// Set camera image orientation
void Camera::setImageOrientation(CameraImageOrientation imageOrientation) {
    properties.imageOrientation = imageOrientation;
}

// Get camera image orientation
CameraImageOrientation Camera::getImageOrientation() const {
    // TODO: in case of AUTO, see if possible to return actual value determined by device?
    return properties.imageOrientation;
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
void Camera::setSize(int width, int height) {
    properties.resolutionWidth = width;
    properties.resolutionHeight = height;
}

void Camera::setSize(std::tuple<int, int> size) {
    setSize(std::get<0>(size), std::get<1>(size));
}

// set still output size
void Camera::setStillSize(int width, int height) {
    properties.stillWidth = width;
    properties.stillHeight = height;
}

void Camera::setStillSize(std::tuple<int, int> size) {
    setStillSize(std::get<0>(size), std::get<1>(size));
}

// void Camera::setIspScale(int horizNum, int horizDenom, int vertNum, int vertDenom) {
//     properties.ispScale.horizNumerator = horizNum;
//     properties.ispScale.horizDenominator = horizDenom;
//     properties.ispScale.vertNumerator = vertNum;
//     properties.ispScale.vertDenominator = vertDenom;
// }

// void Camera::setIspScale(int numerator, int denominator) {
//     setIspScale(numerator, denominator, numerator, denominator);
// }

// void Camera::setIspScale(std::tuple<int, int> scale) {
//     setIspScale(std::get<0>(scale), std::get<1>(scale));
// }

// void Camera::setIspScale(std::tuple<int, int> horizScale, std::tuple<int, int> vertScale) {
//     setIspScale(std::get<0>(horizScale), std::get<1>(horizScale), std::get<0>(vertScale), std::get<1>(vertScale));
// }

// void Camera::setResolution(CameraProperties::SensorResolution resolution) {
//     properties.resolution = resolution;
// }
// CameraProperties::SensorResolution Camera::getResolution() const {
//     return properties.resolution;
// }

void Camera::setFps(float fps) {
    properties.fps = fps;
}

void Camera::setIsp3aFps(int isp3aFps) {
    properties.isp3aFps = isp3aFps;
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
std::tuple<int, int> Camera::getSize() const {
    // TODO(themarpe) - revisit
    return {properties.resolutionWidth, properties.resolutionHeight};
}

int Camera::getWidth() const {
    return std::get<0>(getSize());
}

int Camera::getHeight() const {
    return std::get<1>(getSize());
}

// void Camera::sensorCenterCrop() {
//     properties.sensorCropX = CameraProperties::AUTO;
//     properties.sensorCropY = CameraProperties::AUTO;
// }

// void Camera::setSensorCrop(float x, float y) {
//     if(x < 0 || x >= 1) {
//         throw std::invalid_argument("Sensor crop x must be specified as normalized value [0:1)");
//     }
//     if(y < 0 || y >= 1) {
//         throw std::invalid_argument("Sensor crop y must be specified as normalized value [0:1)");
//     }
//     properties.sensorCropX = x;
//     properties.sensorCropY = y;
// }

// std::tuple<float, float> Camera::getSensorCrop() const {
//     return {properties.sensorCropX, properties.sensorCropY};
// }

// float Camera::getSensorCropX() const {
//     return std::get<0>(getSensorCrop());
// }

// float Camera::getSensorCropY() const {
//     return std::get<1>(getSensorCrop());
// }

void Camera::setMeshSource(Camera::Properties::WarpMeshSource source) {
    properties.warpMeshSource = source;
}
Camera::Properties::WarpMeshSource Camera::getMeshSource() const {
    return properties.warpMeshSource;
}

void Camera::loadMeshData(span<const std::uint8_t> data) {
    if(data.size() <= 0) {
        throw std::runtime_error("Camera | mesh data must not be empty");
    }

    Asset meshAsset;
    std::string assetKey;
    meshAsset.alignment = 64;

    meshAsset.data = std::vector<uint8_t>(data.begin(), data.end());
    assetKey = "warpMesh";
    properties.warpMeshUri = assetManager.set(assetKey, meshAsset)->getRelativeUri();
}

void Camera::loadMeshFile(const dai::Path& warpMesh) {
    std::ifstream streamMesh(warpMesh, std::ios::binary);
    if(!streamMesh.is_open()) {
        throw std::runtime_error(fmt::format("Camera | Cannot open mesh at path: {}", warpMesh.u8string()));
    }
    std::vector<std::uint8_t> data = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(streamMesh), {});

    loadMeshData(data);
}

void Camera::setMeshStep(int width, int height) {
    properties.warpMeshStepWidth = width;
    properties.warpMeshStepHeight = height;
}
std::tuple<int, int> Camera::getMeshStep() const {
    return {properties.warpMeshStepWidth, properties.warpMeshStepHeight};
}

void Camera::setCalibrationAlpha(float alpha) {
    properties.calibAlpha = alpha;
}

std::optional<float> Camera::getCalibrationAlpha() const {
    return properties.calibAlpha;
}

void Camera::setRawOutputPacked(bool packed) {
    properties.rawPacked = packed;
}

Camera::Output* Camera::requestNewOutput(const ImgFrameCapability& capability, bool onHost) {
    (void)onHost;  // This will be used for optimizing network troughput
    DAI_CHECK_V(preview.getConnections().empty() && video.getConnections().empty() && isp.getConnections().empty() && still.getConnections().empty(),
                "Can't use managed and unmanaged mode at the same time. Don't link() preview, video, isp, still outputs or don't use requestNewOutput().");
    const std::optional<ImgFrame::Type> encoding = (capability.encoding && *capability.encoding != ImgFrame::Type::NONE) ? capability.encoding : std::nullopt;
    if(encoding) {
        // TODO(jakgra) set fp16, colorOrder, interleaved, rawPacked ... from encoding and throw for unsupported encodings
    }
    // TODO(jakgra) check if video default output is ok for all cameras / sensors?
    const auto outputType = encoding ? pimpl->getOutputType(*encoding) : Impl::OutputType::VIDEO;
    if(capability.size.value) {
        const auto* size = std::get_if<std::tuple<uint32_t, uint32_t>>(&(*capability.size.value));
        if(size != nullptr) {
            using E = Impl::OutputType;
            switch(outputType) {
                case E::PREVIEW:
                    std::cout << "Setting preview size\n" << std::flush;
                    setPreviewSize(*size);
                    break;
                case E::VIDEO:
                    std::cout << "Setting video size\n" << std::flush;
                    setVideoSize(*size);
                    break;
                case E::RAW:
                    // TODO(jakgra) check if is an exact fit for some SensorConfig or throw otherwise
                    std::cout << "Not setting size because is RAW output\n" << std::flush;
                    break;
                default:
                    DAI_CHECK_IN(false);
                    break;
            }
        } else {
            // TODO(jakgra) support this
            DAI_CHECK(false, "Ranged and array sizes not supported yet. Only fixed size works for now");
        }
    } else {
        // TODO(jakgra) set some default resolution that matches a sensor config here
        // should this be the same for all cameras (ex.: full HD) or camera specific (ex.: max resolution)
        // if there are multiple outputs, should the above logic apply or should we take the same resolution as another output and just return that output?
        std::cout << "NOT Setting preview size because value is NULL\n" << std::flush;
    }
    switch(outputType) {
        using E = Impl::OutputType;
        case E::PREVIEW:
            return &preview;
        case E::RAW:
            return &raw;
        case E::VIDEO:
            return &video;
        default:
            DAI_CHECK_IN(false);
            break;
    }
}

}  // namespace node
}  // namespace dai
