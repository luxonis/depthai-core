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

void Camera::setMockIspSize(int width, int height) {
    properties.mockIspWidth = width;
    properties.mockIspHeight = height;
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

Node::Output* Camera::requestOutput(const Capability& capability, bool onHost) {
    return pimpl->requestOutput(*this, capability, onHost);
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

Camera::Output& Camera::getRecordOutput() {
    return isp;
}
Camera::Input& Camera::getReplayInput() {
    return mockIsp;
}

}  // namespace node
}  // namespace dai
