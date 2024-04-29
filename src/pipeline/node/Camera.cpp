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
    enum class OutputType { PREVIEW, VIDEO, RAW };

    struct OutputRequest {
        int32_t id{};
        ImgFrameCapability capability;
        bool onHost{false};
    };

    // TODO(jakgra) fix so all resolutions work
    struct Res {
        int32_t width;
        int32_t height;
    };

    int32_t nextOutputRequestId = 0;

    std::vector<Res> notWorkingResolutions = {
        // {.width = 1352, .height = 1012}  // dai::ColorCameraProperties::SensorResolution::THE_1352X1012
    };

    std::vector<OutputRequest> outputRequests;

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

    static std::vector<dai::CameraSensorConfig> getColorCameraConfigs(dai::Device& device) {
        // const auto& cameraFeatures = device.getConnectedCameraFeatures();
        std::vector<dai::CameraFeatures> cameraFeatures;
        dai::CameraFeatures feature;
        feature.socket = dai::CameraBoardSocket::CAM_A;
        dai::CameraSensorConfig config;
        config.width = 1920;
        config.height = 1080;
        feature.configs.push_back(config);
        cameraFeatures.push_back(feature);
        std::cout << "Got n features: " << cameraFeatures.size() << "\n" << std::flush;
        const auto& camera = std::find_if(
            cameraFeatures.begin(), cameraFeatures.end(), [](const dai::CameraFeatures& itr) -> bool { return itr.socket == dai::CameraBoardSocket::CAM_A; });
        if(camera == cameraFeatures.end()) {
            throw std::runtime_error("Device doesn't support ColorCamera");
        }
        std::vector<dai::CameraSensorConfig> colorCameraModes;
        std::copy_if(camera->configs.begin(), camera->configs.end(), std::back_inserter(colorCameraModes), [](const auto& itr) {
            return true;
            // TODO(jakgra) correctly handle this
            return itr.type == dai::CameraSensorType::COLOR;
        });
        return colorCameraModes;
    }

    dai::CameraSensorConfig getClosestCameraConfig(const std::vector<dai::CameraSensorConfig>& colorCameraModes, int64_t width, int64_t height) {
        int64_t minAdditionalPixels = -1;
        ssize_t foundIndex = -1;
        ssize_t index = 0;
        for(const auto& mode : colorCameraModes) {
            if(mode.width >= width && mode.height >= height) {
                if(std::find_if(notWorkingResolutions.begin(),
                                notWorkingResolutions.end(),
                                [&mode = std::as_const(mode)](const auto& itr) { return itr.width == mode.width && itr.height == mode.height; })
                   != notWorkingResolutions.end()) {
                    std::cout << "Warning: ignoring possible best resolution " << mode.width << "x" << mode.height << "   because of possible firmware bugs\n"
                              << std::flush;
                } else {
                    int64_t additionalPixels = (mode.width - width) * mode.height + (mode.height - height) * mode.width;
                    if(minAdditionalPixels == -1 || additionalPixels < minAdditionalPixels) {
                        foundIndex = index;
                        minAdditionalPixels = additionalPixels;
                    }
                }
            }
            ++index;
        }
        if(minAdditionalPixels == -1 || foundIndex == -1) {
            throw std::runtime_error("This camera can't provide the wanted resolution " + std::to_string(width) + "x" + std::to_string(height));
        }
        return colorCameraModes[foundIndex];
    }

    static std::set<std::tuple<double, int, int>> validIspScales() {
        std::set<std::tuple<double, int, int>> result;
        for(int numerator = 1; numerator < 16 + 1; ++numerator) {
            for(int denominator = 1; denominator < 63 + 1; ++denominator) {
                // Chroma needs 2x extra downscaling
                if(denominator < 32 || numerator % 2 == 0) {
                    // Only if irreducible
                    if(std::gcd(numerator, denominator) == 1) {
                        result.insert(std::make_tuple(static_cast<double>(numerator) / static_cast<double>(denominator), numerator, denominator));
                    }
                }
            }
        }
        return result;
    }

    static std::tuple<int, int> findClosestIspScale(int width, int height, const dai::CameraSensorConfig& mode) {
        const static auto validScales = validIspScales();
        const auto useWidth = static_cast<double>(width) / static_cast<double>(mode.width) > static_cast<double>(height) / static_cast<double>(mode.height);
        int numerator = useWidth ? width : height;
        int denominator = useWidth ? mode.width : mode.height;
        const auto div = std::gcd(numerator, denominator);
        numerator = numerator / div;
        denominator = denominator / div;
        const auto foundScale = std::find_if(validScales.begin(), validScales.end(), [numerator, denominator](const auto& itr) {
            return std::get<1>(itr) == numerator && std::get<2>(itr) == denominator;
        });
        if(foundScale != validScales.end()) {
            return std::make_tuple(std::get<1>(*foundScale), std::get<2>(*foundScale));
        }
        const double wantedScale = static_cast<double>(numerator) / static_cast<double>(denominator);
        double bestDiff = std::numeric_limits<double>::max();
        auto bestScale = std::make_tuple<int, int>(-1, -1);
        for(const auto& validScale : validScales) {
            const auto scale = std::get<0>(validScale);
            if(scale >= wantedScale) {
                const auto diff = scale - wantedScale;
                if(diff < bestDiff) {
                    bestDiff = diff;
                    bestScale = std::make_tuple(std::get<1>(validScale), std::get<2>(validScale));
                }
            }
        }
        if(std::get<0>(bestScale) == -1) {
            throw std::runtime_error("Couldn't find correct scale");
        }
        return bestScale;
    }

    void setupDynamicOutputsRvc4(Camera& parent, const std::shared_ptr<Device>& device, CameraProperties& properties) {
        /*
          DAI_CHECK_IN(!outputRequests.empty());
          uint32_t index = 0;
          for(const auto& outputRequest : outputRequests) {
              outputRequest.output->name = "out" + std::to_string(index);
              parent.setOutputRefs(outputRequest.output.get());
              ++index;
          }
          */
    }

    void setupDynamicOutputs(Camera& parent, const std::shared_ptr<Device>& device, CameraProperties& properties) {
        /*
          DAI_CHECK_IN(!outputRequests.empty());
          const auto& outputRequest = outputRequests[0];
          const auto& capability = outputRequest.capability;
          const std::optional<ImgFrame::Type> encoding =
              (capability.encoding && *capability.encoding != ImgFrame::Type::NONE) ? capability.encoding : std::nullopt;
          if(encoding) {
              // TODO(jakgra) set fp16, colorOrder, interleaved, rawPacked ... from encoding and throw for unsupported encodings
          }
          // TODO(jakgra) check if video default output is ok for all cameras / sensors?
          const auto outputType = encoding ? getOutputType(*encoding) : OutputType::VIDEO;
          DAI_CHECK_IN(device);
          // TODO(jakgra) support mono camera also. And limit by sensor or not...???
          const auto& colorCameraConfigs = getColorCameraConfigs(*device);
          if(capability.size.value) {
              if(const auto* size = std::get_if<std::tuple<uint32_t, uint32_t>>(&(*capability.size.value))) {
                  const auto& mode = getClosestCameraConfig(colorCameraConfigs, std::get<0>(*size), std::get<1>(*size));
                  // TODO(jakgra) set this to the one the mode was gotten from
                  // setBoardSocket();
                  std::cout << "Setting sensor resolution to: " << mode.width << "x" << mode.height << "\n" << std::flush;
                  properties.resolutionWidth = mode.width;
                  properties.resolutionHeight = mode.height;

                  */
        /**
         * isp scale doesn't work correctly on rvc2 FW camera node and doesn't work on rvc4
         * so disabling for now, we can maybe reenable it for rvc2 sometime.

         const auto [num, denom] = findClosestIspScale(static_cast<int>(std::get<0>(*size)), static_cast<int>(std::get<1>(*size)), mode);
         std::cout << "USING ISP SCALE " << num << "/" << denom << "\n" << std::flush;
         properties.ispScale.horizNumerator = num;
         properties.ispScale.horizDenominator = denom;
         properties.ispScale.vertNumerator = num;
         properties.ispScale.vertDenominator = denom;

        */
        /*

                  std::cout << "Setting preview/video/raw size to: " << std::get<0>(*size) << "x" << std::get<1>(*size) << "\n" << std::flush;
                  using E = OutputType;
                  switch(outputType) {
                      case E::PREVIEW:
                          std::cout << "Setting preview size\n" << std::flush;
                          properties.previewWidth = std::get<0>(*size);
                          properties.previewHeight = std::get<1>(*size);
                          break;
                      case E::VIDEO:
                          std::cout << "Setting video size\n" << std::flush;
                          properties.videoWidth = std::get<0>(*size);
                          properties.videoHeight = std::get<1>(*size);
                          break;
                      case E::RAW:
                          DAI_CHECK_V(std::get<0>(*size) == static_cast<uint32_t>(mode.width) && std::get<1>(*size) == static_cast<uint32_t>(mode.height),
                                      "Raw output has to be the exact same size as a supported sensor resolution. {}x{} is not a supported sensor resolution.",
                                      std::get<0>(*size),
                                      std::get<1>(*size));
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
              // if there are multiple outputs, should the above logic apply or should we take the same resolution as another output and just return that
              // output?
              std::cout << "NOT Setting preview size because value is NULL\n" << std::flush;
          }
          switch(outputType) {
              using E = OutputType;
              case E::PREVIEW:
                  outputRequest.output->name = "preview";
                  break;
              case E::RAW:
                  outputRequest.output->name = "raw";
                  break;
              case E::VIDEO:
                  outputRequest.output->name = "video";
                  break;
              default:
                  DAI_CHECK_IN(false);
                  break;
          }
          DAI_CHECK_IN(outputRequest.output);
          parent.setOutputRefs(outputRequest.output.get());
          */
    }

    void buildStage1(Camera& parent,
                     std::shared_ptr<Device>& device,
                     CameraProperties& properties,
                     Output& preview,  // NOLINT(bugprone-easily-swappable-parameters)
                     Output& video,    // NOLINT(bugprone-easily-swappable-parameters)
                     Output& raw       // NOLINT(bugprone-easily-swappable-parameters)
    ) {
        if(outputRequests.empty()) {
            std::cout << "video setup brand new\n" << std::flush;
            // parent.setOutputRefs(&preview);
            // parent.setOutputRefs(&video);
            // parent.setOutputRefs(&raw);
        } else {
            DAI_CHECK_V(preview.getConnections().empty() && video.getConnections().empty(),
                        "Can't use managed and unmanaged mode at the same time for outputs preview, video and raw. "
                        "Don't link() preview, video, raw outputs or don't use requestNewOutput().");
            if(device->getDeviceInfo().platform == X_LINK_RVC4) {
                setupDynamicOutputsRvc4(parent, device, properties);
            } else {
                setupDynamicOutputs(parent, device, properties);
            }
        }
    }

    Node::Output* requestNewOutput(Camera& parent, const Capability& genericCapability, bool onHost) {
        if(const auto* capability = ImgFrameCapability::get(genericCapability)) {
            static const int32_t requestId = nextOutputRequestId;
            outputRequests.push_back({requestId, *capability, onHost});
            ++nextOutputRequestId;
            std::cout << "output request " << requestId << "\n" << std::flush;
            CameraProperties::OutputSpec spec;
            spec.width = 1920;
            spec.height = 1080;
            if(capability->size.value) {
                if(const auto* size = std::get_if<std::tuple<uint32_t, uint32_t>>(&(*(*capability).size.value))) {
                    spec.width = static_cast<int>(std::get<0>(*size));
                    spec.height = static_cast<int>(std::get<1>(*size));
                } else {
                    // TODO(jakgra) add support for other logic here
                    DAI_CHECK_IN(false);
                }
            } else {
                // TODO(jakgra) add support for other logic here
                DAI_CHECK_IN(false);
            }
            spec.fps = 30;
            parent.properties.outputSpecs.push_back(spec);
            // parent.setOutputRefs(&parent.video);
            return &parent.dynamicOutputs[std::to_string(requestId)];
            // return &parent.video;
            // return &parent.dynamicOutputs[std::to_string(requestId)];
        }
        return nullptr;
    }
};

Camera::Camera() : pimpl(spimpl::make_impl<Impl>()) {}

Camera::Camera(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(std::move(props)), pimpl(spimpl::make_impl<Impl>()) {}

Camera::Camera(std::shared_ptr<Device>& defaultDevice)
    : DeviceNodeCRTP<DeviceNode, Camera, CameraProperties>(defaultDevice), pimpl(spimpl::make_impl<Impl>()) {}

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

Node::Output* Camera::requestNewOutput(const Capability& capability, bool onHost) {
    return pimpl->requestNewOutput(*this, capability, onHost);
}

void Camera::buildStage1() {
    pimpl->buildStage1(*this, device, properties, preview, video, raw);
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
