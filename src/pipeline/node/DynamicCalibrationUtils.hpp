#pragma once

#include <array>
#include <cstdint>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/common/CameraModel.hpp>
#include <depthai/common/EepromData.hpp>
#include <depthai/common/HousingCoordinateSystem.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationControl.hpp>
#include <memory>
#include <variant>
#include <vector>

namespace cv {
class Mat;
}

namespace dcl {
class CameraCalibrationHandle;
struct ImageData;
enum class PerformanceMode : std::int32_t;
}  // namespace dcl

namespace dai {
namespace node {
struct DclUtils {
    static std::vector<std::vector<float>> calibrationHandleToTransform(const std::shared_ptr<const dcl::CameraCalibrationHandle>& calibration);

    static std::vector<CameraBoardSocket> buildSocketConnection(const EepromData& eepromData);

    static std::vector<std::vector<float>> computeBaseToSocketTransform(const CalibrationHandler& currentCalibration,
                                                                        const std::variant<CameraBoardSocket, HousingCoordinateSystem>& boardSocketBase,
                                                                        CameraBoardSocket boardSocket);

    static std::shared_ptr<dcl::CameraCalibrationHandle> convertDaiCalibrationToDcl(const CalibrationHandler& currentCalibration,
                                                                                    std::variant<CameraBoardSocket, HousingCoordinateSystem> boardSocketBase,
                                                                                    CameraBoardSocket boardSocket,
                                                                                    const std::vector<std::vector<float>>& intrinsicsOverride,
                                                                                    const std::vector<float>& distortionOverride,
                                                                                    const CameraModel distortionModelOverride);

    static void setHousingToDai(CalibrationHandler& calibHandler, const std::vector<std::vector<float>>& transformHousingToHousingOrigin);

    static void convertDclCalibrationToDai(CalibrationHandler& calibHandler,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle>& dclCalibrationA,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle>& dclCalibrationB,
                                           const std::variant<CameraBoardSocket, HousingCoordinateSystem> boardSocketBase,
                                           const CameraBoardSocket socketSrc,
                                           const CameraBoardSocket socketDest);

    static std::shared_ptr<dcl::CameraCalibrationHandle> createDclCalibration(const std::array<std::array<float, 3>, 3>& cameraMatrix,
                                                                              const std::vector<float>& distortionCoefficients,
                                                                              const std::vector<std::vector<float>>& rotationMatrix,
                                                                              const std::vector<float>& translationVector,
                                                                              const CameraModel distortionModel);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    static dcl::ImageData cvMatToImageData(const cv::Mat& mat);
#endif
    static dcl::PerformanceMode daiPerformanceModeToDclPerformanceMode(const dai::DynamicCalibrationControl::PerformanceMode mode);
};

}  // namespace node
}  // namespace dai
