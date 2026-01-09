#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>

namespace dai {
namespace node {
struct DclUtils {
    static void convertDclCalibrationToDai(CalibrationHandler& calibHandler,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle>& dclCalibrationA,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle>& dclCalibrationB,
                                           const CameraBoardSocket socketSrc,
                                           const CameraBoardSocket socketDest,
                                           const std::pair<int, int>& resolutionA,
                                           const std::pair<int, int>& resolutionB);

    static std::shared_ptr<dcl::CameraCalibrationHandle> createDclCalibration(const std::vector<std::vector<float>>& cameraMatrix,
                                                                              const std::vector<float>& distortionCoefficients,
                                                                              const std::vector<std::vector<float>>& rotationMatrix,
                                                                              const std::vector<float>& translationVector,
                                                                              const CameraModel distortionModel);

    static std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> convertDaiCalibrationToDcl(
        const CalibrationHandler& currentCalibration,
        const CameraBoardSocket boardSocketA,
        const CameraBoardSocket boardSocketB,
        const std::pair<int, int>& resolutionA,
        const std::pair<int, int>& resolutionB);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    static dcl::ImageData cvMatToImageData(const cv::Mat& mat);
#endif
    static dcl::PerformanceMode daiPerformanceModeToDclPerformanceMode(const dai::DynamicCalibrationControl::PerformanceMode mode);
};

}  // namespace node
}  // namespace dai
