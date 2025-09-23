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
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle> dclCalibrationA,
                                           const std::shared_ptr<const dcl::CameraCalibrationHandle> dclCalibrationB,
                                           const CameraBoardSocket socketSrc,
                                           const CameraBoardSocket socketDest,
                                           const int width,
                                           const int height);

    static std::shared_ptr<dcl::CameraCalibrationHandle> createDclCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                              const std::vector<float> distortionCoefficients,
                                                                              const std::vector<std::vector<float>> rotationMatrix,
                                                                              const std::vector<float> translationVector);

    static std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> convertDaiCalibrationToDcl(
        const CalibrationHandler& currentCalibration,
        const CameraBoardSocket boardSocketA,
        const CameraBoardSocket boardSocketB,
        const int width,
        const int height);

    static dcl::ImageData cvMatToImageData(const cv::Mat& mat);
};

}  // namespace node
}  // namespace dai
