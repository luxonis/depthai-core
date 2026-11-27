#pragma once

#include <DynamicCalibration.hpp>
#include <array>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/datatype/DynamicCalibrationResults.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include <deque>
#include <functional>

#include "utility/WorldConstraintsUtils.hpp"

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

    static dcl::ArucoDictionary toDclArucoDictionary(ArucoDictionary dictionary);  // Throws std::invalid_argument for a value outside the enumeration
    static dcl::MarkerSpec toDclMarkerSpec(const WorldConstraints& constraints);   // Throws std::invalid_argument for a marker type the library cannot detect

    static constexpr double kMinMarkerSizePx = 25.0;
    static constexpr size_t kMaxMarkerObservationSets = 50;
    static double markerSizePx(MarkerType type, const dcl::MarkerDetection& detection);  // 0 when the detection has no shape

    static dcl::Result<std::map<int32_t, dcl::MarkerDetection>> detectConfiguredMarkers(const dcl::ImageData& image,  // Drops too-small and repeated ids
                                                                                        const WorldConstraints& constraints,
                                                                                        std::map<int32_t, double>* tooSmall = nullptr);

    static MarkerOutline toMarkerOutline(const dcl::MarkerDetection& detection);

    struct WorldConstraintsSummary {
        size_t observationSets = 0;
        size_t points = 0;
        size_t pointDistances = 0;
        size_t sensorDistances = 0;
        size_t markerSizes = 0;
    };

    static dcl::WorldConstraints buildWorldConstraints(const WorldConstraints& constraints,  // nullptr from sensorForSocket skips that socket's constraints
                                                       const std::deque<MarkerObservationSet>& observationSets,
                                                       const std::function<std::shared_ptr<const dcl::CameraSensorHandle>(CameraBoardSocket)>& sensorForSocket,
                                                       WorldConstraintsSummary* summary = nullptr);

    static CoverageData::MarkerCoverage markerCoverage(const WorldConstraints& constraints,  // Accepted only when every required marker is seen by two cameras
                                                       const MarkerObservationSet& observations,
                                                       const std::map<int32_t, float>& tooSmallPx,
                                                       size_t observationSets);
};

}  // namespace node
}  // namespace dai
