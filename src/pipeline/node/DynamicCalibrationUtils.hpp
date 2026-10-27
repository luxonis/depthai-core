#pragma once

#include <ArucoMarkers.hpp>
#include <DynamicCalibration.hpp>
#include <array>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Sync.hpp>
#include <depthai/properties/DynamicCalibrationProperties.hpp>
#include <deque>
#include <functional>

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

    static dcl::ArucoDictionary toDclArucoDictionary(ArucoDictionary dictionary);

    /// Centers of the configured markers in one image, keyed by marker id. Markers outside the configuration are dropped.
    static std::map<int32_t, std::array<float, 2>> detectMarkerCenters(const dcl::ImageData& image, const ArucoMarkerConstraints& constraints);

    struct WorldConstraintsSummary {
        size_t observationSets = 0;
        size_t points = 0;
        size_t pointDistances = 0;
        size_t sensorDistances = 0;
    };

    /**
     * Turns the collected marker centers into DCL world constraints. Every frame set contributes one scene
     * point per marker that at least two sensors saw, the configured marker distances between the points of
     * the same frame set, and the configured socket distances for points of that frame set. sensorForSocket
     * returns the DCL sensor of a socket, or nullptr for sockets the node does not know, whose constraints are
     * skipped.
     */
    static dcl::WorldConstraints buildWorldConstraints(const ArucoMarkerConstraints& constraints,
                                                       const std::deque<ArucoMarkerObservationSet>& observationSets,
                                                       const std::function<std::shared_ptr<const dcl::CameraSensorHandle>(CameraBoardSocket)>& sensorForSocket,
                                                       WorldConstraintsSummary* summary = nullptr);
};

}  // namespace node
}  // namespace dai
