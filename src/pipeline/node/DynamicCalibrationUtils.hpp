#pragma once

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
    /// The marker detector request for a constraint set. Throws std::invalid_argument for a marker type the library cannot detect.
    static dcl::MarkerSpec toDclMarkerSpec(const WorldConstraints& constraints);

    /// Smallest marker the calibration uses, in pixels of the input image: the shortest side of an ArUco marker or
    /// the smallest diameter of a Luxonis Marker ring. Smaller markers give too coarse a center and size.
    static constexpr double kMinMarkerSizePx = 25.0;
    /// Size of a detection as compared with kMinMarkerSizePx; 0 when the detection has no shape.
    static double markerSizePx(MarkerType type, const dcl::MarkerDetection& detection);

    /// The configured markers found in one image, keyed by marker id. Markers outside the configuration, smaller than
    /// kMinMarkerSizePx and ids that appear more than once in the image are dropped. Fails when the detector rejects
    /// the image.
    static dcl::Result<std::map<int32_t, dcl::MarkerDetection>> detectConfiguredMarkers(const dcl::ImageData& image, const WorldConstraints& constraints);

    /// The shape of a detection, as kept in a MarkerObservationSet.
    static MarkerOutline toMarkerOutline(const dcl::MarkerDetection& detection);

    struct WorldConstraintsSummary {
        size_t observationSets = 0;
        size_t points = 0;
        size_t pointDistances = 0;
        size_t sensorDistances = 0;
        size_t markerSizes = 0;
    };

    /**
     * Turns the collected marker centers into DCL world constraints. Every frame set contributes one scene
     * point per marker that at least two sensors saw, the configured marker distances between the points of
     * the same frame set, the configured socket distances for points of that frame set, and one size observation
     * per sensor that saw a point's marker when that marker has a configured size. sensorForSocket returns the DCL
     * sensor of a socket, or nullptr for sockets the node does not know, whose constraints are skipped.
     */
    static dcl::WorldConstraints buildWorldConstraints(const WorldConstraints& constraints,
                                                       const std::deque<MarkerObservationSet>& observationSets,
                                                       const std::function<std::shared_ptr<const dcl::CameraSensorHandle>(CameraBoardSocket)>& sensorForSocket,
                                                       WorldConstraintsSummary* summary = nullptr);
};

}  // namespace node
}  // namespace dai
