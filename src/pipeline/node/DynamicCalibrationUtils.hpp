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

    /// The DCL enumerator of a depthai ArucoDictionary. Throws std::invalid_argument for a value outside the enumeration.
    static dcl::ArucoDictionary toDclArucoDictionary(ArucoDictionary dictionary);
    /// The marker detector request for a constraint set. Throws std::invalid_argument for a marker type the library cannot detect.
    static dcl::MarkerSpec toDclMarkerSpec(const WorldConstraints& constraints);

    /// Smallest marker the calibration uses, in pixels of the input image: the shortest side of an ArUco marker or
    /// the smallest diameter of a Luxonis Marker ring. Smaller markers give too coarse a center and size.
    static constexpr double kMinMarkerSizePx = 25.0;
    /// Synchronized frame sets with marker observations kept for the calibration; the oldest set is dropped first.
    static constexpr size_t kMaxMarkerObservationSets = 50;
    /// Size of a detection as compared with kMinMarkerSizePx; 0 when the detection has no shape.
    static double markerSizePx(MarkerType type, const dcl::MarkerDetection& detection);

    /// The configured markers found in one image, keyed by marker id. Markers outside the configuration, smaller than
    /// kMinMarkerSizePx and ids that appear more than once in the image are dropped. tooSmall, when given, receives
    /// the configured markers dropped for their size together with that size in pixels. Fails when the detector
    /// rejects the image.
    static dcl::Result<std::map<int32_t, dcl::MarkerDetection>> detectConfiguredMarkers(const dcl::ImageData& image,
                                                                                        const WorldConstraints& constraints,
                                                                                        std::map<int32_t, double>* tooSmall = nullptr);

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

    /**
     * Marker status of one frame set: which of the markers the constraints refer to were seen by at least two
     * cameras (observations), which were too small (tooSmallPx, marker id to largest size seen), and whether the
     * set is accepted, which needs every required marker seen by two cameras. observationSets is the number of
     * sets kept so far and is copied into the result.
     */
    static CoverageData::MarkerCoverage markerCoverage(const WorldConstraints& constraints,
                                                       const MarkerObservationSet& observations,
                                                       const std::map<int32_t, float>& tooSmallPx,
                                                       size_t observationSets);

    /// A camera as the consistency check of the world constraints needs it: intrinsics in pixels of the observed
    /// image, and the pose that takes a point of the base frame, in meters, into the camera frame.
    struct MetricCamera {
        std::vector<std::vector<float>> intrinsics;
        std::vector<float> distortion;
        CameraModel distortionModel = CameraModel::Perspective;
        std::vector<std::vector<float>> baseToCamera;
    };
    /// One configured measurement next to what the cameras reconstruct for it, both in meters.
    struct ConstraintResidual {
        std::string what;
        double configured = 0.0;
        double measured = 0.0;
        /// |measured - configured| / configured
        double relativeError() const;
        /// False when the two values are further apart than both kMaxConstraintRelativeError and kMaxConstraintError.
        bool fits() const;
    };
    /// A measurement that the calibrated cameras reconstruct further than this from its configured value does not
    /// describe the scene: a wrong number, unit or marker id. The limit is the larger of a fraction of the
    /// configured value and a length in meters, which keeps the noise of short distances and small markers out.
    /// Correct constraints stay within about 6 % on a 7.5 cm baseline at 1-2 m.
    static constexpr double kMaxConstraintRelativeError = 0.15;
    static constexpr double kMaxConstraintError = 0.01;
    /// Applying the constraints must not push the epipolar error of the image features above this many pixels and
    /// above kMaxConstrainedEpipolarErrorGrowth times the error of the calibration in use.
    static constexpr double kMaxConstrainedEpipolarErrorPx = 0.5;
    static constexpr double kMaxConstrainedEpipolarErrorGrowth = 2.0;
    /// True when the epipolar error of a calibration refined by world constraints says they contradict the images.
    static bool epipolarErrorContradictsConstraints(double currentPx, double newPx);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * Compares every configured measurement with the scene the cameras reconstruct from the collected marker
     * observations: marker points are triangulated from all cameras that saw them, a marker distance is the distance
     * between two points, a socket distance the distance from a point to the camera center, and a marker size is
     * taken from the triangulated corners (ARUCO) or the angular size of the ring at the triangulated range
     * (LUXONIS). The reported value is the median over the frame sets. Measurements that no frame set lets the
     * cameras reconstruct are left out. cameras holds every socket the constraints or observations refer to.
     */
    static std::vector<ConstraintResidual> worldConstraintResiduals(const WorldConstraints& constraints,
                                                                    const std::deque<MarkerObservationSet>& observationSets,
                                                                    const std::map<CameraBoardSocket, MetricCamera>& cameras);
#endif
};

}  // namespace node
}  // namespace dai
