#pragma once

#include <depthai/common/WorldConstraints.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dai {
namespace node {
namespace detail {

/// Intrinsics and distortion of the actual marker images, including any crop, resize or undistortion.
/// Camera poses always come from the assembled candidate calibration.
struct ObservationCamera {
    std::vector<std::vector<float>> intrinsics;
    std::vector<float> distortion;
    CameraModel distortionModel = CameraModel::Perspective;
};

/// A camera of the reconstruction with its pose: the 4x4 transform from the common base frame to the camera, in meters.
struct PosedObservationCamera {
    ObservationCamera model;
    std::vector<std::vector<float>> baseToCamera;
};

/// One configured measurement; measured is empty when the observations cannot reconstruct it.
struct ConstraintResidual {
    std::string what;
    double configured = 0.0;
    std::optional<double> measured;
    /// Requires a finite reconstruction within 15 percent or 1 cm of the configured measurement.
    bool fits() const;
};

/// True if either epipolar error is invalid, or the candidate exceeds both 0.5 px and twice the current error.
bool epipolarErrorContradictsConstraints(double currentPx, double newPx);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
/// Reconstructs every configured measurement using the final assembled calibration and the camera models
/// of the observed images. Returns one result per measurement, including those that cannot be reconstructed.
/// The base socket only selects the common reconstruction frame; socket distances use the candidate's poses.
std::vector<ConstraintResidual> worldConstraintResiduals(const WorldConstraints& constraints,
                                                         const std::deque<MarkerObservationSet>& observationSets,
                                                         const CalibrationHandler& calibration,
                                                         CameraBoardSocket base,
                                                         const std::map<CameraBoardSocket, ObservationCamera>& cameras);

/// The same reconstruction for cameras whose poses in a common base frame are given directly. The cameras are keyed
/// as in the observation sets; a socket distance to a key that is not among them is reported as not reconstructed.
std::vector<ConstraintResidual> worldConstraintResiduals(const WorldConstraints& constraints,
                                                         const std::deque<MarkerObservationSet>& observationSets,
                                                         const std::map<CameraBoardSocket, PosedObservationCamera>& cameras);
#endif

}  // namespace detail
}  // namespace node
}  // namespace dai
