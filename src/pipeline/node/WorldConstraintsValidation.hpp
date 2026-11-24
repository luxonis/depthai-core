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
#endif

}  // namespace detail
}  // namespace node
}  // namespace dai
