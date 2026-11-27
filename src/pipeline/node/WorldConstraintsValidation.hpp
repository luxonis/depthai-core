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

struct ObservationCamera {  // Intrinsics of the marker images as observed (after crop, resize, undistortion)
    std::vector<std::vector<float>> intrinsics;
    std::vector<float> distortion;
    CameraModel distortionModel = CameraModel::Perspective;
};

struct ConstraintResidual {  // measured is empty when the observations cannot reconstruct it
    std::string what;
    double configured = 0.0;
    std::optional<double> measured;
    bool fits() const;  // Finite and within 15 percent or 1 cm of the configured value
};

bool epipolarErrorContradictsConstraints(double currentPx, double newPx);  // True if an error is invalid, or newPx > 0.5 px and > 2 * currentPx

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
std::vector<ConstraintResidual> worldConstraintResiduals(const WorldConstraints& constraints,  // One result per measurement, also for ones not reconstructable
                                                         const std::deque<MarkerObservationSet>& observationSets,
                                                         const CalibrationHandler& calibration,
                                                         CameraBoardSocket base,
                                                         const std::map<CameraBoardSocket, ObservationCamera>& cameras);
#endif

}  // namespace detail
}  // namespace node
}  // namespace dai
