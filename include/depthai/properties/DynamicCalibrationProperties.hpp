#pragma once
#include <optional>

#include "depthai/common/ArucoMarkerConstraints.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    bool emptyBool;

    /**
     * ArUco markers in the scene with measured distances, giving the calibration its metric scale. See
     * ArucoMarkerConstraints.
     */
    std::optional<ArucoMarkerConstraints> arucoConstraints;

    ~DynamicCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, emptyBool, arucoConstraints);

}  // namespace dai
