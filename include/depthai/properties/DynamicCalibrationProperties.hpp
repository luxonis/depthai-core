#pragma once
#include <optional>

#include "depthai/common/SceneMarkerConstraints.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    bool emptyBool;

    /**
     * Fiducial markers in the scene with measured distances, giving the calibration its metric scale. See
     * SceneMarkerConstraints.
     */
    std::optional<SceneMarkerConstraints> sceneConstraints;

    ~DynamicCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, emptyBool, sceneConstraints);

}  // namespace dai
