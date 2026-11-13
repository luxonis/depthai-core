#pragma once
#include <optional>

#include "depthai/common/WorldConstraints.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    bool emptyBool;

    /**
     * Fiducial markers in the scene with measured distances, giving the calibration its metric scale. See
     * WorldConstraints.
     */
    std::optional<WorldConstraints> worldConstraints;

    ~DynamicCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT_DECL(DynamicCalibrationProperties, emptyBool, worldConstraints);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::DynamicCalibrationProperties>;
