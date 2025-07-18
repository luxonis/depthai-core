#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * DynamicCalibrationConfig message.
 */
struct DynamicCalibrationConfig : public Buffer {
   public:
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationConfig;
    };

    enum class CalibrationCommand : int32_t {
        START_CALIBRATION_QUALITY_CHECK = 0,        ///< Start calibration quality check
        START_RECALIBRATION = 1,                    ///< Start recalibration
        START_FORCE_CALIBRATION_QUALITY_CHECK = 2,  ///< Start recalibration
        START_FORCE_RECALIBRATION = 3,              ///< Start recalibration
    };

    CalibrationCommand calibrationCommand;

    DynamicCalibrationConfig() = default;

    virtual ~DynamicCalibrationConfig() = default;

    DEPTHAI_SERIALIZE_EXT(DynamicCalibrationConfig, calibrationCommand);
};

}  // namespace dai
