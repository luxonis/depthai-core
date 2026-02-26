#pragma once

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
struct AutoCalibrationResult : public Buffer {
    AutoCalibrationResult() = default;
    AutoCalibrationResult(double dataQuality, double calibrationConfidence, bool passed, CalibrationHandler calibration)
        : dataQuality(dataQuality), calibrationConfidence(calibrationConfidence), passed(passed), calibration(calibration) {};
    virtual ~AutoCalibrationResult();

    double dataQuality;
    double calibrationConfidence;
    bool passed;
    CalibrationHandler calibration;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
};

}  // namespace dai
