#pragma once

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
struct DynamicCalibrationWorkerResult : public Buffer {
    DynamicCalibrationWorkerResult() = default;
    DynamicCalibrationWorkerResult(double dataQuality, double calibrationConfidence, bool passed, CalibrationHandler calibration)
        : dataQuality(dataQuality), calibrationConfidence(calibrationConfidence), passed(passed), calibration(calibration) {};
    virtual ~DynamicCalibrationWorkerResult();
    double dataQuality;
    double calibrationConfidence;
    bool passed;
    CalibrationHandler calibration;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
};

}  // namespace dai
