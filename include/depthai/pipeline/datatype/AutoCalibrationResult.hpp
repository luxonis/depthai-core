#pragma once

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * AutoCalibrationResult message.
 * Carries the result and quality metrics produced by the AutoCalibration node.
 */
class AutoCalibrationResult : public Buffer {
   public:
    AutoCalibrationResult() = default;

    /**
     * Full constructor for AutoCalibrationResult.
     * @param dataConfidence Confidence score of the input data used for calibration.
     * @param calibrationConfidence Confidence score of the resulting calibration.
     * @param passed Whether the calibration cycle met the required thresholds.
     * @param calibration The actual calibration handler containing the new parameters.
     */
    AutoCalibrationResult(double dataConfidence, double calibrationConfidence, bool passed, CalibrationHandler calibration)
        : dataConfidence(dataConfidence), calibrationConfidence(calibrationConfidence), passed(passed), calibration(calibration){};

    virtual ~AutoCalibrationResult();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * @brief Quality of input features used (0.0 to 1.0).
     */
    double dataConfidence = 0.0;

    /**
     * @brief Confidence in the final calibration result (0.0 to 1.0).
     */
    double calibrationConfidence = 0.0;

    /**
     * @brief True if thresholds were met and calibration is valid.
     */
    bool passed = false;

    /**
     * @brief CalibrationHandler containing the resulting camera parameters.
     */
    CalibrationHandler calibration;

    // clang-format off
    DEPTHAI_SERIALIZE(AutoCalibrationResult,
                      dataConfidence,
                      calibrationConfidence,
                      passed,
                      calibration);
    // clang-format on
};

}  // namespace dai
