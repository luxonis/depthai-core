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
    /**
     * @brief Get the data confidence score
     * @return Quality of input features used (0.0 to 1.0)
     */
    double getDataConfidence() const {
        return dataConfidence;
    }

    /**
     * @brief Set the data confidence score
     * @param value Data confidence to set
     */
    void setDataConfidence(double value) {
        dataConfidence = value;
    }

    /**
     * @brief Get the calibration confidence score
     * @return Confidence in the final calibration result (0.0 to 1.0)
     */
    double getCalibrationConfidence() const {
        return calibrationConfidence;
    }

    /**
     * @brief Set the calibration confidence score
     * @param value Calibration confidence to set
     */
    void setCalibrationConfidence(double value) {
        calibrationConfidence = value;
    }

    /**
     * @brief Check if the calibration process was successful
     * @return True if thresholds were met and calibration is valid
     */
    bool getPassed() const {
        return passed;
    }

    /**
     * @brief Set the success status of the calibration
     * @param value Success status to set
     */
    void setPassed(bool value) {
        passed = value;
    }

    /**
     * @brief Get the calibration handler
     * @return CalibrationHandler containing camera parameters
     */
    CalibrationHandler getCalibration() const {
        return calibration;
    }

    /**
     * @brief Set the calibration handler
     * @param value Calibration data to set
     */
    void setCalibration(CalibrationHandler value) {
        calibration = value;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    // Member variables
    double dataConfidence = 0.0;
    double calibrationConfidence = 0.0;
    bool passed = false;
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
