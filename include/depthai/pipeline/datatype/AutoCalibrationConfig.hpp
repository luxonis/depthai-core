#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * AutoCalibrationConfig message. Carries configuration for the automatic camera calibration algorithm.
 * Defines parameters for periodic or one-time recalibration of the camera sensors.
 */
struct AutoCalibrationConfig : public Buffer {
   public:
    /**
     * Defines when the auto-calibration process should be triggered.
     */
    enum Mode : int {
        /** Run calibration once upon pipeline startup. */
        ON_START = 1,
        /** Periodically run calibration while the pipeline is running. */
        CONTINUOUS = 2,
    };

    AutoCalibrationConfig() = default;

    /**
     * Full constructor for initializing auto-calibration parameters.
     * @param mode Calibration trigger mode (ON_START or CONTINUOUS).
     * @param sleepingTime Time in seconds between calibration cycles.
     * @param calibrationConfidence Threshold for accepting a new calibration result.
     * @param dataConfidence Threshold for the quality of input data/features.
     * @param maxIter Maximum number of optimization iterations.
     * @param maxImg Maximum number of images to collect for a single recalibration.
     * @param valSize Number of images used to validate the calibration result.
     * @param flash Whether to save the resulting calibration to non-volatile memory.
     */
    AutoCalibrationConfig(
        Mode mode, int sleepingTime, double calibrationConfidence, double dataConfidence, unsigned int maxIter, unsigned int maxImg, int valSize, bool flash)
        : mode(mode),
          sleepingTime(sleepingTime),
          calibrationConfidenceThreshold(calibrationConfidence),
          dataConfidenceThreshold(dataConfidence),
          maxIterations(maxIter),
          maxImagesPerReacalibration(maxImg),
          validationSetSize(valSize),
          flashCalibration(flash) {}

    virtual ~AutoCalibrationConfig();

    /**
     * @brief Get the current auto-calibration mode
     * @return Calibration trigger mode (ON_START or CONTINUOUS)
     */
    Mode getMode() const {
        return mode;
    }

    /**
     * @brief Set the auto-calibration trigger mode
     * @param value Trigger mode to set
     */
    void setMode(Mode value) {
        mode = value;
    }

    /**
     * @brief Get sleep time between recalibration attempts in CONTINUOUS mode
     * @return Time in seconds
     */
    int getSleepingTime() const {
        return sleepingTime;
    }

    /**
     * @brief Set sleep time between recalibration attempts in CONTINUOUS mode
     * @param value Seconds to sleep between calibration cycles
     */
    void setSleepingTime(int value) {
        sleepingTime = value;
    }

    /**
     * @brief Get the calibration confidence threshold
     * @return Confidence threshold (0.0 to 1.0)
     */
    double getCalibrationConfidenceThreshold() const {
        return calibrationConfidenceThreshold;
    }

    /**
     * @brief Set the calibration confidence threshold
     * @param value Minimum confidence score required to apply a new calibration
     */
    void setCalibrationConfidenceThreshold(double value) {
        calibrationConfidenceThreshold = value;
    }

    /**
     * @brief Get the data confidence threshold
     * @return Data quality threshold (0.0 to 1.0)
     */
    double getDataConfidenceThreshold() const {
        return dataConfidenceThreshold;
    }

    /**
     * @brief Set the data confidence threshold
     * @param value Minimum quality threshold for input features to be used
     */
    void setDataConfidenceThreshold(double value) {
        dataConfidenceThreshold = value;
    }

    /**
     * @brief Get maximum number of optimization iterations
     * @return Max iterations
     */
    unsigned int getMaxIterations() const {
        return maxIterations;
    }

    /**
     * @brief Set maximum number of optimization iterations
     * @param value Max number of iterations per calibration cycle
     */
    void setMaxIterations(unsigned int value) {
        maxIterations = value;
    }

    /**
     * @brief Get max images per recalibration
     * @return Number of images
     */
    unsigned int getMaxImagesPerReacalibration() const {
        return maxImagesPerReacalibration;
    }

    /**
     * @brief Set maximum images to collect for one recalibration event
     * @param value Max image count
     */
    void setMaxImagesPerReacalibration(unsigned int value) {
        maxImagesPerReacalibration = value;
    }

    /**
     * @brief Get validation set size
     * @return Number of validation images
     */
    int getValidationSetSize() const {
        return validationSetSize;
    }

    /**
     * @brief Set number of images used for validating calibration result
     * @param value Validation set size
     */
    void setValidationSetSize(int value) {
        validationSetSize = value;
    }

    /**
     * @brief Check if results are written to flash
     * @return True if flash storage is enabled
     */
    bool getFlashCalibration() const {
        return flashCalibration;
    }

    /**
     * @brief Enable or disable writing successful calibration to non-volatile memory
     * @param value True to save to EPROM, false to keep in RAM only
     */
    void setFlashCalibration(bool value) {
        flashCalibration = value;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    Mode mode = Mode::ON_START;
    int sleepingTime = 30;
    double calibrationConfidenceThreshold = 0.9;
    double dataConfidenceThreshold = 0.7;
    unsigned int maxIterations = 10;
    unsigned int maxImagesPerReacalibration = 20;
    int validationSetSize = 5;
    bool flashCalibration = true;

    DEPTHAI_SERIALIZE(AutoCalibrationConfig,
                      mode,
                      sleepingTime,
                      calibrationConfidenceThreshold,
                      dataConfidenceThreshold,
                      maxIterations,
                      maxImagesPerReacalibration,
                      validationSetSize,
                      flashCalibration);
};

}  // namespace dai
