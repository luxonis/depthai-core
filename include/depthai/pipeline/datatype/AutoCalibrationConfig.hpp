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
          maxImagesPerRecalibration(maxImg),
          validationSetSize(valSize),
          flashCalibration(flash) {}

    virtual ~AutoCalibrationConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::AutoCalibrationConfig;
    }

    /**
     * @brief Calibration trigger mode (ON_START or CONTINUOUS).
     */
    Mode mode = Mode::ON_START;

    /**
     * @brief Seconds to sleep between calibration cycles in CONTINUOUS mode.
     */
    int sleepingTime = 30;

    /**
     * @brief Minimum confidence score (0.0 to 1.0) required to apply a new calibration.
     */
    double calibrationConfidenceThreshold = 0.9;

    /**
     * @brief Minimum quality threshold (0.0 to 1.0) for input features to be used.
     */
    double dataConfidenceThreshold = 0.7;

    /**
     * @brief Maximum number of optimization iterations per calibration cycle.
     */
    unsigned int maxIterations = 5;

    /**
     * @brief Maximum number of images to collect for one recalibration event.
     */
    unsigned int maxImagesPerRecalibration = 10;

    /**
     * @brief Number of images used for validating the calibration result.
     */
    int validationSetSize = 5;

    /**
     * @brief If true, saves successful calibration to non-volatile memory (EEPROM); otherwise, keeps in RAM only.
     */
    bool flashCalibration = true;

    // clang-format off
    DEPTHAI_SERIALIZE(AutoCalibrationConfig,
                      mode,
                      sleepingTime,
                      calibrationConfidenceThreshold,
                      dataConfidenceThreshold,
                      maxIterations,
                      maxImagesPerRecalibration,
                      validationSetSize,
                      flashCalibration);
    // clang-format on
};

}  // namespace dai
