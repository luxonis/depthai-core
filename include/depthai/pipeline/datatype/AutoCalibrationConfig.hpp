#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

struct AutoCalibrationConfig : public Buffer {
    enum Mode : int {
        ON_START = 1,
        CONTINUOUS = 2,
    };

    AutoCalibrationConfig() = default;

    AutoCalibrationConfig(Mode mode, int sleepingTime, double calConf, double dataConf, unsigned int maxIter, unsigned int maxImg, int valSize, bool flash)
        : mode(mode),
          sleepingTime(sleepingTime),
          calibrationConfidenceThreshold(calConf),
          dataConfidenceThreshold(dataConf),
          maxIterations(maxIter),
          maxImagesPerReacalibration(maxImg),
          validationSetSize(valSize),
          flashCalibration(flash) {}

    virtual ~AutoCalibrationConfig();

    Mode mode = Mode::ON_START;

    int sleepingTime = 30;  // seconds to sleep between iterations in continuous mode

    double calibrationConfidenceThreshold = 0.9;

    double dataConfidenceThreshold = 0.7;

    unsigned int maxIterations = 10;

    unsigned int maxImagesPerReacalibration = 20;

    int validationSetSize = 5;

    bool flashCalibration = true;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

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
