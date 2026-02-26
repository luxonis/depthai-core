#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

struct DynamicCalibrationWorkerConfig : public Buffer {
    DynamicCalibrationWorkerConfig() = default;
    virtual ~DynamicCalibrationWorkerConfig();

    enum Mode : int {
        ON_START = 1,
        CONTINUOUS = 2,
    };

    Mode mode = Mode::ON_START;

    int sleepingTime = 30;  // seconds to sleep between iterations in continuous mode

    double calibrationConfidenceThreshold = 0.9;

    double dataQualityThreshold = 0.7;

    unsigned int maxIterations = 10;

    int validationSetSize = 5;

    bool flashCalibration = false;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(DynamicCalibrationWorkerConfig,
                      mode,
                      sleepingTime,
                      calibrationConfidenceThreshold,
                      dataConfidenceThreshold,
                      maxIterations,
                      validationSetSize,
                      flashCalibration);
};

}  // namespace dai
