#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

struct AutoCalibrationConfig : public Buffer {
    AutoCalibrationConfig() = default;
    virtual ~AutoCalibrationConfig();

    enum Mode : int {
        ON_START = 1,
        CONTINUOUS = 2,
    };

    Mode mode = Mode::ON_START;

    int sleepingTime = 30;  // seconds to sleep between iterations in continuous mode

    double calibrationConfidenceThreshold = 0.9;

    double dataConfidenceThreshold = 0.7;

    unsigned int maxIterations = 10;

    int validationSetSize = 5;

    bool flashCalibration = false;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(
        AutoCalibrationConfig, mode, sleepingTime, calibrationConfidenceThreshold, dataConfidenceThreshold, maxIterations, validationSetSize, flashCalibration);
};

}  // namespace dai
