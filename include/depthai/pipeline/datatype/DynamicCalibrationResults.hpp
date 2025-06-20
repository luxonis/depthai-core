#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/device/CalibrationHandler.hpp"

namespace dai {


/**
 * DynamicCalibrationResults message.
 */
struct DynamicCalibrationResults : public Buffer {
    /**
     * Construct DynamicCalibrationResults message.
     */
    DynamicCalibrationResults() = default;
    virtual ~DynamicCalibrationResults() = default;

    //TODO DCL: This is not needed as a separate struct
    struct CalibrationResult {
        std::optional<dai::CalibrationHandler> calibration;
        // TODO DCL:: don't use valid and info, use the optional functionality
        // IF valid: set it
        // if invalid: don't set it. From python side, it will be None
        bool valid = false;
        std::string info;

        static CalibrationResult Invalid(std::string reason = "No result") {
            return CalibrationResult{std::nullopt, false, std::move(reason)};
        }
        DEPTHAI_SERIALIZE(CalibrationResult, calibration, valid, info);
    };

    struct QualityResult {
        float value = -1.0f;
        // TODO: don't use valid and info, use the optional functionality
        bool valid = false;
        std::string info;

        static QualityResult Invalid(std::string reason = "No result") {
            return QualityResult{-1.0f, false, std::move(reason)};
        }
        DEPTHAI_SERIALIZE(QualityResult, value, valid, info);
    };

    QualityResult quality;
    // TODO DCL: This should be std::optional<dai::CalibrationHandler>
    CalibrationResult calibration;

    void reset() {
        quality = QualityResult::Invalid();
        calibration = CalibrationResult::Invalid();
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResults;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationResults, quality, calibration);
};

}  // namespace dai
