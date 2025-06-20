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

    struct CalibrationResult {
        std::optional<dai::EepromData> eepromData;  ///< Eeprom data containing calibration results
        bool valid = false;
        std::string info;

        dai::CalibrationHandler getCalibration() const {
            if(eepromData) {
                return dai::CalibrationHandler(*eepromData);
            } else {
                throw std::runtime_error("No calibration data available");
            }
        }
        void setCalibration(const dai::CalibrationHandler& calibration) {
            eepromData = calibration.getEepromData();
        }

        static CalibrationResult Invalid(std::string reason = "No result") {
            return CalibrationResult{std::nullopt, false, std::move(reason)};
        }
        DEPTHAI_SERIALIZE(CalibrationResult, eepromData, valid, info);
    };

    struct QualityResult {
        float value = -1.0f;
        bool valid = false;
        std::string info;

        static QualityResult Invalid(std::string reason = "No result") {
            return QualityResult{-1.0f, false, std::move(reason)};
        }
        DEPTHAI_SERIALIZE(QualityResult, value, valid, info);
    };

    QualityResult quality;
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
