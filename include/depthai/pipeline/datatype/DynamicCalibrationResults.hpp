#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include <DynamicCalibration.hpp>

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
        std::optional<dai::CalibrationHandler> calibHandler;

        static CalibrationResult Invalid() {
            return CalibrationResult{std::nullopt};
        }

        DEPTHAI_SERIALIZE(CalibrationResult, calibHandler);
    };

    /**
     * CalibrationQuality is result which is returned after dai::DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK
     * It contains information about:
     * - CoverageData: 2D distribution of data over image
     * - CalibrationData: information about theoretical predictions how would the new recalibrated calibration look like.
     */
    struct CalibrationQuality
        {
        /**
        *
        * It contains information about 2D distribution of data over image. It is created per frame:
        * - coveragePerCell; tells the 2D spatial distribution, set by the DCL itself
        *    It is a matrix, which presents how data is distributed on the 2D image
        *    Values in matrix presents overall fullness of the bin; [0, 1], with 0 being worst, 1 best.
        * - meanCoverage; tells overall Quality of 2D distribution, combined from both images.
        */
        struct CoverageData
        {
            std::vector<std::vector<float>> coveragePerCellA;
            std::vector<std::vector<float>> coveragePerCellB;
            float meanCoverage;
            DEPTHAI_SERIALIZE(CoverageData, coveragePerCellA, coveragePerCellB, meanCoverage);
        };

        /**
         *
         * It contains information about how would new calibration will affect current state of device. It is created after succesful START_CALIBRATION_QUALITY_CHECK:
         * - rotationChange: difference in rotation angles in extrinsics matrix with old and new calibration. In case angle difference is constantly over some threshold
         *   it would mean, device calibration has been afected and should be good to recalibrate the device.
         *   UNITS [deg].
         * - depthErrorDifference: tehoretical prediction of relative depth difference between the old and new calibration. It includes the values from [1m, 2m, 5m, 10m].
         *   in case that difference is very high for all presented distances, it would mean, that recalibration is required.
         *   UNITS [%]
         */
        struct CalibrationData
        {
            std::array<float, 3> rotationChange  = {0.0f, 0.0f, 0.0f};;
            float epipolarErrorChange;
            std::vector<float> depthErrorDifference;
            DEPTHAI_SERIALIZE(CalibrationData, rotationChange, epipolarErrorChange, depthErrorDifference);
        };

        float dataAcquired  = 0.0f;
        std::optional<CoverageData> coverageQuality;
        std::optional<CalibrationData> calibrationQuality; // <--- optional



        DEPTHAI_SERIALIZE(CalibrationQuality, coverageQuality, calibrationQuality);
    };

    struct CalibrationQualityResult {

        std::optional<CalibrationQuality> report;

        static CalibrationQualityResult Invalid() {
            return CalibrationQualityResult{std::nullopt};
        }

        DEPTHAI_SERIALIZE(CalibrationQualityResult, report);
    };

    // TODO DCL: This should be std::optional<dai::CalibrationHandler>
    std::optional<CalibrationResult> newCalibration;
    std::optional<CalibrationQualityResult> calibOverallQuality;
    std::string info = "";

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResults;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationResults, calibOverallQuality, newCalibration, info);
};

}  // namespace dai
