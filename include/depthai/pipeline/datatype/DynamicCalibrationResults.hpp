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
        // TODO DCL:: don't use valid and info, use the optional functionality
        // IF valid: set it
        // if invalid: don't set it. From python side, it will be None

        static CalibrationResult Invalid() {
            return CalibrationResult{std::nullopt};
        }

        DEPTHAI_SERIALIZE(CalibrationResult, calibHandler);
    };
    struct CalibrationData
    {
        std::vector<float> rotationChange;
        float epipolarErrorChange;
        std::vector<float> depthAccuracy;
        DEPTHAI_SERIALIZE(CalibrationData, rotationChange, epipolarErrorChange, depthAccuracy);
    };

    struct CoverageData
    {
        std::vector<std::vector<float>> coveragePerCellA;
        std::vector<std::vector<float>> coveragePerCellB;
        float meanCoverage;
        DEPTHAI_SERIALIZE(CoverageData, coveragePerCellA, coveragePerCellB, meanCoverage);
    };

    struct CalibrationQuality
    {
        std::optional<CoverageData> coverageQuality;
        std::optional<CalibrationData> calibrationQuality; // <--- optional


        static CalibrationQuality fromDCL(const dcl::CalibrationQuality& src) {
            CalibrationQuality out;

            if(src.coverageQuality.has_value()) {
                CoverageData cov;
                cov.coveragePerCellA = src.coverageQuality->coveragePerCellA;
                cov.coveragePerCellB = src.coverageQuality->coveragePerCellB;
                cov.meanCoverage = src.coverageQuality->meanCoverage;
                out.coverageQuality = cov;
            }

            if(src.calibrationQuality.has_value()) {
                CalibrationData cal;
                cal.rotationChange = src.calibrationQuality->rotationChange;
                cal.epipolarErrorChange = src.calibrationQuality->epipolarErrorChange;
                   cal.depthAccuracy = src.calibrationQuality->depthAccuracy;
                out.calibrationQuality = cal;
            }
            return out;
        };

        DEPTHAI_SERIALIZE(CalibrationQuality, coverageQuality, calibrationQuality);
    };
    struct CalibrationQualityResult {

        std::optional<CalibrationQuality> report;

        static CalibrationQualityResult fromDCL(const dcl::CalibrationQuality& src) {
            CalibrationQualityResult out;
            out.report = CalibrationQuality::fromDCL(src);
            return out;
        }
        static CalibrationQualityResult Invalid() {
            return CalibrationQualityResult{std::nullopt};
        }

        DEPTHAI_SERIALIZE(CalibrationQualityResult, report);
    };

    // TODO DCL: This should be std::optional<dai::CalibrationHandler>
    std::optional<CalibrationResult> newCalibration;
    std::optional<CalibrationQualityResult> calibOverallQuality;

    void reset() {
        calibOverallQuality = CalibrationQualityResult::Invalid();
        newCalibration = CalibrationResult::Invalid();
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResults;
    };
    DEPTHAI_SERIALIZE(DynamicCalibrationResults, calibOverallQuality, newCalibration);
};

}  // namespace dai
