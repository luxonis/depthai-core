#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * CoverageData message
 *
 * Contains information about the 2D spatial distribution of calibration data
 * across the image pair. Generated per frame by the DCL.
 */
struct CoverageData : public Buffer {
    // clang-format off
    CoverageData(const dcl::CoverageData& cd)
      : coveragePerCellA(cd.coveragePerCellA)
      , coveragePerCellB(cd.coveragePerCellB)
      , meanCoverage(cd.meanCoverage)
      , coverageAcquired(cd.coverageAcquired)
      , dataAcquired(cd.dataAcquired)
    {}
    // clang-format on
    CoverageData() = default;
    virtual ~CoverageData() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::CoverageData;
    }

    /**
     * 2D coverage matrix for input A (e.g. left image).
     * Each cell represents how well that spatial bin is populated; range [0, 1].
     */
    std::vector<std::vector<float>> coveragePerCellA;

    /**
     * 2D coverage matrix for input B (e.g. right image).
     * Each cell represents how well that spatial bin is populated; range [0, 1].
     */
    std::vector<std::vector<float>> coveragePerCellB;

    /**
     * Overall quality metric summarizing 2D coverage across both inputs.
     * Typically normalized to [0, 1].
     */
    float meanCoverage;

    /**
     * Proportion of the desired spatial coverage achieved so far; range [0, 1].
     */
    float coverageAcquired = 0.0f;

    /**
     * Proportion of calibration-relevant data acquired from the frame; range [0, 1].
     */
    float dataAcquired = 0.0f;

    DEPTHAI_SERIALIZE(CoverageData, coveragePerCellA, coveragePerCellB, meanCoverage, dataAcquired, coverageAcquired);
};

/**
 * CalibrationQuality message
 *
 * Returned after running a calibration quality check.
 * Provides feedback on how a potential recalibration would affect the
 * device, including rotation changes, predicted depth accuracy, and
 * epipolar error metrics.
 */

struct CalibrationQuality : public Buffer {
    /**
     * Quality metrics for a proposed calibration.
     *
     * Includes rotation differences, predicted depth error changes,
     * and Sampson error comparison between current and new (achievable) calibration.
     */
    struct Data {
        /**
         * Difference in rotation angles (extrinsics) between current and new calibration.
         * Units: degrees [deg].
         */
        std::array<float, 3> rotationChange;

        /**
         * Predicted relative depth error difference between current and new calibration.
         * Reported at reference distances [1m, 2m, 5m, 10m].
         * Units: percent [%].
         */
        std::vector<float> depthErrorDifference;

        /**
         * Current calibration Sampson error.
         */
        float sampsonErrorCurrent;

        /**
         * Estimated new (achievable) Sampson error with recalibration applied.
         */
        float sampsonErrorNew;

        DEPTHAI_SERIALIZE(Data, rotationChange, sampsonErrorCurrent, sampsonErrorNew, depthErrorDifference);
    };

    /**
     * Construct empty CalibrationQuality message.
     */
    CalibrationQuality() = default;
    virtual ~CalibrationQuality() = default;

    /**
     * Construct CalibrationQuality with quality metrics and info string.
     */
    CalibrationQuality(Data data, std::string info) : data(std::make_optional(std::move(data))), info(std::move(info)) {}

    /**
     * Construct CalibrationQuality with only info string (no quality metrics).
     */
    CalibrationQuality(std::string info) : data(std::nullopt), info(std::move(info)) {}

    /**
     * Optional quality metrics data.
     * May be missing if the quality check did not produce valid results.
     */
    std::optional<Data> data;

    /**
     * Informational message describing the outcome of the quality check.
     */
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::CalibrationQuality;
    }

    DEPTHAI_SERIALIZE(CalibrationQuality, data, info);
};

/**
 * DynamicCalibrationResult message
 *
 * Returned after a dynamic calibration process completes.
 * Provides the newly computed calibration, the previous calibration,
 * and the difference metrics between them.
 */
struct DynamicCalibrationResult : public Buffer {
    /**
     * Detailed calibration result data.
     *
     * Includes:
     * - **newCalibration**: CalibrationHanlder obtained from the recalibration.
     * - **currentCalibration**: CalibrationHandler before recalibration.
     * - **calibrationDifference**: Quality metrics comparing old vs new calibration
     *   (rotation changes, depth error predictions, Sampson errors).
     */
    struct Data {
        /// Newly generated calibrationHAndler after recalibration
        dai::CalibrationHandler newCalibration;

        /// CalibrationHandler that was active before recalibration
        dai::CalibrationHandler currentCalibration;

        /// Differences and quality metrics between old and new calibration
        CalibrationQuality::Data calibrationDifference;

        DEPTHAI_SERIALIZE(Data, newCalibration, currentCalibration, calibrationDifference);
    };

    /**
     * Construct empty DynamicCalibrationResult message.
     */
    DynamicCalibrationResult() = default;
    virtual ~DynamicCalibrationResult() = default;

    /**
     * Construct with result data and informational string.
     */
    // clang-format off
    DynamicCalibrationResult(const Data& data, std::string information)
      : calibrationData(std::make_optional(std::move(data)))
      , info(std::move(information)) {}

    /**
     * Construct with informational string only (no result data).
     */
    DynamicCalibrationResult(std::string information) : calibrationData(std::nullopt), info(std::move(information)) {}
    // clang-format on

    /**
     * Optional calibration result data.
     * May be missing if recalibration failed or produced no valid result.
     */
    std::optional<Data> calibrationData;

    /**
     * Informational message describing the result of recalibration.
     */
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResult;
    }

    DEPTHAI_SERIALIZE(DynamicCalibrationResult, calibrationData, info);
};

}  // namespace dai
