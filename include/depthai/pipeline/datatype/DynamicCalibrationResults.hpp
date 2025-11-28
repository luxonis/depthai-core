#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * @defgroup dcl_results Dynamic Calibration Messages
 * @brief Result messages produced by the Dynamic Calibration Library (DCL).
 * @{
 */

/**
 * @brief Coverage information for a single frame.
 *
 * Contains 2D spatial coverage metrics used internally during
 * dynamic calibration to measure how well the image area has been populated.
 *
 * @ingroup dcl_results
 */
struct CoverageData : public Buffer {
    CoverageData() = default;
    virtual ~CoverageData();

    /**
     * @brief Serialize CoverageData to buffer.
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::CoverageData;
    }

    /** @name Spatial coverage matrices */
    ///@{

    /** 2D coverage matrix for input A (e.g. left image). Values are ∈ [0, 1]. */
    std::vector<std::vector<float>> coveragePerCellA;

    /** 2D coverage matrix for input B (e.g. right image). Values are ∈ [0, 1]. */
    std::vector<std::vector<float>> coveragePerCellB;

    ///@}

    /** @name Summary coverage metrics */
    ///@{

    /** Overall mean coverage across both inputs ∈ [0, 1]. */
    float meanCoverage = 0.0f;

    /** Proportion of desired spatial coverage acquired so far. */
    float coverageAcquired = 0.0f;

    /** Proportion of calibration-relevant data acquired from the frame. */
    float dataAcquired = 0.0f;

    ///@}

    DEPTHAI_SERIALIZE(CoverageData, coveragePerCellA, coveragePerCellB, meanCoverage, dataAcquired, coverageAcquired);
};

/**
 * @brief Summary of calibration quality metrics.
 *
 * Returned from a calibration quality check. Describes
 * expected accuracy improvement if a new calibration were applied.
 *
 * @ingroup dcl_results
 */
struct CalibrationQuality : public Buffer {
    /**
     * @brief Quality metrics describing differences between current
     *        and predicted calibration.
     *
     * @ingroup dcl_results
     */
    struct Data {
        /** Rotation difference between old and new extrinsics (degrees). */
        std::array<float, 3> rotationChange;

        /**
         * Predicted relative depth error difference between current and new calibration.
         * Reported at reference distances [1m, 2m, 5m, 10m].
         * Units: percent [%].
         */
        std::vector<double> depthErrorDifference;

        /** Sampson error of currently installed calibration. */
        float sampsonErrorCurrent = 0.0f;

        /** Estimated new Sampson error if the new calibration is applied. */
        float sampsonErrorNew = 0.0f;

        DEPTHAI_SERIALIZE(Data, rotationChange, sampsonErrorCurrent, sampsonErrorNew, depthErrorDifference);
    };

    CalibrationQuality() = default;
    virtual ~CalibrationQuality();

    /**
     * @brief Construct a CalibrationQuality message with data.
     * @param qualityData Metrics describing the quality difference.
     * @param info Informational text describing the result.
     */
    CalibrationQuality(Data qualityData, std::string info) : qualityData(std::make_optional(std::move(qualityData))), info(std::move(info)) {}

    /**
     * @brief Construct a CalibrationQuality message without metric data.
     * @param info Informational text describing the result.
     */
    CalibrationQuality(std::string info) : qualityData(std::nullopt), info(std::move(info)) {}

    /** Optional quality metrics. */
    std::optional<Data> qualityData;

    /** Human-readable result description. */
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::CalibrationQuality;
    }

    DEPTHAI_SERIALIZE(CalibrationQuality, qualityData, info);
};

/**
 * @brief Final result of running dynamic calibration.
 *
 * Includes:
 *  - newly computed calibration
 *  - previously installed calibration
 *  - metrics comparing the two
 *
 * @ingroup dcl_results
 */
struct DynamicCalibrationResult : public Buffer {
    /**
     * @brief Detailed calibration result.
     *
     * @ingroup dcl_results
     */
    struct Data {
        /** Calibration produced by dynamic calibration. */
        dai::CalibrationHandler newCalibration;

        /** Calibration present before dynamic calibration. */
        dai::CalibrationHandler currentCalibration;

        /** Per-metric comparison of new vs old calibration. */
        CalibrationQuality::Data calibrationDifference;

        DEPTHAI_SERIALIZE(Data, newCalibration, currentCalibration, calibrationDifference);
    };

    DynamicCalibrationResult() = default;
    virtual ~DynamicCalibrationResult();

    /**
     * @brief Construct result including calibration data.
     */
    DynamicCalibrationResult(const Data& data, std::string information) : calibrationData(std::make_optional(data)), info(std::move(information)) {}

    /**
     * @brief Construct result containing only info text.
     */
    DynamicCalibrationResult(std::string information) : calibrationData(std::nullopt), info(std::move(information)) {}

    /** Optional calibration result data. */
    std::optional<Data> calibrationData;

    /** Informational result message. */
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::DynamicCalibrationResult;
    }

    DEPTHAI_SERIALIZE(DynamicCalibrationResult, calibrationData, info);
};

/** @}  End of dcl_results group */

}  // namespace dai
