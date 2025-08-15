#pragma once

#include <DynamicCalibration.hpp>
#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <unordered_map>
#include <vector>

#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

struct CoverageData : public Buffer {
    /**
     *
     * It contains information about 2D distribution of data over image. It is created per frame:
     * - coveragePerCell; tells the 2D spatial distribution, set by the DCL itself
     *    It is a matrix, which presents how data is distributed on the 2D image
     *    Values in matrix presents overall fullness of the bin; [0, 1], with 0 being worst, 1 best.
     * - meanCoverage; tells overall Quality of 2D distribution, combined from both images.
     */
    // clang-format off
    CoverageData(const dcl::CoverageData& cd)
      : coveragePerCellA(cd.coveragePerCellA)
      , coveragePerCellB(cd.coveragePerCellB)
      , meanCoverage(cd.meanCoverage)
      , dataAcquired(cd.dataAcquired)
    {}
    // clang-format on
    CoverageData() = default;
    virtual ~CoverageData() = default;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::CoverageData;
    }

    std::vector<std::vector<float>> coveragePerCellA;
    std::vector<std::vector<float>> coveragePerCellB;
    float meanCoverage;

    float dataAcquired;

    DEPTHAI_SERIALIZE(CoverageData, coveragePerCellA, coveragePerCellB, meanCoverage, dataAcquired);
};

/**
 * CalibrationQuality is result which is returned after
 * dai::DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK It contains information about:
 * - CoverageData: 2D distribution of data over image
 * - CalibrationData: information about theoretical predictions how would the new recalibrated calibration look like.
 */

struct CalibrationQuality : public Buffer {
    /**
     *
     * It contains information about how would new calibration will affect current state of device. It is created after
     * succesful START_CALIBRATION_QUALITY_CHECK:
     * - rotationChange: difference in rotation angles in extrinsics matrix with old and new calibration.
     * In case angle difference is constantly over some
     * threshold it would mean, device calibration has been afected and should be good to recalibrate the device. UNITS
     * [deg].
     * - depthErrorDifference: tehoretical prediction of relative depth difference between the old and new calibration.
     * It includes the values from [1m, 2m, 5m, 10m].
     * In case that difference is very high for all presented distances, it would mean, that recalibration is required.
     * UNITS
     * [%]
     */
    struct Data {
        std::array<float, 3> rotationChange;
        std::vector<float> depthErrorDifference;
        float sampsonErrorCurrent;
        float sampsonErrorAchievable;

        DEPTHAI_SERIALIZE(Data, rotationChange, sampsonErrorCurrent, sampsonErrorAchievable, depthErrorDifference);
    };

    CalibrationQuality() = default;
    virtual ~CalibrationQuality() = default;
    CalibrationQuality(Data data, std::string info) : data(std::make_optional(std::move(data))), info(std::move(info)) {}
    CalibrationQuality(std::string info) : data(std::nullopt), info(std::move(info)) {}

    std::optional<Data> data;
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResult;
    }

    DEPTHAI_SERIALIZE(CalibrationQuality, data, info);
};

/**
 * DynamicCalibrationResults message.
 */

struct DynamicCalibrationResult : public Buffer {
    struct Data {
        dai::CalibrationHandler newCalibration;
        dai::CalibrationHandler currentCalibration;
        CalibrationQuality::Data calibrationDifference;

        DEPTHAI_SERIALIZE(Data, newCalibration, currentCalibration, calibrationDifference);
    };

    DynamicCalibrationResult() = default;
    virtual ~DynamicCalibrationResult() = default;
    // clang-format off
    DynamicCalibrationResult(const Data& data, std::string information)
      : calibrationData(std::make_optional(std::move(data)))
      , info(std::move(information)) {}

    DynamicCalibrationResult(std::string information) : calibrationData(std::nullopt), info(std::move(information)) {}
    // clang-format on

    std::optional<Data> calibrationData;

    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResult;
    }

    DEPTHAI_SERIALIZE(DynamicCalibrationResult, calibrationData, info);
};

}  // namespace dai
