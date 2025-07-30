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

    float dataAcquired = 0.0f;

    DEPTHAI_SERIALIZE(CoverageData, coveragePerCellA, coveragePerCellB, meanCoverage, dataAcquired);
};

/**
 * CalibrationQuality is result which is returned after
 * dai::DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK It contains information about:
 * - CoverageData: 2D distribution of data over image
 * - CalibrationData: information about theoretical predictions how would the new recalibrated calibration look like.
 */
struct CalibrationQuality : Buffer {
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
        float epipolarErrorChange;
        std::vector<float> depthErrorDifference;
        DEPTHAI_SERIALIZE(Data, rotationChange, epipolarErrorChange, depthErrorDifference);
    };

    CalibrationQuality() = default;
    virtual ~CalibrationQuality() = default;
    CalibrationQuality(Data data, std::string info) : data(std::make_optional(std::move(data))), info(std::move(info)) {}

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
    DynamicCalibrationResult() = default;
    virtual ~DynamicCalibrationResult() = default;

    DynamicCalibrationResult(std::optional<dai::CalibrationHandler> calib, std::optional<CalibrationQuality::Data> quality, std::string information)
        : calibration(std::move(calib)), calibrationQuality(std::move(quality)), info(std::move(information)) {}

    std::optional<dai::CalibrationHandler> calibration;
    std::optional<CalibrationQuality::Data> calibrationQuality;
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DynamicCalibrationResult;
    }

    DEPTHAI_SERIALIZE(DynamicCalibrationResult, calibrationQuality, calibration, info);
};

}  // namespace dai
