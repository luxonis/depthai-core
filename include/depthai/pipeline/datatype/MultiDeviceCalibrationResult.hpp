#pragma once

#include <string>

#include "depthai/common/MultiDeviceCalibrationData.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * MultiDeviceCalibrationResult message.
 * Carries the rig calibration estimated by the MultiDeviceCalibration node, i.e. the transformations between frames of
 * different devices. Per-device calibration is not part of it - it always comes from the live device.
 */
class MultiDeviceCalibrationResult : public Buffer {
   public:
    MultiDeviceCalibrationResult() = default;
    explicit MultiDeviceCalibrationResult(std::string info) : info(std::move(info)) {}
    MultiDeviceCalibrationResult(MultiDeviceCalibrationData calibration, double dataConfidence, std::string info = {})
        : calibration(std::move(calibration)), dataConfidence(dataConfidence), passed(true), info(std::move(info)) {}

    virtual ~MultiDeviceCalibrationResult();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MultiDeviceCalibrationResult;
    }

    /**
     * @brief Estimated rig calibration. Empty if the estimation did not succeed.
     */
    MultiDeviceCalibrationData calibration;

    /**
     * @brief Quality of the input data the estimation was run on (0.0 to 1.0).
     */
    double dataConfidence = 0.0;

    /**
     * @brief True if a rig calibration was estimated.
     */
    bool passed = false;

    /**
     * @brief Human readable information about the estimation, e.g. the reason it did not succeed.
     */
    std::string info;

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationResult, calibration, dataConfidence, passed, info);
};

}  // namespace dai
