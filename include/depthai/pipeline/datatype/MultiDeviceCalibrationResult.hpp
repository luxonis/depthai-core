#pragma once

#include <cstdint>
#include <depthai/device/MultiDeviceCalibrationHandler.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/Serialization.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dai {

/**
 * @brief Pure data emitted by a MultiDeviceCalibration node.
 *
 * The handler is a snapshot and is never applied to a device or pipeline by
 * the node.
 */
struct MultiDeviceCalibrationResult : public Buffer {
    MultiDeviceCalibrationResult() = default;
    explicit MultiDeviceCalibrationResult(std::string information) : info(std::move(information)) {}
    ~MultiDeviceCalibrationResult() override;

    std::optional<MultiDeviceCalibrationHandler> handler;
    bool passed = false;
    double dataConfidence = 0.0;
    double sampsonError = 0.0;
    std::string info;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MultiDeviceCalibrationResult;
    }

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationResult, handler, passed, dataConfidence, sampsonError, info);
};

}  // namespace dai
