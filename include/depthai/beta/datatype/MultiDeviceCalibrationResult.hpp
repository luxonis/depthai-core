#pragma once

#include <cstdint>
#include <depthai/beta/device/MultiDeviceCalibrationHandler.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/Serialization.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dai {
namespace beta {

/**
 * @brief Pure data emitted by a MultiDeviceCalibration node.
 *
 * The calibration graph is a snapshot and is never applied to a device or
 * pipeline by the node. Use getHandler() to resolve it or store it on a
 * pipeline with Pipeline::setMultiDeviceCalibration.
 */
struct MultiDeviceCalibrationResult : public Buffer {
    MultiDeviceCalibrationResult() = default;
    explicit MultiDeviceCalibrationResult(std::string information) : info(std::move(information)) {}
    ~MultiDeviceCalibrationResult() override;

    /**
     * Validated, meter-normalized cross-device calibration edges. Present when
     * the calibration passed.
     */
    std::optional<std::vector<MultiDeviceExtrinsics>> graph;
    bool passed = false;
    double dataConfidence = 0.0;
    double sampsonError = 0.0;
    std::string info;

    /**
     * Build a MultiDeviceCalibrationHandler from the calibration graph.
     *
     * @return The handler when the result carries a graph, or std::nullopt
     * when it does not.
     */
    std::optional<MultiDeviceCalibrationHandler> getHandler() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MultiDeviceCalibrationResult;
    }

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationResult, graph, passed, dataConfidence, sampsonError, info);
};

}  // namespace beta
}  // namespace dai
