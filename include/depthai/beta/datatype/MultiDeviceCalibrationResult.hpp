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
 * @brief Final result of running multi-device calibration.
 *
 * Includes:
 *  - the estimated cross-device calibration graph (only when the run passed)
 *  - metrics evaluating the reliability of the estimate and the quality of the
 *    data used to compute it
 *  - a human-readable description of why a run did not pass
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
     * @brief Validated, meter-normalized cross-device calibration edges.
     *
     * One edge per non-reference device, mapping that device's local origin to
     * the local origin of the reference device. Present only when `passed` is
     * true.
     */
    std::optional<std::vector<MultiDeviceExtrinsics>> graph;

    /**
     * @brief True if the calibration completed and `graph` holds a pose for
     * every registered device.
     *
     * False when data collection or the solver failed; `info` explains why.
     */
    bool passed = false;

    /**
     * @brief Quality score of the input data.
     *
     * A normalized value between 0.0 and 1.0 indicating how much you can trust
     * the collected samples. Remains 0.0 when the solver was not reached.
     */
    double dataConfidence = 0.0;

    /**
     * @brief Sampson error of the estimated calibration over the collected
     * samples.
     *
     * Lower is better. Remains 0.0 when the solver was not reached.
     */
    double sampsonError = 0.0;

    /**
     * @brief Human-readable result description.
     *
     * Explains why `passed` is false; empty when the calibration passed.
     */
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
