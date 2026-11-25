#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/device/CalibrationHandler.hpp"

namespace dai {
namespace gate {

/**
 * Device-wide owner of the resolved multi-device calibration graph.
 *
 * The graph arrives once with the pipeline schema and can be replaced at runtime
 * through the setMultiDeviceCalibration RPC. Source nodes keep a snapshot and
 * compare its generation against getGeneration() per frame to pick up changes.
 */
class MultiDeviceCalibrationStore {
   public:
    struct Snapshot {
        /** Resolved handler, or nullptr when no graph is set. */
        std::shared_ptr<const beta::MultiDeviceCalibrationHandler> handler;
        uint32_t generation = 0;
    };

    static MultiDeviceCalibrationStore& getInstance();

    /**
     * Validates the graph structure and this device's local calibration origin
     * against it, then installs it and bumps the generation.
     *
     * @param graph Cross-device edges, or std::nullopt to remove the graph
     * @param localCalibration This device's local calibration, if available
     * @param deviceId This device's stable identifier
     * @throws std::invalid_argument when the graph structure is invalid
     * @throws std::runtime_error when the local calibration does not match the graph
     */
    void set(const std::optional<std::vector<MultiDeviceExtrinsics>>& graph,
             const std::optional<CalibrationHandler>& localCalibration,
             const std::string& deviceId);

    /**
     * Checks that a (new) local calibration is compatible with the installed graph.
     *
     * @throws std::runtime_error when the device is part of the graph and the local
     * calibration lacks the graph socket or resolves to a different origin
     */
    void validateLocalCalibration(const std::optional<CalibrationHandler>& localCalibration, const std::string& deviceId) const;

    Snapshot get() const;

    /** Lock-free generation lookup for the per-frame change check. */
    uint32_t getGeneration() const;

   private:
    MultiDeviceCalibrationStore() = default;

    mutable std::mutex mtx;
    std::shared_ptr<const beta::MultiDeviceCalibrationHandler> handler;
    std::atomic<uint32_t> generation{0};
};

}  // namespace gate
}  // namespace dai
