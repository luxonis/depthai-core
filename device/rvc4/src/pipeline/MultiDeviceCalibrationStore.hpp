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

class MultiDeviceCalibrationStore {
   public:
    struct Snapshot {
        std::shared_ptr<const beta::MultiDeviceCalibrationHandler> handler;  // nullptr when no graph is set
        uint32_t generation = 0;
    };

    static MultiDeviceCalibrationStore& getInstance();

    void set(const std::optional<std::vector<MultiDeviceExtrinsics>>& graph,  // graph nullopt removes it; throws on an invalid graph or a local origin mismatch
             const std::optional<CalibrationHandler>& localCalibration,
             const std::string& deviceId);

    void validateLocalCalibration(const std::optional<CalibrationHandler>& localCalibration, const std::string& deviceId) const;

    Snapshot get() const;

    uint32_t getGeneration() const;

   private:
    MultiDeviceCalibrationStore() = default;

    mutable std::mutex mtx;
    std::shared_ptr<const beta::MultiDeviceCalibrationHandler> handler;
    std::atomic<uint32_t> generation{0};
};

}  // namespace gate
}  // namespace dai
