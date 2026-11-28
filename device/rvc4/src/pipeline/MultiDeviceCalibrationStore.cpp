#include "MultiDeviceCalibrationStore.hpp"

#include <stdexcept>

#include "depthai/beta/device/LocalCalibrationOrigin.hpp"

namespace dai {
namespace gate {

namespace {

beta::LocalCalibrationOrigin describeLocalCalibration(const std::optional<CalibrationHandler>& localCalibration) {
    std::optional<EepromData> calibration;
    if(localCalibration.has_value()) {
        calibration = localCalibration->getEepromData();
    }
    return beta::describeLocalCalibration(calibration, Platform::RVC4);
}

}  // namespace

MultiDeviceCalibrationStore& MultiDeviceCalibrationStore::getInstance() {
    static MultiDeviceCalibrationStore instance;
    return instance;
}

void MultiDeviceCalibrationStore::set(const std::optional<std::vector<MultiDeviceExtrinsics>>& graph,
                                      const std::optional<CalibrationHandler>& localCalibration,
                                      const std::string& deviceId) {
    std::shared_ptr<const beta::MultiDeviceCalibrationHandler> resolved;
    if(graph.has_value()) {
        resolved = std::make_shared<const beta::MultiDeviceCalibrationHandler>(*graph);
        resolved->validateLocalOrigin(deviceId, describeLocalCalibration(localCalibration));
    }

    std::lock_guard<std::mutex> lock(mtx);
    handler = std::move(resolved);
    generation.fetch_add(1, std::memory_order_acq_rel);
}

void MultiDeviceCalibrationStore::validateLocalCalibration(const std::optional<CalibrationHandler>& localCalibration, const std::string& deviceId) const {
    const auto snapshot = get();
    if(snapshot.handler) {
        snapshot.handler->validateLocalOrigin(deviceId, describeLocalCalibration(localCalibration));
    }
}

MultiDeviceCalibrationStore::Snapshot MultiDeviceCalibrationStore::get() const {
    std::lock_guard<std::mutex> lock(mtx);
    return {handler, generation.load(std::memory_order_acquire)};
}

uint32_t MultiDeviceCalibrationStore::getGeneration() const {
    return generation.load(std::memory_order_acquire);
}

}  // namespace gate
}  // namespace dai
