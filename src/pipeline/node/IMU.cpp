#include "depthai/pipeline/node/IMU.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

void IMU::enableIMUSensor(IMUSensorConfig sensorConfig) {
    properties.imuSensors.push_back(sensorConfig);
}

void IMU::enableIMUSensor(const std::vector<IMUSensorConfig>& sensorConfigs) {
    properties.imuSensors = sensorConfigs;
}

void IMU::enableIMUSensor(IMUSensor sensor, uint32_t reportRate) {
    IMUSensorConfig config;
    config.sensorId = sensor;
    config.reportRate = reportRate;
    properties.imuSensors.push_back(config);
}

void IMU::enableIMUSensor(const std::vector<IMUSensor>& sensors, uint32_t reportRate) {
    std::vector<IMUSensorConfig> configs;
    for(auto& sensor : sensors) {
        IMUSensorConfig config;
        config.sensorId = sensor;
        config.reportRate = reportRate;
        configs.push_back(config);
    }
    properties.imuSensors = configs;
}

void IMU::setBatchReportThreshold(std::int32_t batchReportThreshold) {
    properties.batchReportThreshold = batchReportThreshold;
}

std::int32_t IMU::getBatchReportThreshold() const {
    return properties.batchReportThreshold;
}

void IMU::setMaxBatchReports(std::int32_t maxBatchReports) {
    properties.maxBatchReports = maxBatchReports;
}

std::int32_t IMU::getMaxBatchReports() const {
    return properties.maxBatchReports;
}

void IMU::enableFirmwareUpdate(bool enable) {
    properties.enableFirmwareUpdate = enable;
}

bool IMU::isSourceNode() const {
    return true;
}

NodeRecordParams IMU::getNodeRecordParams() const {
    NodeRecordParams params;
    params.name = "IMU";
    return params;
}

IMU::Output& IMU::getRecordOutput() {
    return out;
}
IMU::Input& IMU::getReplayInput() {
    return mockIn;
}

}  // namespace node
}  // namespace dai
