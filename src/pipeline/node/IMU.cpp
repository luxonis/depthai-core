#include "depthai/pipeline/node/IMU.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

IMU::IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string IMU::getName() const {
    return "IMU";
}

std::vector<Node::Output> IMU::getOutputs() {
    return {out};
}

std::vector<Node::Input> IMU::getInputs() {
    return {};
}

nlohmann::json IMU::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> IMU::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void IMU::enableIMUSensor(IMUSensorConfig imuSensor) {
    properties.imuSensors.push_back(imuSensor);
}

void IMU::enableIMUSensors(std::vector<IMUSensorConfig> imuSensors) {
    properties.imuSensors = imuSensors;
}

void IMU::setBatchReportThreshold(std::int32_t batchReportThreshold) {
    properties.batchReportThreshold = batchReportThreshold;
}

std::int32_t IMU::getBatchReportThreshold(std::int32_t batchReportThreshold) const {
    return properties.batchReportThreshold;
}

void IMU::setMaxBatchReports(std::int32_t maxBatchReports) {
    properties.maxBatchReports = maxBatchReports;
}

std::int32_t IMU::getMaxBatchReports(std::int32_t maxBatchReports) const {
    return properties.maxBatchReports;
}

}  // namespace node
}  // namespace dai
