#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/IMUProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief IMU node for BNO08X.
 */
class IMU : public Node {
   public:
    using Properties = dai::IMUProperties;

   private:
    Properties properties;

    std::string getName() const override;
    std::vector<Output> getOutputs() override;
    std::vector<Input> getInputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

   public:
    /**
     * Constructs IMU node.
     */
    IMU(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Outputs IMUData message that carries IMU packets.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::IMUData, false}}};

    /**
     * Enable a new IMU sensor with explicit configuration
     */
    void enableIMUSensor(IMUSensorConfig sensorConfig);

    /**
     * Enable a list of IMU sensors with explicit configuration
     */
    void enableIMUSensor(const std::vector<IMUSensorConfig>& sensorConfigs);

    /**
     * Enable a new IMU sensor with default configuration
     */
    void enableIMUSensor(IMUSensor sensor, uint32_t reportRate);

    /**
     * Enable a list of IMU sensors with default configuration
     */
    void enableIMUSensor(const std::vector<IMUSensor>& sensors, uint32_t reportRate);

    /**
     * Above this packet threshold data will be sent to host, if queue is not blocked
     */
    void setBatchReportThreshold(std::int32_t batchReportThreshold);

    /**
     * Above this packet threshold data will be sent to host, if queue is not blocked
     */
    std::int32_t getBatchReportThreshold() const;

    /**
     * Maximum number of IMU packets in a batch report
     */
    void setMaxBatchReports(std::int32_t maxBatchReports);

    /**
     * Maximum number of IMU packets in a batch report
     */
    std::int32_t getMaxBatchReports() const;
};

}  // namespace node
}  // namespace dai
