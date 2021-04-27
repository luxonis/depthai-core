#pragma once

#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/IMUProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief IMU node. For use with color sensors.
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
     * Outputs ImgFrame message that carries BGR/RGB planar/interleaved encoded frame data.
     *
     * Suitable for use with NeuralNetwork node
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::IMUDatas, false}}};

    /**
     * Enable a new IMU sensor
     */
    void enableIMUSensor(IMUSensorConfig imuSensor);
    /**
     * Enable a list of IMU sensors
     */
    void enableIMUSensors(std::vector<IMUSensorConfig> imuSensors);
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
