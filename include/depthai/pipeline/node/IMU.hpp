#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/IMUProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief IMU node for BNO08X.
 */
class IMU : public DeviceNodeCRTP<DeviceNode, IMU, IMUProperties> {
   protected:
    bool isSourceNode() const override;
    NodeRecordParams getNodeRecordParams() const override;
    Output& getRecordOutput() override;
    Input& getReplayInput() override;

   public:
    constexpr static const char* NAME = "IMU";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    std::shared_ptr<IMU> build() {
        return std::static_pointer_cast<IMU>(shared_from_this());
    }
    /**
     * Outputs IMUData message that carries IMU packets.
     */
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::IMUData, false}}}};

    /**
     * Mock IMU data for replaying recorded data
     */
    Input mockIn{*this, {.name = "mockIn", .types = {{DatatypeEnum::IMUData, false}}}};

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

    /*
     * Whether to perform firmware update or not.
     * Default value: false.
     */
    void enableFirmwareUpdate(bool enable);
};

}  // namespace node
}  // namespace dai
