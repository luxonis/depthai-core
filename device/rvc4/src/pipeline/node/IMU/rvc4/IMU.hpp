#pragma once

#include <cstddef>
#include <unordered_set>

#include "SensorDirectChannel.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/properties/IMUProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

struct ImuSensorConfig {
    uint32_t sensorType = 0;
    SensorStreamConfig streamConfig;
    SensorDirectChannel sensorDirectChannel;
};

class IMU : public dai::node::IMU {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::IMU::NAME;
    IMU() : IMU(std::make_unique<IMU::Properties>()) {}
    IMU(std::unique_ptr<Properties> props) : node::IMU(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
    static int Probe();

    static std::string getIMUVersion();
    static std::string getIMUFirmwareVersion();
    static std::string getEmbeddedIMUFirmwareVersion();
    static void overrideConnectedIMU(const std::string& imu);

    void buildStage1() override;
    void run() override;

   private:
    int Init();
    static bool daiToSensorConfig(const dai::IMUSensorConfig& src, ImuSensorConfig& dst);

    static std::unordered_set<int> reportIdsRequested;
    static spdlog::logger* pLogger;

    static float imuFwUpdateProgress;
    static bool imuFwUpdatePending;
    static std::string firmwareVersion;
    static std::string latestAvailableFirmwareVersion;
    static bool probed;
    static std::mutex imuMtx;
    static std::string detectedImu;
    static std::atomic<bool> instanceCount;

    bool imuInitialized = false;
    bool doImuFirmwareUpdate = false;

    std::vector<ImuSensorConfig> initialReports;

    void readMuxedDataProducer(char* shared_buff_ptr, size_t sharedBuffSize, uint32_t sampleRateHz, bool shouldSignal = false);
    void readGenericDataProducer(char* shared_buff_ptr, size_t sharedBuffSize, uint32_t sampleRateHz, bool shouldSignal = false);
};

}  // namespace gate
}  // namespace dai
