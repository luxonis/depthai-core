#pragma once

// std
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/utility/Pimpl.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

// shared
#include "depthai-shared/log/LogLevel.hpp"
#include "depthai-shared/pb/common/ChipTemperature.hpp"
#include "depthai-shared/pb/common/MemoryInfo.hpp"

// libraries
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

namespace dai {

// Device (RAII), connects to device and maintains watchdog, timesync, ...
class Device {
   public:
    // static API
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();
    static std::tuple<bool, DeviceInfo> getDeviceByMxId(std::string mxId);
    static std::vector<DeviceInfo> getAllAvailableDevices();
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(bool usb2Mode, OpenVINO::Version version = Pipeline::DEFAULT_OPENVINO_VERSION);

    // static std::vector<deviceDesc_t> getAllConnectedDevices();
    // static std::tuple<bool, deviceDesc_t> getFirstAvailableDeviceDesc();
    /////

    explicit Device(const Pipeline& pipeline);
    Device(const Pipeline& pipeline, bool usb2Mode);
    Device(const Pipeline& pipeline, const char* pathToCmd);
    Device(const Pipeline& pipeline, const std::string& pathToCmd);
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, bool usb2Mode = false);
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, const char* pathToCmd);
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, const std::string& pathToCmd);
    ~Device();

    bool isPipelineRunning();
    bool startPipeline();

    void setLogLevel(LogLevel level);
    LogLevel getLogLevel();

    // data queues
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name, unsigned int maxSize = 16, bool blocking = true);
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name, unsigned int maxSize = 16, bool blocking = true);

    // callback
    void setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);

    // Convinience functions for querying current system information
    MemoryInfo getDdrMemoryUsage();
    MemoryInfo getLeonOsHeapUsage();
    MemoryInfo getLeonRtHeapUsage();
    ChipTemperature getChipTemperature();

   private:
    // private static
    void init(const Pipeline& pipeline, bool embeddedMvcmd, bool usb2Mode, const std::string& pathToMvcmd);

    std::shared_ptr<XLinkConnection> connection;
    std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>> client;
    std::mutex rpcMutex;
    std::vector<uint8_t> patchedCmd;

    DeviceInfo deviceInfo = {};

    std::unordered_map<std::string, std::shared_ptr<DataOutputQueue>> outputQueueMap;
    std::unordered_map<std::string, std::shared_ptr<DataInputQueue>> inputQueueMap;
    std::unordered_map<std::string, CallbackHandler> callbackMap;

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // Timesync thread
    std::thread timesyncThread;
    std::atomic<bool> timesyncRunning{true};

    // Logging thread
    std::thread loggingThread;
    std::atomic<bool> loggingRunning{true};

    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    // Serialized pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    OpenVINO::Version version;
};

}  // namespace dai
