#pragma once

// std
#include <condition_variable>
#include <mutex>
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
    // constants
    static constexpr std::chrono::seconds DEFAULT_SEARCH_TIME{3};
    static constexpr std::size_t EVENT_QUEUE_MAXIMUM_SIZE{32768};

    // static API
    template <typename Rep, typename Period>
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice(std::chrono::duration<Rep, Period> timeout);
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice();
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();
    static std::tuple<bool, DeviceInfo> getDeviceByMxId(std::string mxId);
    static std::vector<DeviceInfo> getAllAvailableDevices();
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(bool usb2Mode, OpenVINO::Version version = Pipeline::DEFAULT_OPENVINO_VERSION);

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

    /**
     * Gets an output queue corresponding to stream name. If it doesn't exist it throws
     *
     * @param name Queue/stream name, created by XLinkOut node
     * @return Smart pointer to DataOutputQueue
     */
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name);

    /**
     * Gets a queue corresponding to stream name, if it exists, otherwise it throws. Also sets queue options
     *
     * @param name Queue/stream name, set in XLinkOut node
     * @param maxSize Maximum number of messages in queue
     * @param blocking Queue behavior once full. True specifies blocking and false overwriting of oldest messages. Default: true
     * @return Smart pointer to DataOutputQueue
     */
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name, unsigned int maxSize, bool blocking = true);

    /**
     * Gets an input queue corresponding to stream name. If it doesn't exist it throws
     *
     * @param name Queue/stream name, set in XLinkIn node
     * @return Smart pointer to DataInputQueue
     */
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name);

    /**
     * Gets an input queue corresponding to stream name. If it doesn't exist it throws. Also sets queue options
     *
     * @param name Queue/stream name, set in XLinkOut node
     * @param maxSize Maximum number of messages in queue
     * @param blocking Queue behavior once full. True: blocking, false: overwriting of oldest messages. Default: true
     * @return Smart pointer to DataInputQueue
     */
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name, unsigned int maxSize, bool blocking = true);

    // callback
    void setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);

    /**
     * Gets or waits until any of specified queues has received a message
     *
     * @param queueNames Names of queues for which to block
     * @param maxNumEvents Maximum number of events to remove from queue - Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite - Default is -1
     * @return Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(const std::vector<std::string>& queueNames,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));
    std::vector<std::string> getQueueEvents(const std::initializer_list<std::string>& queueNames,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until specified queue has received a message
     *
     * @overload
     */
    std::vector<std::string> getQueueEvents(std::string queueName,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any any queue has received a message
     *
     * @overload
     */
    std::vector<std::string> getQueueEvents(std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any of specified queues has received a message
     *
     * @param queueNames Names of queues for which to block
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite - Default is -1
     * @return Queue name which received a message first
     */
    std::string getQueueEvent(const std::vector<std::string>& queueNames, std::chrono::microseconds timeout = std::chrono::microseconds(-1));
    std::string getQueueEvent(const std::initializer_list<std::string>& queueNames, std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until specified queue has received a message
     *
     * @overload
     */
    std::string getQueueEvent(std::string queueName, std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any queue has received a message
     *
     * @overload
     */
    std::string getQueueEvent(std::chrono::microseconds timeout = std::chrono::microseconds(-1));

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

    // Event queue
    std::mutex eventMtx;
    std::condition_variable eventCv;
    std::deque<std::string> eventQueue;
    std::vector<std::string> allQueueNames;

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
