#pragma once

// std
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "device_support_listener.hpp"
#include "disparity_stream_post_processor.hpp"
#include "host_capture_command.hpp"
#include "nlohmann/json.hpp"
#include "pipeline/Pipeline.hpp"
#include "pipeline/cnn_host_pipeline.hpp"
#include "pipeline/host_pipeline.hpp"
#include "xlink/XLinkConnection.hpp"

// libraries
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

namespace dai {

// Device (RAII), connects to device and maintains watchdog, timesync, ...
class Device {
   public:
    // static API
    static std::vector<deviceDesc_t> getAllConnectedDevices();
    static std::tuple<bool, deviceDesc_t> getFirstAvailableDeviceDesc();
    /////

    Device();
    explicit Device(const DeviceInfo& devInfo, bool usb2Mode = false);
    Device(const DeviceInfo& devInfo, const char* pathToCmd);
    Device(const DeviceInfo& devInfo, const std::string& pathToCmd);
    ~Device();

    bool isPipelineRunning();
    bool startPipeline(Pipeline& pipeline);

    // data queues
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name, unsigned int maxSize = 120, bool overwrite = false);
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name, unsigned int maxSize = 120, bool overwrite = false);

    // callback
    void setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);

    std::vector<std::string> get_available_streams();

    void request_jpeg();
    void request_af_trigger();
    void request_af_mode(CaptureMetadata::AutofocusMode mode);
    std::map<std::string, int> get_nn_to_depth_bbox_mapping();

    bool startTestPipeline(int testId);

   private:
    // private static
    static std::vector<std::uint8_t> getDefaultCmdBinary(bool usb2_mode);

    void init();

    std::shared_ptr<XLinkConnection> connection;
    std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>> client;
    std::mutex rpcMutex;
    std::vector<uint8_t> patchedCmd;

    DeviceInfo deviceInfo = {};

    std::unordered_map<std::string, std::shared_ptr<DataOutputQueue>> outputQueueMap;
    std::unordered_map<std::string, std::shared_ptr<DataInputQueue>> inputQueueMap;
    std::unordered_map<std::string, CallbackHandler> callbackMap;

    // XLink timeouts
    const std::chrono::milliseconds READ_WRITE_TIMEOUT{500};

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // Timesync thread
    std::thread timesyncThread;
    std::atomic<bool> timesyncRunning{true};
};

}  // namespace dai
