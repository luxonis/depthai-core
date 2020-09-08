#pragma once

//std
#include <string>
#include <thread>


//project
#include "xlink/XLinkConnection.hpp"
#include "nlohmann/json.hpp"
#include "pipeline/cnn_host_pipeline.hpp"
#include "pipeline/host_pipeline.hpp"
#include "disparity_stream_post_processor.hpp"
#include "device_support_listener.hpp"
#include "host_capture_command.hpp"
#include "DataQueue.hpp"
#include "pipeline/Pipeline.hpp"
#include "CallbackHandler.hpp"

// libraries
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"



namespace dai {

// RAII for specific Device device
class Device{

public:

    // static API
    static std::vector<deviceDesc_t> getAllConnectedDevices();
    static std::tuple<bool, deviceDesc_t> getFirstAvailableDeviceDesc();
    /////

    Device();
    Device(const DeviceInfo& deviceDesc, bool usb2Mode = false);
    Device(const DeviceInfo& deviceDesc, std::string pathToCmd);
    ~Device();


    bool isPipelineRunning();
    bool startPipeline(Pipeline pipeline);


    // data queues
    DataOutputQueue& getOutputQueue(std::string name);
    DataInputQueue& getInputQueue(std::string name);

    // callback
    void setCallback(std::string name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);

    


    std::vector<std::string> get_available_streams();

    void request_jpeg();
    void request_af_trigger();
    void request_af_mode(CaptureMetadata::AutofocusMode mode);
    std::map<std::string, int> get_nn_to_depth_bbox_mapping();


    bool startTestPipeline();

private:

    std::vector<std::uint8_t> getDefaultCmdBinary(bool usb2_mode);

    std::shared_ptr<XLinkConnection> connection;
    std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>> client;
    std::vector<uint8_t> patched_cmd;

    DeviceInfo deviceInfo;

    std::unordered_map<std::string, DataOutputQueue> outputQueueMap;
    std::unordered_map<std::string, DataInputQueue> inputQueueMap;
    std::unordered_map<std::string, CallbackHandler> callbackMap;



    void wdog_thread(int& wd_timeout_ms);
    int wdog_start(void);
    int wdog_stop(void);

    void init();
    void deinit();

    std::shared_ptr<CNNHostPipeline> gl_result = nullptr;


    std::string config_backup;
    std::string cmd_backup;
    std::string usb_device_backup;
    uint8_t* binary_backup;
    long binary_size_backup;

    volatile std::atomic<int> wdog_keep;
    int wdog_thread_alive = 1;

    std::thread wd_thread;
    int wd_timeout_ms = 1000;

    //std::unique_ptr<XLinkWrapper> g_xlink; // TODO: make sync
    nlohmann::json g_config_d2h;

    std::unique_ptr<DisparityStreamPostProcessor> g_disparity_post_proc;
    std::unique_ptr<DeviceSupportListener>        g_device_support_listener;
    std::unique_ptr<HostCaptureCommand>           g_host_capture_command;

    std::map<std::string, int> nn_to_depth_mapping = {
        { "off_x", 0 },
        { "off_y", 0 },
        { "max_w", 0 },
        { "max_h", 0 },
    };

};

}
