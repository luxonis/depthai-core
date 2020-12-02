#pragma once

//std
#include <string>
#include <thread>

//shared
#include "depthai-shared/xlink/xlink_wrapper.hpp"
#include "depthai-shared/metadata/camera_control.hpp"

//project
#include "nlohmann/json.hpp"
#include "pipeline/cnn_host_pipeline.hpp"
#include "pipeline/host_pipeline.hpp"
#include "disparity_stream_post_processor.hpp"
#include "device_support_listener.hpp"
#include "host_capture_command.hpp"


// RAII for specific Device device
class Device{

public:

    Device();

    Device(std::string usb_device, bool usb2_mode = false);

    // Basically init_device but RAII
    Device(std::string cmd_file, std::string usb_device);

    // Basically deinit_device but RAII
    ~Device();

    std::shared_ptr<CNNHostPipeline> create_pipeline(
        const std::string &config_json_str
    );
    std::shared_ptr<CNNHostPipeline> get_pipeline();
    std::vector<std::string> get_available_streams();

    std::vector<std::vector<float>> get_left_intrinsic();
    std::vector<std::vector<float>> get_left_homography();
    std::vector<std::vector<float>> get_right_intrinsic();
    std::vector<std::vector<float>> get_right_homography();
    std::vector<std::vector<float>> get_rotation();
    std::vector<float> get_translation();
    std::string get_mx_id();

    bool is_usb3();
    bool is_eeprom_loaded();
    bool is_rgb_connected();
    bool is_left_connected();
    bool is_right_connected();
    bool is_device_changed();

    void reset_device_changed();
    void request_jpeg();
    void request_af_trigger();
    void request_af_mode(CaptureMetadata::AutofocusMode mode);
    void send_disparity_confidence_threshold(uint8_t confidence);
    void send_camera_control(CameraControl::CamId camera_id,
            CameraControl::Command command_id,
            const std::string &extra_args);
    void write_eeprom_data(const std::string &board_config);
    
    std::map<std::string, int> get_nn_to_depth_bbox_mapping();

private:
    
    std::vector<uint8_t> patched_cmd;
    volatile std::atomic<int> wdog_keep;

    void wdog_keepalive(void);
    void wdog_thread(std::chrono::milliseconds& wd_timeout);
    int wdog_start(void);
    int wdog_stop(void);


    bool init_device(
        const std::string &device_cmd_file,
        const std::string &usb_device,
        uint8_t* binary = nullptr,
        long binary_size = 0
    );
    void soft_deinit_device()
    {
        if(g_host_capture_command != nullptr)
            g_host_capture_command->sendCustomDeviceResetRequest();
        g_xlink = nullptr;
        g_disparity_post_proc = nullptr;
        g_device_support_listener = nullptr;
        g_host_capture_command = nullptr;
    };
    void deinit_device(){
        wdog_stop();
        soft_deinit_device();
        gl_result = nullptr;
    };
    int read_and_parse_config_d2h(void);
    void load_and_print_config_d2h(void);


    std::shared_ptr<CNNHostPipeline> gl_result = nullptr;
    std::vector<std::vector<float>> R1_l;
    std::vector<std::vector<float>> R2_r;
    std::vector<std::vector<float>> H1_l;
    std::vector<std::vector<float>> H2_r;
    std::vector<std::vector<float>> M1_l;
    std::vector<std::vector<float>> M2_r;
    std::vector<std::vector<float>> R;
    std::vector<float> T;
    std::vector<float> d1_l;
    std::vector<float> d2_r;
    int32_t version;
    bool device_changed = true;
    std::string config_backup;
    std::string cmd_backup;
    std::string usb_device_backup;
    uint8_t* binary_backup;
    long binary_size_backup;

    int wdog_thread_alive = 0;

    std::thread wd_thread;
    std::chrono::milliseconds wd_timeout = std::chrono::milliseconds(5000);

    std::unique_ptr<XLinkWrapper> g_xlink; // TODO: make sync
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

    XLinkHandler_t g_xlink_device_handler = {};

};
