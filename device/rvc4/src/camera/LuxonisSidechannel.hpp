#pragma once

#include <iostream>
#include <mutex>
#include <optional>
#include <thread>

#include "luxonis_sidechannel_data.h"
#include "utilities/ErrorMacros.hpp"
#include "camera/CameraSingleton.hpp"

class LuxonisSidechannel {
   public:
    // This can be NULL on old sidechannel version so always check and handle correctly before calling!!!
    struct operations {
        lux_sc_camera_mode_set_cb_t camera_mode_set;
        lux_sc_camera_sockets_config_cb_t camera_sockets_config;
        lux_sc_camera_lens_pos_set_cb_t camera_lens_pos_set;
        lux_sc_sensor_register_control_cb_t sensor_register_control;
        lux_sc_camera_tuning_update_cb_t camera_tuning_update;
        lux_sc_camera_socket_for_csi_get_cb_t camera_socket_for_csi_get;
        lux_sc_sensors_get_cb_t sensors_get;
        lux_sc_sensor_get_error_gpio_cb_t sensor_get_error_gpio;
        lux_sc_sensor_err_cb_add_t sensor_err_cb_add;
        lux_sc_sensor_err_cb_rm_t sensor_err_cb_rm;
        // ISP brightness -10..10; NULL with a camera HAL that does not implement it.
        // shortcut: luxonis_sidechannel_data.h has no lux_sc_camera_brightness_set_cb_t yet. Use it, like the other ops, once it does.
        void (*camera_brightness_set)(uint32_t camera_id, int32_t level);
    };

    operations ops{};

    // Must be called exactly once before initialize().
    // Must be called before HAL dlopen()'s the camx *.so file
    // not thread safe
    static void earlyInit();

    // Should be called exactly once before calling getInstance().
    // not thread safe
    static void initialize(void* camxDlopenHandle);

    static LuxonisSidechannel& getInstance() {
        // this line is necessary to create the below instance
        (void)CameraSingleton::getInstance();
        DAI_CHECK_IN(instance);
        return *instance;
    }

    static LuxonisSidechannel* tryGetInstance() {
        return instance.get();
    }

    bool isV1();

    LuxonisSidechannel(void* camxDlopenHandle);

    ~LuxonisSidechannel();

    void setCameraMode(lux_sc_camera_mode_set_t cameraMode);

    template <typename T>
    void sendMessage(const T& data) {
        initializeFifo();
        std::cout << "Sending message type: " << data.type << "\n" << std::flush;
        writeToPipe(&data, sizeof(data));
    }

    std::optional<lux_sc_version_result_t> const& getProtocolVersion() {
        return protocolVersion;
    }

    void handleExposureUpdate(const lux_sc_exposure_update_t& msg);

   private:
    static std::unique_ptr<LuxonisSidechannel> instance;

    std::thread sidechannelThread;
    std::optional<int> fifoFd;
    std::mutex fifoFdLock;

    std::optional<lux_sc_version_result_t> protocolVersion;
    lux_sc_exposure_update_listen_t scExposureUpdateListen;
    void* camxDlopenHandle{nullptr};

    void listenToLuxonisSidechannel();
    void initializeFifo();
    void writeToPipe(const void* data, size_t dataLength);
};
