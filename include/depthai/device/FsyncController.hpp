#pragma once

#include <cstdint>
#include <linux/ioctl.h>

namespace dai {

class FsyncController {
public:
    // Struct definitions
    struct FsyncStmIoctlInfoData {
        uint32_t fw_ver;
        uint32_t config_reg;
        uint32_t internal_src_freq;
        float actual_out_freq;
        uint32_t in1_present;
        float in1_freq;
        float in1_duty;
        int num_outputs;
        int num_leds;
    };

    struct FsyncStmIoctlPinData {
        int id;
        int fps;
        int polarity;
        int duty;
        bool locked;
    };

    struct FsyncStmIoctlCameraData {
        uint32_t fps;
        uint32_t exposure_time;
    };

    struct FsyncStmIoctlLedData {
        int id;
        char name[32];  // LED_MAX_NAME_LEN
    };

    // IOCTL command constants
    static constexpr unsigned int CMD_SET_PROG_STATE = _IOW('f', 0, uint32_t*);
    static constexpr unsigned int CMD_GET_PROG_STATE = _IOR('f', 0, uint32_t*);
    static constexpr unsigned int CMD_GET_STM_TYPE = _IOR('f', 1, uint32_t*);
    static constexpr unsigned int CMD_SET_PIN_STATE = _IOW('f', 2, struct FsyncStmIoctlPinData*);
    static constexpr unsigned int CMD_GET_PIN_STATE = _IOR('f', 2, struct FsyncStmIoctlPinData*);
    static constexpr unsigned int CMD_SET_INFO = _IOW('f', 3, struct FsyncStmIoctlInfoData*);
    static constexpr unsigned int CMD_GET_INFO = _IOR('f', 3, struct FsyncStmIoctlInfoData*);
    static constexpr unsigned int CMD_SET_CAMERA_DATA = _IOW('f', 4, struct FsyncStmIoctlCameraData*);
    static constexpr unsigned int CMD_GET_LED = _IOR('f', 5, struct FsyncStmIoctlLedData*);

    FsyncController();
    ~FsyncController();

    int readInfoData(FsyncStmIoctlInfoData& infoData);
    int readPinData(int pinId, FsyncStmIoctlPinData& pinData);
    int readLedData(int ledId, FsyncStmIoctlLedData& ledData);
    int writePinData(const FsyncStmIoctlPinData& pinData);
    int writeCameraData(const FsyncStmIoctlCameraData& cameraData);
    int writeInfoData(const FsyncStmIoctlInfoData& infoData);

private:
    static constexpr const char* FSYNC_STM_DEVICE_PATH = "/dev/fsync-stm";

    enum class FsyncStmType {
        FSYNC_STM32C0,
        FSYNC_STM32G0,
    };
};

} // namespace dai
