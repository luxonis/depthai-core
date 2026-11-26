#pragma once

#include <hardware/camera3.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"

class CameraSingleton {
   public:
    struct SensorConfigInfo {
        int modeIndex = -1;
        dai::CameraSensorConfig mode;
        float fps = -1.0f;
    };

    camera_module_t* cameraModule{nullptr};

    static CameraSingleton& getInstance() {
        static CameraSingleton instance;
        return instance;
    }

    std::vector<camera_info> cameraInfos;

    void registerSensorConfig(dai::CameraBoardSocket socket, dai::CameraControl::FrameSyncMode frameSyncMode, const SensorConfigInfo& sensorConfig);
    bool isFsyncEnabled(dai::CameraBoardSocket socket);
    // Camera board assembly (CBA) calibration EEPROM I2C addresses. The sensor's RESET line drives the EEPROM's A0 pin:
    // 0x56 while RESET is low or floating, 0x57 while it is high
    static constexpr std::array<uint16_t, 2> CBA_EEPROM_ADDRESSES{0x56, 0x57};
    struct CbaEeprom {
        int bus = -1;
        uint16_t address = 0;
    };
    // I2C bus and address of the socket's CBA EEPROM, if one was detected at startup
    std::optional<CbaEeprom> getCbaEeprom(dai::CameraBoardSocket socket);

    // Camera-module (CCM) EEPROM I2C address
    static constexpr uint16_t CCM_EEPROM_ADDRESS = 0x50;
    // I2C bus of the socket's CCM EEPROM. Throws with the reason (e.g. I2C bus shared with other sockets) when it can't be accessed
    int getCcmEepromBus(dai::CameraBoardSocket socket);
    // A camera is active from when the pipeline creates its sensor until the sensor is stopped
    void setCameraActive(dai::CameraBoardSocket socket, bool active);

   private:
    struct RegisteredSensorConfig {
        dai::CameraBoardSocket socket = dai::CameraBoardSocket::AUTO;
        dai::CameraControl::FrameSyncMode frameSyncMode = dai::CameraControl::FrameSyncMode::AUTO;
        SensorConfigInfo sensorConfig;
    };

    CameraSingleton() {
        init();
    }

    mutable std::mutex registeredSensorConfigsMtx;
    std::vector<RegisteredSensorConfig> registeredSensorConfigs;
    mutable std::mutex cbaEepromsMtx;
    std::unordered_map<dai::CameraBoardSocket, CbaEeprom> cbaEeproms;
    mutable std::mutex activeCamerasMtx;
    std::set<dai::CameraBoardSocket> activeCameras;

    void init();
    void probeCbaEeproms();
    void checkCcmEepromBuses();
    std::set<dai::CameraBoardSocket> getActiveCameras();
    camera_module_t* initializeCameraModule();
};
