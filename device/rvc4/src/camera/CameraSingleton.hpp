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
    static constexpr std::array<uint16_t, 2> CBA_EEPROM_ADDRESSES{0x56, 0x57};  // Sensor RESET drives EEPROM A0: 0x56 if low or floating, 0x57 if high
    struct CbaEeprom {
        int bus = -1;
        uint16_t address = 0;
    };
    std::optional<CbaEeprom> getCbaEeprom(dai::CameraBoardSocket socket);

    static constexpr uint16_t CCM_EEPROM_ADDRESS = 0x50;
    int getCcmEepromBus(dai::CameraBoardSocket socket);  // Throws with the reason (e.g. I2C bus shared with other sockets) when it can't be accessed
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
