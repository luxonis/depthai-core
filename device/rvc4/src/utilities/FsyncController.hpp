#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <set>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"

#include "depthai/common/ExternalFrameSyncRoles.hpp"

namespace dai {
namespace gate {

struct FsyncDescription {
    int id;
    bool isFsyncStm;
    float fps;
};

class FsyncControllerLed {
    int fd;

    std::recursive_mutex &mtx;

    std::string ledName;
    std::string label;
    bool ceilingWarned = false;

   public:
    FsyncControllerLed(const char* ledName, const char* label, std::recursive_mutex &mtx);
    ~FsyncControllerLed();
    bool isOk() const {
        return fd >= 0;
    }
    bool setIntensity(float intensity);

    std::optional<std::string> checkBrightnessCeiling(float intensity);  // Warns once until a request fits under the ceiling or is 0; std::nullopt otherwise
};

struct CameraExposureMaster {
    CameraBoardSocket socket;
    int cameraId;
};

class FsyncController {
    FsyncController();
    ~FsyncController() = default;

    mutable std::recursive_mutex mtx;
    std::unordered_map<int, FsyncDescription> fsync;

    int configCount = 0;  // Guarded by mtx
    float fps = -1;  // All pins have the same FPS (except ADIM on leds has 10x higher)
    dai::ExternalFrameSyncRole m_role;
    bool m_roleSetExplicitly;
    std::set<dai::CameraBoardSocket> m_fsyncConsumers;
    int stmFd;
    bool m_ioctl_failure;
    bool m_disabled;
    bool m_supportsExternalFrameSyncMode;

    std::set<int> m_updatedChannels;

    float m_strobeMinDutyCycle;
    float m_strobeMaxDutyCycle;
    bool m_enableExternalStrobe;

    std::optional<CameraExposureMaster> m_ledExposureMasterCamera;
    std::optional<CameraExposureMaster> m_externalStrobeExposureMasterCamera;

    bool m_shutdown;

   public:
    static FsyncController& getInstance();
    FsyncController(FsyncController const&) = delete;
    void operator=(FsyncController const&) = delete;

    void setFps(int id, float fps);  // Call once per user: each call counts for start()
    float getFps(int id);

    void start();  // Each user must call it; the actual start happens at the last call
    void stop();

    void cleanup();

    bool isOk() const {
        std::unique_lock<std::recursive_mutex> lock(mtx);
        return !m_ioctl_failure && !m_disabled;
    }

    bool isDisabled() const {
        std::unique_lock<std::recursive_mutex> lock(mtx);
        return m_disabled;
    }

    bool failed() const {
        std::unique_lock<std::recursive_mutex> lock(mtx);
        return m_ioctl_failure;
    }

    void setLedExposureMasterCamera(CameraBoardSocket socket);
    void setExternalStrobeExposureMasterCamera(CameraBoardSocket socket);
    std::optional<CameraBoardSocket> getLedExposureMasterCamera();
    std::optional<CameraBoardSocket> getExternalStrobeExposureMasterCamera();

    void updatelCameraExposure(uint32_t exposureNs, int cameraId);

    void registerChannelForExposureUpdates(int id);

    void setExternalStrobeRelativeLimits(float minDutyCycle, float maxDutyCycle);

    void setExternalStrobeEnable(bool enable) {
        m_enableExternalStrobe = enable;
    }

    bool getExternalStrobeEnable() const {
        return m_enableExternalStrobe;
    }

    void setExternalFrameSyncRole(dai::ExternalFrameSyncRole role, bool setExplicit);  // Call before setFps()/start(). AUTO_DETECT: SLAVE if M8 connected

    bool isExternalFrameSyncRoleSetExplicitly() const {
        return m_roleSetExplicitly;
    }

    void addFsyncConsumer(dai::CameraBoardSocket socket);

    std::size_t getNumFsyncConsumers() const {
        return m_fsyncConsumers.size();
    }

    bool isFsyncConsumer(dai::CameraBoardSocket socket) const {
        return m_fsyncConsumers.find(socket) != m_fsyncConsumers.end();
    }

    float getClosestFps(float wantedFrequency);

    bool supportsExternalFrameSyncMode() const {
        return m_supportsExternalFrameSyncMode;
    }

    float slaveMeasureFrequency();

    FsyncControllerLed dotProjector;
    FsyncControllerLed floodLight;

    dai::ExternalFrameSyncRole getExternalFrameSyncRole() const {
        return m_role;
    }

    bool isExternalFrameSyncSignalPresent();

   private:
    int getDutyCycle(int stmFd);

    float getClosestFpsIoctl(float wantedFrequency);

    float stopSetAndQueryActualFrequency(int pinId, float wantedFrequency);

    void updateStmLedExposure(uint32_t exposureNs);
    void updateExternalStrobeExposure(uint32_t exposureNs);
};

}  // namespace gate

}  // namespace dai
