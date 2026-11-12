#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <set>

// shared
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"

// project
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "utilities/FsyncRole.hpp"

// #define FSYNC_DONT_WRITE

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

    /**
     * Compare the intensity just set with the brightness ceiling the OS power profile applies to this LED.
     * @return A warning for the first limited request of an episode, std::nullopt otherwise.
     *         An episode ends when a request fits under the ceiling or is 0.
     */
    std::optional<std::string> checkBrightnessCeiling(float intensity);
};

struct CameraExposureMaster {
    CameraBoardSocket socket;
    int cameraId;
};

class FsyncController {
    // private constructor
    FsyncController();
    ~FsyncController() = default;

    mutable std::recursive_mutex mtx;
    std::unordered_map<int, FsyncDescription> fsync;

    // config count, to be updated only under `mtx` lock
    int configCount = 0;
    float fps = -1;  // All pins have the same FPS (except ADIM on leds has 10x higher)
    std::optional<RoleState> m_roleState;
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

    // Sets FSync FPS of specified source by ID, which will start after calling 'start' method
    // Each user shouldn't call this more than once, each call counts, affecting 'start'
    void setFps(int id, float fps);
    // Retrieves FPS of source if already specified
    float getFps(int id);

    // starts all fsync sources. Each user needs to call it, actual start happens at the last call
    void start();
    // stops all fsync sources
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

    /**
     * Set the external frame sync role.
     * If set to AUTO_DETECT, the role will be determined based on the state of the M8 connector:
     *  - If we detect the M8 connector is connected, we set the role to SLAVE.
     *  - If we detect the M8 connector is disconnected, we set the role to MASTER.
     * If set to any other role, that role will be used directly.
     * NOTE: We can detect if the M8 connector is connected only on slave devices.
     *       This detection relies on the state of one GPIO pin which is pulled low only on slave devices when the M8 connector is connected.
     * WARNING: This function must be called before any calls to setFps() or start().
     */
    void setExternalFrameSyncRole(dai::ExternalFrameSyncRole role);

    /**
     * Apply AUTO_DETECT unless a role is already applied. Called when a pipeline starts,
     * so the STM stays in CONFIG 0 with the FSYNC pin free until the host needs a role.
     */
    void applyPendingRole();

    bool externalRoleForced() const;

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
    
    /**
     * Read STM frequency measurement of the external frame sync input pin.
     * @return Frequency in Hz.
     */
    float slaveMeasureFrequency();

    FsyncControllerLed dotProjector;
    FsyncControllerLed floodLight;

    /**
     * The applied role. Before a role is applied, the role AUTO_DETECT would choose, read without writing to the STM.
     */
    dai::ExternalFrameSyncRole getExternalFrameSyncRole() const;

    /**
     * Query the STM if there is a valid signal present on the external frame sync input pin.
     */
    bool isExternalFrameSyncSignalPresent();
   private:
    int getDutyCycle(int stmFd);

    /**
     * Query the STM for the closest frequency it can generate to the requested frequency.
     */
    float getClosestFpsIoctl(float wantedFrequency);

    float stopSetAndQueryActualFrequency(int pinId, float wantedFrequency);

    SetInfoResult writeConfig(uint32_t configReg);

    void updateStmLedExposure(uint32_t exposureNs);
    void updateExternalStrobeExposure(uint32_t exposureNs);
};

}  // namespace gate

}  // namespace dai
