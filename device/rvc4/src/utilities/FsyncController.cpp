#include "FsyncController.hpp"

// std
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// libraries
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

// project
#include <fsync-stm/fsync-stm.h>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "utilities/AuxConnector.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/DeviceModel.hpp"
#include "utilities/CameraDiscovery.hpp"
#include "utilities/OSUtils.hpp"

namespace dai {
namespace gate {

namespace {
    uint32_t constexpr FSYNC_STM_FPS_FIXED_POINT = 16;
    uint32_t constexpr FSYNC_STM_FPS_FIXED_POINT_SCALE = (1UL << FSYNC_STM_FPS_FIXED_POINT);

    uint32_t constexpr LED_BRIGHTNESS_MAX = 255;
}

FsyncControllerLed::FsyncControllerLed(const char* ledName, const char* label, std::recursive_mutex &mtx) : mtx(mtx), ledName(ledName), label(label) {

    std::unique_lock<std::recursive_mutex> lock(mtx);

    const char* FSYNC_STM_LED_NAME = "/dev/fsync-stm:led:";
    fd = open(fmt::format("{}{}", FSYNC_STM_LED_NAME, ledName).c_str(), O_RDWR);
    auto err = errno;

    if(fd == -1) {
        if (err == ENOENT) {
            // If device file not found, no need to log, since the device model does not support this led.
            DAI_INFO("Failed to open fsync_stm led: {}{}. Device probably doesn't have IR leds.", FSYNC_STM_LED_NAME, ledName);
        } else {
            DAI_ERROR("Failed to open fsync_stm led: {}{}. Reason: {} (code: {})", FSYNC_STM_LED_NAME, ledName, strerror(err), err);
        }
    }
}

FsyncControllerLed::~FsyncControllerLed() {
    if(fd >= 0) {
        this->setIntensity(0);
        close(fd);
    }
}

bool FsyncControllerLed::setIntensity(float intensity) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    uint32_t val = static_cast<uint32_t>(intensity * LED_BRIGHTNESS_MAX);
    int ret = ioctl(fd, SET_BRIGHTNESS, &val);
    if(ret == -1) {
        DAI_ERROR("Failed to set intensity for led: {} (code: {})", strerror(errno), errno);
        return false;
    }
    return true;
}

std::optional<std::string> FsyncControllerLed::checkBrightnessCeiling(float intensity) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    uint32_t requested = static_cast<uint32_t>(intensity * LED_BRIGHTNESS_MAX);
    std::optional<uint32_t> ceiling = requested > 0 ? getLedBrightnessCeiling(ledName) : std::nullopt;
    if(!ceiling || requested <= *ceiling) {
        ceilingWarned = false;
        return std::nullopt;
    }
    if(ceilingWarned) {
        return std::nullopt;
    }
    ceilingWarned = true;

    std::optional<std::string> profile = getPowerProfile();
    std::string source = profile ? fmt::format("the {} power profile", *profile) : "the device's power profile";
    if(*ceiling == 0) {
        return fmt::format("{} is disabled by {}, so intensity {:.2f} has no effect", label, source, intensity);
    }
    return fmt::format("{} is limited to intensity {:.2f} by {}, so intensity {:.2f} is reduced",
                       label, static_cast<float>(*ceiling) / LED_BRIGHTNESS_MAX, source, intensity);
}

FsyncController::FsyncController()
:
    dotProjector("ir_dot", "IR dot projector", mtx),
    floodLight("ir_flood", "IR flood light", mtx),
    stmFd(-1),
    m_ioctl_failure(false),
    m_disabled(false),
    m_supportsExternalFrameSyncMode(false),
    m_strobeMinDutyCycle(0.005),
    m_strobeMaxDutyCycle(0.995),
    m_enableExternalStrobe(false),
    m_shutdown(false)
{
    std::unique_lock<std::recursive_mutex> lock(mtx);

    DeviceModel::Model deviceModel = DeviceModel::get();
    std::optional<std::uint32_t> deviceRevision = DeviceModel::getRevision();

    if ((deviceModel == DeviceModel::Model::OAK4_S && deviceRevision.has_value() && deviceRevision.value() == 1)
        || (deviceModel == DeviceModel::Model::FAST_STATION_CAMERA)
    )
    {
        // OAK4-S R3 is presented as OAK4-S R1.
        // OAK4-S R3 does not have fsync lines connected to the CCM.
        // We therefore disable fsync controller for OAK4-S R1 and R3 devices.

        // FAST STATION CAMERA fsync stm was not brought up, since they use only one camera and no external fsync.
        // We therefore disable fsync controller for FAST STATION CAMERA devices.
        m_disabled = true;
        DAI_INFO("FSYNC controller is not supported on OAK4-S R1 and R3 devices.");
        return;
    }

    // Hog the file descriptor, because shutdown hook will be executed when file is closed
    stmFd = open("/dev/fsync-stm", O_RDWR);
    auto err = errno;
    if(stmFd < 0) {
        m_ioctl_failure = true;
        m_openErrno = err;
        DAI_ERROR("Failed to open the fsync-stm device {} (code: {})", strerror(err), err);
    }

    float test = 30.0f;
    auto ret = ioctl(stmFd, GET_CLOSEST_FREQUENCY, &test);
    err = errno;
    if (ret == -1) {
        if (err == EBADRQC) {
            // Ioctl failed, because external fsync is not supported on this OS.
            m_supportsExternalFrameSyncMode = false;
        } else {
            m_ioctl_failure = true;
            DAI_ERROR("Failed to detect if the device supports external Fsync slave mode. {} (code: {})", strerror(err), err);
        }
    } else {
        m_supportsExternalFrameSyncMode = true;
    }

    writeConfig(FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT);

    if (m_supportsExternalFrameSyncMode) {
        // We need to turn off external fsync output when we finish.
        // If we don't do this and we set a different device to be external Fsync master on the next run,
        // there will be two masters driving the Fsync line, and slaves won't be able to lock on.
        // This shutdown hook is executed by the OS if depthai_device crashes.
        struct fsync_stm_ioctl_shutdown_hook_data shutdown_hooks = {0};
        shutdown_hooks.enable_hook = 1;
        shutdown_hooks.restore_config_reg = FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT;
        auto ret = ioctl(stmFd, SHUTDOWN_HOOK, &shutdown_hooks);
        if(ret == -1) {
            DAI_ERROR("Failed to write shutdown hooks to the fsync-stm device: {} ({})", strerror(errno), errno);
        }
    }
}

void FsyncController::addFsyncConsumer(dai::CameraBoardSocket socket)
{
    m_fsyncConsumers.emplace(socket);
}

void FsyncController::setExternalFrameSyncRole(dai::ExternalFrameSyncRole role) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    std::optional<M8ConnectorState> m8State;
    if(role == ExternalFrameSyncRole::AUTO_DETECT && m_supportsExternalFrameSyncMode) {
        m8State = getM8State();
        if(!m8State) {
            DAI_WARN("Failed to determine if the device is running as a slave or master, assuming master.");
        }
    }
    ExternalFrameSyncRole resolved = resolveRole(role, m_supportsExternalFrameSyncMode, m8State);
    if(role == ExternalFrameSyncRole::SLAVE && resolved != ExternalFrameSyncRole::SLAVE) {
        DAI_WARN("This OS version doesn't support external Fsync slave mode. Please update your OS.");
        DAI_WARN("External frame sync role set to MASTER.");
    }

    bool isExplicit = role != ExternalFrameSyncRole::AUTO_DETECT;
    if(isExplicit && m_openErrno) {
        m_roleState = RoleState{role, resolved, DeviceOpenFailure{*m_openErrno}};
    } else if(isExplicit && m_ioctl_failure) {
        m_roleState = RoleState{role, resolved, EarlierControllerFailure{}};
    } else {
        m_roleState = RoleState{role, resolved, writeConfig(configFor(resolved))};
    }
    if(auto warning = autoWarning(*m_roleState)) {
        DAI_WARN("{}", *warning);
    }
    if(auto error = setRoleError(*m_roleState)) {
        throw std::runtime_error(*error);
    }
}

void FsyncController::applyPendingRole() {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    if(!m_roleState) {
        setExternalFrameSyncRole(ExternalFrameSyncRole::AUTO_DETECT);
    }
}

namespace {

PinUnavailable describeBusyPin() {
    auto pins = aux::readConnector();
    if(const auto* read = std::get_if<std::vector<aux::Pin>>(&pins)) {
        return aux::busyFsyncPin(*read);
    }
    int err = std::get<int>(pins);
    if(err != ENOENT) {
        DAI_INFO("Cannot read the aux connector to name the holder of the FSYNC pin: {} ({})", strerror(err), err);
    }
    return PinUnavailable{PinProblem::Busy};
}

std::optional<uint32_t> readConfig(int stmFd) {
    struct fsync_stm_ioctl_info_data info = {0};
    if(ioctl(stmFd, GET_INFO, &info) != 0) {
        return std::nullopt;
    }
    return info.config_reg;
}

}  // namespace

SetInfoResult FsyncController::writeConfig(uint32_t configReg) {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    if(stmFd < 0) {
        return Applied{};
    }
    struct fsync_stm_ioctl_info_data infoData = {0};
    infoData.config_reg = configReg;
    if(ioctl(stmFd, SET_INFO, &infoData) != 0) {
        int err = errno;
        // CONFIG 0 never asks the aux connector for the FSYNC pin
        if(configReg != FSYNC_STM_CONFIG_MASTER_MODE_WITH_INPUT) {
            bool linked = aux::fsyncLinkedToConnector();
            if(auto problem = classifySetInfoError(err, linked, linked && aux::connectorBound(), readConfig(stmFd))) {
                return *problem == PinProblem::Busy ? describeBusyPin() : PinUnavailable{*problem};
            }
        }
        m_ioctl_failure = true;
        DAI_ERROR("Failed to write info to the fsync-stm device: {} ({})", strerror(err), err);
        return ControllerFailure{err};
    }
    return Applied{};
}

dai::ExternalFrameSyncRole FsyncController::getExternalFrameSyncRole() const {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    if(m_roleState) {
        return effectiveRole(*m_roleState);
    }
    return resolveRole(ExternalFrameSyncRole::AUTO_DETECT, m_supportsExternalFrameSyncMode, getM8State());
}

std::optional<RoleState> FsyncController::getRoleState() const {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    return m_roleState;
}

FsyncController& FsyncController::getInstance() {
    static FsyncController instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

static std::optional<int> fsyncDutyCycle;
int FsyncController::getDutyCycle(int stmFd) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");
    if(fsyncDutyCycle.has_value()) {
        return fsyncDutyCycle.value();
    }
    struct fsync_stm_ioctl_info_data stmInfo {};
    if(-1 == ioctl(stmFd, GET_INFO, &stmInfo)) {
        DAI_ERROR("Failed to read fsync-stm firmware version. {}. Defaulting fsync duty cycle to 1!", strerror(errno));
        return 1;
    }
    if(9 <= stmInfo.fw_ver) {
        fsyncDutyCycle = 8;
    } else {
        fsyncDutyCycle = 1;
    }
    DAI_CHECK_IN(fsyncDutyCycle.has_value());
    return fsyncDutyCycle.value();
}

float FsyncController::getFps(int id) {
    // Lock for this length of operation
    if(fsync.count(id) > 0) {
        return fsync[id].fps;
    }
    return -1;
}

void FsyncController::registerChannelForExposureUpdates(int id) {
    m_updatedChannels.emplace(id);
}

void FsyncController::setFps(int id, float fps) {
    if(getExternalFrameSyncRole() == ExternalFrameSyncRole::SLAVE) {
        std::unique_lock<std::recursive_mutex> lock(mtx);
        DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");

        // In slave mode, we still need to do the ioctl. Only polarity/duty are used, fps is ignored. We must also set this->fps in order for exposure updates to work.
        struct fsync_stm_ioctl_pin_data ioctl_data = {
            .id = id,
            .fps = *((int*)&fps),
            .polarity = 0,
            .duty = getDutyCycle(stmFd),
            .locked = false,
        };

        if(ioctl(stmFd, SET_PIN_STATE, &ioctl_data) == -1) {
            m_ioctl_failure = true;
            DAI_ERROR("Failed to perform ioctl() call to fsync-stm kernel driver and set FPS!: {} ({})", strerror(errno), errno);
        }
        this->fps = fps;
        return;
    }

    // Lock for this length of operation
    std::unique_lock<std::recursive_mutex> lock(mtx);
    configCount++;

    // Default-constructs if it doesn't exist (may be called by multiple users with same `id`)
    FsyncDescription& desc = fsync[id];
    desc.isFsyncStm = std::filesystem::exists("/dev/fsync-stm");  // if /dev/fsync-stm exists, then we have fsync STM controller available
    desc.id = id;
    desc.fps = fps;

    if(desc.isFsyncStm) {
        DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");

        struct fsync_stm_ioctl_pin_data ioctl_data = {
            .id = id,
            .fps = *((int*)&fps),
            .polarity = 0,
            .duty = getDutyCycle(stmFd),
            .locked = false,
        };

        if(ioctl(stmFd, SET_PIN_STATE, &ioctl_data) == -1) {
            m_ioctl_failure = true;
            DAI_ERROR("Failed to perform ioctl() call to fsync-stm kernel driver and set FPS!: {} ({})", strerror(errno), errno);
        }
        this->fps = fps;
    } else {
        DAI_ERROR("No valid Fsync source!\n");
    }

    printf("Using FSYNC STM?: %d\n", desc.isFsyncStm);
}

bool FsyncController::isExternalFrameSyncSignalPresent()
{
    std::unique_lock<std::recursive_mutex> lock(mtx);

    DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");

    struct fsync_stm_ioctl_info_data fsyncStmData;
    int ret = ioctl(stmFd, GET_INFO, &fsyncStmData);
    if(-1 == ret) {
        m_ioctl_failure = true;
        throw std::runtime_error("Failed to get fsync stm info. " + std::to_string(errno) + " " + strerror(errno));
    }

    return fsyncStmData.in1_present == 1;
}

float FsyncController::getClosestFps(float wantedFrequency)
{
#ifdef FSYNC_DONT_WRITE
    return wantedFrequency;
#else
    if(!m_supportsExternalFrameSyncMode) {
        return stopSetAndQueryActualFrequency(2, wantedFrequency);
    }
    
    return getClosestFpsIoctl(wantedFrequency);
#endif
}


float FsyncController::getClosestFpsIoctl(float wantedFrequency)
{
    std::unique_lock<std::recursive_mutex> lock(mtx);

    DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");

    printf("Querying closest fps to %f.\n", wantedFrequency);

    int ret = ioctl(stmFd, GET_CLOSEST_FREQUENCY, &wantedFrequency);
    if(-1 == ret) {
        m_ioctl_failure = true;
        throw std::runtime_error("Failed to query the stm for closest fps. "
             + std::to_string(errno) + " " + std::string(strerror(errno)));
    } else {
        printf("Closest fps is %f.\n", wantedFrequency);
    }

    return wantedFrequency;
}

float FsyncController::stopSetAndQueryActualFrequency(int pinId, float wantedFrequency) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    // if(m_role == ExternalFrameSyncRole::SLAVE) {
    //     return wantedFrequency;
    // }
    if(stmFd < 0) {
        return wantedFrequency;
    }

    struct fsync_stm_ioctl_pin_data pinData = {
        .id = pinId, .fps = *((int*)&(wantedFrequency)), .polarity = 0, .duty = getDutyCycle(stmFd), .locked = false};
    int ret = ioctl(stmFd, SET_PIN_STATE, &pinData);
    if(-1 == ret) {
        m_ioctl_failure = true;
        printf("Failed to set fsync pin state for pin: %d. %d %s\n", pinId, errno, strerror(errno));
        return 0.0f;
    } else {
        printf("FSYNC set pin successful\n");
    }
    struct fsync_stm_ioctl_info_data fsyncStmData;
    ret = ioctl(stmFd, GET_INFO, &fsyncStmData);
    if(-1 == ret) {
        m_ioctl_failure = true;
        printf("Failed to get fsync stm info. %d %s\n", ret, strerror(errno));
        return 0.0f;
    }

    // TODO(stasbucik) for slave, we must wait until fsyncStmData.in1_present == 1 to get good actual frequency 

    printf("Got stm actual frequency %f for wanted frequency %f\n", fsyncStmData.actual_out_freq, wantedFrequency);
    return fsyncStmData.actual_out_freq;
}

float FsyncController::slaveMeasureFrequency() {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");
    struct fsync_stm_ioctl_info_data fsyncStmData;
    int ret = ioctl(stmFd, GET_INFO, &fsyncStmData);

    if (ret == -1) {
        m_ioctl_failure = true;
        std::stringstream str;
        str << "Unable to measure frequency.: " << strerror(errno) << " (" << errno << ")";
        throw std::runtime_error(str.str());
    }

    if (fsyncStmData.in1_present != 1) {
        throw std::runtime_error("Signal not present on input.");
    }

    return fsyncStmData.in1_freq;
}

void FsyncController::start() {
    printf("Starting FSYNC controller.\n");
    // Lock for this length of operation
    std::unique_lock<std::recursive_mutex> lock(mtx);

    // 'dequeue' config state, and only when last user calls start, then do start
    // (as we don't want signal generation to start prior to all cameras being configured)
    configCount--;
    if(configCount > 0) return;

    // Nothing else to do, fsync-stm is started when setFps is called
}
void FsyncController::cleanup() {
    std::unique_lock<std::recursive_mutex> lock(mtx);
    if (stmFd > -1) {
        close(stmFd);
        stmFd = -1;
        m_shutdown = true;
    }
}

void FsyncController::stop() {
    // Lock for this length of operation
    std::unique_lock<std::recursive_mutex> lock(mtx);
    if (dotProjector.isOk()) {
        dotProjector.setIntensity(0);
    }
    if (floodLight.isOk()) {
        floodLight.setIntensity(0);
    }
}

void FsyncController::setExternalStrobeRelativeLimits(float minDutyCycle, float maxDutyCycle) {
        if (minDutyCycle < 0.f || minDutyCycle > 1.f) {
            throw std::invalid_argument("Invalid min duty cycle value. Must be between 0 and 1.");
        }
        if (maxDutyCycle < 0.f || maxDutyCycle > 1.f) {
            throw std::invalid_argument("Invalid max duty cycle value. Must be between 0 and 1.");
        }
        if (minDutyCycle > maxDutyCycle) {
            throw std::invalid_argument("Invalid min duty cycle value. Must be less than max duty cycle value.");
        }
        m_strobeMinDutyCycle = minDutyCycle;
        m_strobeMaxDutyCycle = maxDutyCycle;
    }


void FsyncController::setLedExposureMasterCamera(CameraBoardSocket socket) {
    auto ret = CameraDiscovery::getInstance().getCameraId(socket);
    if (ret == -1) {
        throw std::runtime_error("Couldn't find camera id for socket " + toString(socket));
    }

    m_ledExposureMasterCamera = {
        .socket = socket,
        .cameraId = ret
    };
    DAI_INFO("Setting led exposure master camera to socket {}, camera id {}", toString(socket), ret);
}

void FsyncController::setExternalStrobeExposureMasterCamera(CameraBoardSocket socket) {
    auto ret = CameraDiscovery::getInstance().getCameraId(socket);
    if (ret == -1) {
        throw std::runtime_error("Couldn't find camera id for socket " + toString(socket));
    }

    m_externalStrobeExposureMasterCamera = {
        .socket = socket,
        .cameraId = ret
    };
    DAI_INFO("Setting external strobe exposure master camera to socket {}, camera id {}", toString(socket), ret);
}

std::optional<CameraBoardSocket> FsyncController::getLedExposureMasterCamera() {
    if (!m_ledExposureMasterCamera.has_value()) {
        return std::nullopt;
    }
    return m_ledExposureMasterCamera->socket;
}

std::optional<CameraBoardSocket> FsyncController::getExternalStrobeExposureMasterCamera() {
    if (!m_externalStrobeExposureMasterCamera.has_value()) {
        return std::nullopt;
    }
    return m_externalStrobeExposureMasterCamera->socket;
}

void FsyncController::updatelCameraExposure(uint32_t exposureNs, int cameraId) {
    if (m_ledExposureMasterCamera.has_value() &&
        cameraId == m_ledExposureMasterCamera->cameraId)
    {
        updateStmLedExposure(exposureNs);
    }
    
    if (m_externalStrobeExposureMasterCamera.has_value() &&
        cameraId == m_externalStrobeExposureMasterCamera->cameraId)
    {
        updateExternalStrobeExposure(exposureNs);
    }
}

void FsyncController::updateStmLedExposure(uint32_t exposureNs) {
    std::unique_lock<std::recursive_mutex> lock(mtx);

    if (m_shutdown && stmFd < 0) {
        return;
    } else {
        DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");
    }

    if (m_supportsExternalFrameSyncMode) {
        struct fsync_stm_ioctl_camera_data_fixed_point cameraDataFixedPoint = {
            .fps = static_cast<uint32_t>(round(fps * FSYNC_STM_FPS_FIXED_POINT_SCALE)),
            .exposure_time = exposureNs,
            .fixed_point_scale = FSYNC_STM_FPS_FIXED_POINT_SCALE
        };
        
        if(ioctl(stmFd, SET_CAMERA_DATA_FIXED_POINT, &cameraDataFixedPoint) == -1) {
            m_ioctl_failure = true;
            DAI_ERROR("Failed to perform ioctl() call to fsync-stm kernel driver and set camera exposure: {} ({})", strerror(errno), errno);
        }
    } else {
        struct fsync_stm_ioctl_camera_data cameraData = {
            .fps = static_cast<uint32_t>(round(fps)),
            .exposure_time = exposureNs
        };

        if(ioctl(stmFd, SET_CAMERA_DATA, &cameraData) == -1) {
            m_ioctl_failure = true;
            DAI_ERROR("Failed to perform ioctl() call to fsync-stm kernel driver and set camera exposure: {} ({})", strerror(errno), errno);
        }
    }
}

void FsyncController::updateExternalStrobeExposure(uint32_t exposureNs) {
    if (!m_enableExternalStrobe) {
        return;
    }

    std::unique_lock<std::recursive_mutex> lock(mtx);

    if (m_shutdown && stmFd < 0) {
        return;
    } else {
        DAI_CHECK_V(stmFd >= 0, "Failed to open the fsync-stm device");
    }
    
    for (auto id : m_updatedChannels) {
        float fps = getFps(id);

        int duty = static_cast<int>(round(2048.f * float(exposureNs) * 1e-9 * fps));
        duty = 2048 - duty;

        duty = std::min(static_cast<int>(ceil(m_strobeMaxDutyCycle * 2048.f)), duty);
        duty = std::max(static_cast<int>(floor(m_strobeMinDutyCycle * 2048.f)), duty);
        
        struct fsync_stm_ioctl_pin_data ioctl_data = {
            .id = id,
            .fps = *((int*)&fps),
            .polarity = 0,
            .duty = duty,
            .locked = false,
        };

        if(ioctl(stmFd, SET_PIN_STATE, &ioctl_data) == -1) {
            m_ioctl_failure = true;
            DAI_ERROR("Failed to perform ioctl() call to fsync-stm kernel driver and set FPS!: {} ({})", strerror(errno), errno);
        }
    }
}

}  // namespace gate
}  // namespace dai
