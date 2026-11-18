#include "LuxonisSidechannel.hpp"

#include <dlfcn.h>
#include <fcntl.h>
#include <spdlog/spdlog.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>

#include "utilities/ErrorMacros.hpp"
#include "utilities/FsyncController.hpp"
#include "utilities/DeviceTreeDiscovery.hpp"

static constexpr const char* FIFO_PATH = LUX_SC_REVERSE_FIFO_PATH;
std::unique_ptr<LuxonisSidechannel> LuxonisSidechannel::instance = nullptr;

static void clearPipe(int fd) {
    char buffer[64];
    ssize_t count = 1;
    while(count > 0) {
        count = read(fd, buffer, sizeof(buffer));
        if(count == -1) {
            if(errno == EAGAIN) {
                // pipe is empty
                break;
            } else {
                std::cerr << "Failed while emptying the pipe: " << strerror(errno) << "\n";
                // non fatal error continue
                break;
            }
        }
    }
}

template <typename T>
static bool readMessageAfterHeaderRead(T& msg, int fd) {
    size_t bytesToRead = sizeof(msg) - sizeof(lux_sc_type_t);
    auto res = read(fd, (uint8_t*)&msg + sizeof(lux_sc_type_t), sizeof(msg) - sizeof(lux_sc_type_t));
    if(res == -1) {
        perror("Something went wrong when reading message from pipe");
        return false;
    }
    if(res != bytesToRead) {
        perror("Message to short for it's type");
        return false;
    }
    return true;
}

extern "C" {
static void exposureUpdateCb(const lux_sc_exposure_update_t& msg) {
    LuxonisSidechannel::getInstance().handleExposureUpdate(msg);
}
}

LuxonisSidechannel::LuxonisSidechannel(void* halDlopenHandle) {
    DAI_CHECK_IN(halDlopenHandle);
    this->halDlopenHandle = halDlopenHandle;
    auto scVersionGet = (lux_sc_version_get_cb_t)dlsym(halDlopenHandle, "lux_sc_version_get");
    if(scVersionGet != nullptr) {
        // This version supports direct calling
        protocolVersion = scVersionGet();
        std::cout << "Running camera HAL sidechannel shared library version: " << protocolVersion->major << "." << protocolVersion->minor
                  << " and depthai-device build container version: " << LUX_SC_VERSION_MAJOR << "." << LUX_SC_VERSION_MINOR << "\n"
                  << std::flush;
        scExposureUpdateListen = (lux_sc_exposure_update_listen_t)dlsym(halDlopenHandle, "lux_sc_exposure_update_listen");
        DAI_CHECK_IN(scExposureUpdateListen);
        scExposureUpdateListen(exposureUpdateCb);
        ops.camera_mode_set = (lux_sc_camera_mode_set_cb_t)dlsym(halDlopenHandle, "lux_sc_camera_mode_set");
        DAI_CHECK_IN(ops.camera_mode_set);
        ops.camera_sockets_config = (lux_sc_camera_sockets_config_cb_t)dlsym(halDlopenHandle, "lux_sc_camera_sockets_config");
        DAI_CHECK_IN(ops.camera_sockets_config);
        ops.camera_lens_pos_set = (lux_sc_camera_lens_pos_set_cb_t)dlsym(halDlopenHandle, "lux_sc_camera_lens_pos_set");
        DAI_CHECK_IN(ops.camera_lens_pos_set);
        ops.sensor_register_control = (lux_sc_sensor_register_control_cb_t)dlsym(halDlopenHandle, "lux_sc_sensor_register_control");
        DAI_CHECK_IN(ops.sensor_register_control);
        ops.camera_tuning_update = (lux_sc_camera_tuning_update_cb_t)dlsym(halDlopenHandle, "lux_sc_camera_tuning_update");
        DAI_CHECK_IN(ops.camera_tuning_update);
        ops.camera_socket_for_csi_get = (lux_sc_camera_socket_for_csi_get_cb_t)dlsym(halDlopenHandle, "lux_sc_camera_socket_for_csi_get");
        DAI_CHECK_IN(ops.camera_socket_for_csi_get);
        ops.sensors_get = (lux_sc_sensors_get_cb_t)dlsym(halDlopenHandle, "lux_sc_sensors_get");
        ops.sensor_get_error_gpio = (lux_sc_sensor_get_error_gpio_cb_t)dlsym(halDlopenHandle, "lux_sc_sensor_get_error_gpio");
        ops.sensor_err_cb_add = (lux_sc_sensor_err_cb_add_t)dlsym(halDlopenHandle, "lux_sc_sensor_err_cb_add");
        ops.sensor_err_cb_rm = (lux_sc_sensor_err_cb_rm_t)dlsym(halDlopenHandle, "lux_sc_sensor_err_cb_rm");
    } else {
        // Note: do not put this in a initializer list, since the thread will start before the object is fully constructed
        sidechannelThread = std::thread(&LuxonisSidechannel::listenToLuxonisSidechannel, this);
    }
}

LuxonisSidechannel::~LuxonisSidechannel() {
    if(scExposureUpdateListen) {
        scExposureUpdateListen(nullptr);
    }
}

bool LuxonisSidechannel::isV1() {
    return !protocolVersion.has_value() || protocolVersion->major < 2;
}

void LuxonisSidechannel::listenToLuxonisSidechannel() {
    while(true) {
        lux_sc_version_query_t versionQueryMsg;
        sendMessage(versionQueryMsg);
        int fd = open(LUX_SC_FIFO_PATH, O_RDONLY);
        if(fd < 0) {
            perror("Failed to open fifo pipe, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        while(true) {
            lux_sc_type_t msgType{};
            auto count = read(fd, &msgType, sizeof(lux_sc_type_t));
            if(count == -1) {
                spdlog::debug("Something went wrong when reading message type from pipe");
                close(fd);
                break;
            }
            if(count == 0) {
                std::cout << "The camera HAL service closed the fifo pipe " << LUX_SC_FIFO_PATH << "\n" << std::flush;
                close(fd);
                break;
            }
            if(count != sizeof(lux_sc_type_t)) {
                spdlog::debug("Message type to short");
                close(fd);
                break;
            }
            // printf("Got message type: %d\n", msgType);
            if(msgType == LUX_SC_EXPOSURE_UPDATE) {
                lux_sc_exposure_update_t msg;
                auto suc = readMessageAfterHeaderRead(msg, fd);
                if(!suc) {
                    close(fd);
                    break;
                }

                if (dai::gate::FsyncController::getInstance().isOk()) {
                    dai::gate::FsyncController::getInstance().updatelCameraExposure(msg.exposure_ns, msg.camera_id);
                }
                /*
                std::cout << "Read a exposure update message that said:\n"
                          << "timestamp: " << msg.timestamp.tv_sec << "s " << msg.timestamp.tv_nsec << "ns\n"
                          << "camera: " << msg.camera_id << "\n"
                          << "exposure: " << msg.exposure_ns << "ns\n"
                          << "previous exposure: " << msg.previous_exposure_ns << "ns\n"
                          << std::flush;
                          */
            } else if(msgType == LUX_SC_VERSION_RESULT) {
                lux_sc_version_result_t msg;
                auto suc = readMessageAfterHeaderRead(msg, fd);
                if(!suc) {
                    close(fd);
                    break;
                }
                std::cout << "Running camera HAL sidechannel server version: " << msg.major << "." << msg.minor
                          << " and client version: " << LUX_SC_VERSION_MAJOR << "." << LUX_SC_VERSION_MINOR << "\n"
                          << std::flush;
            } else if(msgType == LUX_SC_VERSION_QUERY) {
                lux_sc_version_query_t msg;
                auto suc = readMessageAfterHeaderRead(msg, fd);
                if(!suc) {
                    close(fd);
                    break;
                }
                lux_sc_version_result_t versionResultMsg;
                std::cout << "Sending version result to server: " << versionResultMsg.type << " | " << versionResultMsg.major << "." << versionResultMsg.minor
                          << "\n"
                          << std::flush;
                sendMessage(versionResultMsg);
            } else {
                // This is pretty bad because we didn't read the whole message but read
                // the header so clear the message / whole pipe
                clearPipe(fd);
                spdlog::debug("Got unknown message type");
                close(fd);
                break;
            }
        }
    }
};

void LuxonisSidechannel::initializeFifo() {
    // if protocol version < 2 it is normal that we first have to sendMessage() to query the version
    if(protocolVersion.has_value()) {
        // LuxonisSidechannel::sendMessage() or LuxonisSidechannel::initializeFifo() should never be called on sidechannel version 2.0+
        // as we don't use named pipes anymore but dlopen() and dlsym() to communicate between the camera HAL and depthai-device
        DAI_CHECK_IN(protocolVersion->major < 2);
    }
    std::lock_guard lock(fifoFdLock);
    if(fifoFd == std::nullopt) {
        struct stat statStruct {};
        if(stat(FIFO_PATH, &statStruct) == -1) {
            auto res = mkfifo(FIFO_PATH, 0666);
            DAI_CHECK_V(res != -1, "Couldn't create FIFO named pipe. Error was: %s", strerror(errno));
        }
        int fd = open(FIFO_PATH, O_RDWR | O_NONBLOCK);
        DAI_CHECK_V(fd >= 0, "Pipe opening failed: %s", strerror(errno));
        fifoFd = fd;
    }
}

void LuxonisSidechannel::writeToPipe(const void* data, size_t dataLength) {
    std::lock_guard lock(fifoFdLock);
    DAI_CHECK_IN(fifoFd.has_value());
    auto res = write(*fifoFd, data, dataLength);
    DAI_CHECK_V(res != -1, "Something went wrong when writting to pipe: %s", strerror(errno));
}

void LuxonisSidechannel::setCameraMode(lux_sc_camera_mode_set_t cameraMode) {
    std::cout << "sending camera mode: " << (int)cameraMode.fsync_mode << "\n" << std::flush;
    auto cb = ops.camera_mode_set;
    if(cb) {
        cb(cameraMode);
    } else {
        sendMessage(cameraMode);
    }
}

void LuxonisSidechannel::initialize(void* halDlopenHandle) {
    instance = std::make_unique<LuxonisSidechannel>(halDlopenHandle);
}

void LuxonisSidechannel::handleExposureUpdate(const lux_sc_exposure_update_t& msg) {
    /*
    std::cout << "Read a exposure update message V2 that said:\n"
              << "timestamp: " << msg.timestamp.tv_sec << "s " << msg.timestamp.tv_nsec << "ns\n"
              << "camera: " << msg.camera_id << "\n"
              << "exposure: " << msg.exposure_ns << "ns\n"
              << "previous exposure: " << msg.previous_exposure_ns << "ns\n"
              << std::flush;
              */
    if (dai::gate::FsyncController::getInstance().isOk()) {
        dai::gate::FsyncController::getInstance().updatelCameraExposure(msg.exposure_ns, msg.camera_id);
    }
}

void LuxonisSidechannel::earlyInit() {
    int rc = setenv("LUX_SC_CLIENT_VERSION_MAJOR", std::to_string(LUX_SC_VERSION_MAJOR).c_str(), 1);
    DAI_CHECK_IN(rc == 0);
    rc = setenv("LUX_SC_CLIENT_VERSION_MINOR", std::to_string(LUX_SC_VERSION_MINOR).c_str(), 1);
    DAI_CHECK_IN(rc == 0);
}
