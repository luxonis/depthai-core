#include "depthai/device/FsyncController.hpp"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace dai {

FsyncController::FsyncController() = default;
FsyncController::~FsyncController() = default;

int FsyncController::readInfoData(FsyncStmIoctlInfoData& infoData) {
    int fd = open(FSYNC_STM_DEVICE_PATH, O_RDWR);
    if(fd < 0) return -1;

    int ret = ioctl(fd, CMD_GET_INFO, &infoData);
    close(fd);
    
    return ret;
}

int FsyncController::readPinData(int pinId, FsyncStmIoctlPinData& pinData) {
    int fd = open(FSYNC_STM_DEVICE_PATH, O_RDWR);
    if(fd < 0) return -1;

    pinData.id = pinId;
    
    int ret = ioctl(fd, CMD_GET_PIN_STATE, &pinData);
    close(fd);

    return ret;
}

int FsyncController::readLedData(int ledId, FsyncStmIoctlLedData& ledData) {
    int fd = open(FSYNC_STM_DEVICE_PATH, O_RDWR);
    if(fd < 0) return -1;

    ledData.id = ledId;

    int ret = ioctl(fd, CMD_GET_LED, &ledData);
    close(fd);

    return ret;
}

int FsyncController::writePinData(const FsyncStmIoctlPinData& pinData) {
    int fd = open(FSYNC_STM_DEVICE_PATH, O_RDWR);
    if(fd < 0) return -1;

    int ret = ioctl(fd, CMD_SET_PIN_STATE, const_cast<FsyncStmIoctlPinData*>(&pinData));
    close(fd);

    return ret;
}

int FsyncController::writeCameraData(const FsyncStmIoctlCameraData& cameraData) {
    int fd = open(FSYNC_STM_DEVICE_PATH, O_RDWR);
    if(fd < 0) return -1;

    int ret = ioctl(fd, CMD_SET_CAMERA_DATA, const_cast<FsyncStmIoctlCameraData*>(&cameraData));
    close(fd);

    return ret;
}

int FsyncController::writeInfoData(const FsyncStmIoctlInfoData& infoData) {
    int fd = open(FSYNC_STM_DEVICE_PATH, O_RDWR);
    if(fd < 0) return -1;

    int ret = ioctl(fd, CMD_SET_INFO, const_cast<FsyncStmIoctlInfoData*>(&infoData));
    close(fd);

    return ret;
}

} // namespace dai
