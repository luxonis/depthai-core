#include "eeprom.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <thread>

#include "ImuModelData.hpp"
#include "camera/CameraSingleton.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraModel.hpp"

#define EEPROM_DATA_OFFSET (128)  // EVM boards have 35 bytes written at the start; do not overwrite them
#define TOTAL_DISTORTION_PARAMETERS (14)

#define EEPROM_MEM_SIZE (4096)
#define EEPROM_FACTORY_OFFSET ((EEPROM_MEM_SIZE - EEPROM_DATA_OFFSET) / 2)

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

constexpr size_t SIZE_OF_CONFIGURATION = EEPROM_FACTORY_OFFSET;

static_assert(sizeof(gen2_eeprom_data) + offsetof(gen2_eeprom_ex_data, board_options) == 1810 + 96);  // Fixed: the bootloader reads version and board_options

static_assert(sizeof(gen2_eeprom_data) + sizeof(gen2_eeprom_ex_data) <= SIZE_OF_CONFIGURATION);

const char* eeprom_status_to_str(eeprom_status status) {
    switch(status) {
        case EEPROM_OK:
            return "EEPROM_OK";
        case EEPROM_ERROR:
            return "EEPROM_ERROR";
        case EEPROM_NO_FACTORY_PERMISSIONS:
            return "EEPROM_NO_FACTORY_PERMISSIONS";
        case EEPROM_NO_PROTECTED_PERMISSIONS:
            return "EEPROM_NO_PROTECTED_PERMISSIONS";
        case EEPROM_INVALID_DATA:
            return "EEPROM_INVALID_DATA";
        case EEPROM_INVALID_AREA:
            return "EEPROM_INVALID_AREA";
        case EEPROM_NOT_AVAILABLE:
            return "EEPROM_NOT_AVAILABLE";
        case EEPROM_OUT_OF_BOUNDS:
            return "EEPROM_OUT_OF_BOUNDS";
        case EEPROM_FIELD_TOO_LONG:
            return "EEPROM_FIELD_TOO_LONG";
        case EEPROM_NO_BOARD_CONFIG:
            return "EEPROM_NO_BOARD_CONFIG";
    }
    return "EEPROM_UNKNOWN_STATUS";
}

#define CAM_OFFSET 1
struct OpenEeprom {  // Index AUTO is the device's own EEPROM; cbaAddress is the one of two CBA addresses it was opened at
    int fd = -1;
    uint16_t cbaAddress = 0;
};
static constexpr size_t EEPROM_FD_COUNT = CAM_OFFSET + static_cast<size_t>(dai::CameraBoardSocket::CAM_J) + 1;
static std::array<OpenEeprom, EEPROM_FD_COUNT> open_eeproms;

static OpenEeprom* getOpenEeprom(dai::CameraBoardSocket camSocket) {
    const auto index = static_cast<int>(camSocket) + CAM_OFFSET;
    if(index < 0 || index >= static_cast<int>(open_eeproms.size())) {
        return nullptr;
    }
    return &open_eeproms[index];
}

static const std::vector<std::vector<float>> kImuCalibrationMatrixIdentity = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f},
};

static std::vector<float> flattenImuCalibrationMatrix(const std::vector<std::vector<float>>& calibration) {
    if(calibration.size() != 3) {
        return {};
    }

    std::vector<float> flattened;
    flattened.reserve(12);
    for(const auto& row : calibration) {
        if(row.size() != 4) {
            return {};
        }
        flattened.insert(flattened.end(), row.begin(), row.end());
    }
    return flattened;
}

static std::vector<float> copyPackedFloats(const void* src, size_t count) {  // EEPROM structs are packed: a float* to their members is misaligned (UB)
    std::vector<float> values(count);
    if(count > 0) {
        std::memcpy(values.data(), src, count * sizeof(float));
    }
    return values;
}

static void assignFlatImuCalibration(std::vector<std::vector<float>>& calibration, const void* data, size_t count) {
    const std::vector<float> values = copyPackedFloats(data, std::min<size_t>(count, 12));
    calibration = kImuCalibrationMatrixIdentity;
    for(size_t i = 0; i < values.size(); ++i) {
        calibration[i / 4][i % 4] = values[i];
    }
}

static eeprom_status eeprom_store(const void* data, size_t size, size_t add_offset, eeprom_area area, dai::CameraBoardSocket camSocket) {
    if(!eeprom_found(camSocket)) {
        return EEPROM_NOT_AVAILABLE;
    }
    const auto* eeprom = getOpenEeprom(camSocket);

    if(area == EEPROM_AREA_USER) {
        if(size + add_offset > EEPROM_FACTORY_OFFSET) {
            return EEPROM_OUT_OF_BOUNDS;
        }
    } else if(area == EEPROM_AREA_FACTORY) {
        add_offset += EEPROM_FACTORY_OFFSET;
    } else {
        return EEPROM_INVALID_AREA;
    }

    if(camSocket != dai::CameraBoardSocket::AUTO) {  // CBA EEPROMs share the camera I2C buses with the sensors: direct, paced writes
        const auto error = i2c_eeprom_write(eeprom->fd, eeprom->cbaAddress, static_cast<const uint8_t*>(data), size, EEPROM_DATA_OFFSET + add_offset);
        if(!error.empty()) {
            printf("CBA EEPROM %s\n", error.c_str());
            return EEPROM_ERROR;
        }
        return EEPROM_OK;
    }

    int sc;
    sc = lseek(eeprom->fd, EEPROM_DATA_OFFSET + add_offset, SEEK_SET);
    if(sc < 0) {
        perror("eeprom lseek");
        printf("eeprom lseek error");
        return EEPROM_ERROR;
    }
    sc = write(eeprom->fd, data, size);
    if(sc < 0) {
        perror("eeprom write");
        printf("eeprom write error");
        return EEPROM_ERROR;
    }

    return EEPROM_OK;
}

static eeprom_status eeprom_read(void* data, size_t size, size_t add_offset, eeprom_area area, dai::CameraBoardSocket camSocket) {
    if(!eeprom_found(camSocket)) {
        return EEPROM_NOT_AVAILABLE;
    }
    const auto* eeprom = getOpenEeprom(camSocket);

    if(area == EEPROM_AREA_USER) {
        add_offset += 0;
    } else if(area == EEPROM_AREA_FACTORY) {
        add_offset += EEPROM_FACTORY_OFFSET;
    } else {
        return EEPROM_INVALID_AREA;
    }

    if(camSocket != dai::CameraBoardSocket::AUTO) {
        const auto error = i2c_eeprom_read(eeprom->fd, eeprom->cbaAddress, static_cast<uint8_t*>(data), size, EEPROM_DATA_OFFSET + add_offset);
        if(!error.empty()) {
            printf("CBA EEPROM %s\n", error.c_str());
            return EEPROM_ERROR;
        }
        return EEPROM_OK;
    }

    int sc;
    sc = lseek(eeprom->fd, EEPROM_DATA_OFFSET + add_offset, SEEK_SET);
    if(sc < 0) {
        perror("eeprom lseek");
        return EEPROM_ERROR;
    }
    sc = read(eeprom->fd, data, size);
    if(sc < 0) {
        perror("eeprom read");
        return EEPROM_ERROR;
    }

    return EEPROM_OK;
}

static bool eeprom_check_protected_fields_are_same(const dai::EepromData& l, const dai::EepromData& r) {  // Changeable only in factory mode (pre and post v7)
    return l.boardName == r.boardName && l.boardRev == r.boardRev && l.boardConf == r.boardConf && l.hardwareConf == r.hardwareConf
           && l.deviceName == r.deviceName && l.batchTime == r.batchTime && l.boardOptions == r.boardOptions
           && l.productName == r.productName;  // As of v2.20.0
}

int eeprom_open_cba(dai::CameraBoardSocket camSocket, int bus, uint16_t address) {
    auto* eeprom = getOpenEeprom(camSocket);
    if(eeprom == nullptr || camSocket == dai::CameraBoardSocket::AUTO) {
        return -1;
    }
    eeprom_close(camSocket);
    std::string error;
    eeprom->fd = i2c_eeprom_open(bus, address, error);
    if(eeprom->fd < 0) {
        printf("CBA EEPROM open failed: %s\n", error.c_str());
        return -1;
    }
    eeprom->cbaAddress = address;
    return 0;
}

int eeprom_open(dai::CameraBoardSocket camSocket) {
    auto* eeprom = getOpenEeprom(camSocket);
    if(eeprom == nullptr) {
        return -1;
    }
    int* eepFd = &eeprom->fd;

    if(camSocket != dai::CameraBoardSocket::AUTO) {
        const auto cba = CameraSingleton::getInstance().getCbaEeprom(camSocket);
        if(!cba.has_value()) {
            return -1;
        }
        return eeprom_open_cba(camSocket, cba->bus, cba->address);
    }

    std::vector<std::string> fileList = {"/sys/bus/nvmem/devices/calibration-eeprom/nvmem",
                                         "/sys/bus/i2c/drivers/at24/1-0051/eeprom",
                                         "/sys/bus/i2c/drivers/at24/0-0051/eeprom",
                                         "/data/eeprom.data"};

    for(auto& fileName : fileList) {
        *eepFd = open(fileName.c_str(), O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        printf("Eeprom file %s open status %d", fileName.c_str(), *eepFd);
        if(*eepFd >= 0) {
            return 0;
        }
    }
    perror("eeprom open");
    return *eepFd;
}

int eeprom_close(dai::CameraBoardSocket camSocket) {
    auto* eeprom = getOpenEeprom(camSocket);
    if(eeprom == nullptr) {
        return -1;
    }
    if(eeprom->fd >= 0) {
        close(eeprom->fd);
    }
    eeprom->fd = -1;
    return 0;
}

bool eeprom_found(dai::CameraBoardSocket camSocket) {
    const auto* eeprom = getOpenEeprom(camSocket);
    return eeprom != nullptr && eeprom->fd >= 0;
}

std::tuple<bool, std::string, std::vector<uint8_t>> eeprom_read_raw(eeprom_area area, dai::CameraBoardSocket camSocket) {
    std::vector<uint8_t> binary;
    binary.resize(SIZE_OF_CONFIGURATION);
    int ret = 0;
    if((ret = eeprom_read(binary.data(), SIZE_OF_CONFIGURATION, 0, area, camSocket)) != 0) {
        printf("Couldn't read raw EEPROM contents (code %d)\n", ret);
        return {false, "Couldn't read raw EEPROM contents", {}};
    }
    return {true, "", binary};
}

namespace {
constexpr size_t I2C_EEPROM_PAGE_SIZE = 32;        // A write must stay within one page, otherwise it wraps to the start of that page
constexpr size_t I2C_EEPROM_MAX_WRITE_CHUNK = 9;   // The CCI I2C bridge carries at most 11 bytes per write: the 2-byte memory address plus 9 data bytes
constexpr size_t I2C_EEPROM_MAX_READ_CHUNK = 128;  // Short reads so the CCI never delays the sensors' per-frame register writes
constexpr auto I2C_EEPROM_WRITE_CYCLE = std::chrono::milliseconds(10);  // Worst-case write cycle; polling gives NACKs that reset the I2C master of the sensor

constexpr size_t CCM_EEPROM_SIZE = 8192;
constexpr size_t CCM_EEPROM_MAX_WRITE_SIZE = 4096;  // Keeps a write call well within the host's default 10 s RPC timeout

int ccm_eeprom_open(dai::CameraBoardSocket camSocket, std::string& error) {
    try {
        return i2c_eeprom_open(CameraSingleton::getInstance().getCcmEepromBus(camSocket), CameraSingleton::CCM_EEPROM_ADDRESS, error);
    } catch(const std::exception& ex) {
        error = ex.what();
        return -1;
    }
}

bool ccm_eeprom_in_range(size_t size, size_t offset, std::string& error) {
    if(offset > CCM_EEPROM_SIZE || size > CCM_EEPROM_SIZE - offset) {
        error = "CCM EEPROM access out of range: offset " + std::to_string(offset) + " + size " + std::to_string(size) + " exceeds "
                + std::to_string(CCM_EEPROM_SIZE) + " bytes";
        return false;
    }
    return true;
}
}  // namespace

int i2c_eeprom_open(int bus, uint16_t address, std::string& error) {
    const auto path = "/dev/i2c-" + std::to_string(bus);
    const int fd = open(path.c_str(), O_RDWR);
    if(fd < 0) {
        error = "Couldn't open " + path + ": " + strerror(errno);
        return -1;
    }
    if(ioctl(fd, I2C_SLAVE, address) < 0) {  // Fails with EBUSY if a kernel driver is bound to the address
        char addressText[8];
        snprintf(addressText, sizeof(addressText), "0x%02x", address);
        error = std::string("Couldn't address ") + addressText + " on " + path + ": " + strerror(errno);
        close(fd);
        return -1;
    }
    return fd;
}

std::string i2c_eeprom_read(int fd, uint16_t address, uint8_t* data, size_t size, size_t offset) {
    for(size_t done = 0; done < size;) {
        const size_t at = offset + done;
        const size_t chunk = std::min(I2C_EEPROM_MAX_READ_CHUNK, size - done);
        uint8_t memoryAddress[2] = {static_cast<uint8_t>(at >> 8), static_cast<uint8_t>(at)};
        i2c_msg msgs[2] = {{address, 0, sizeof(memoryAddress), memoryAddress}, {address, I2C_M_RD, static_cast<uint16_t>(chunk), data + done}};
        i2c_rdwr_ioctl_data transfer{msgs, 2};
        if(ioctl(fd, I2C_RDWR, &transfer) < 0) {
            return "read failed at offset " + std::to_string(at) + ": " + strerror(errno);
        }
        done += chunk;
    }
    return "";
}

std::string i2c_eeprom_write(int fd, uint16_t address, const uint8_t* data, size_t size, size_t offset) {
    for(size_t done = 0; done < size;) {
        const size_t at = offset + done;
        const size_t chunk = std::min({I2C_EEPROM_MAX_WRITE_CHUNK, I2C_EEPROM_PAGE_SIZE - at % I2C_EEPROM_PAGE_SIZE, size - done});
        uint8_t buffer[2 + I2C_EEPROM_MAX_WRITE_CHUNK] = {static_cast<uint8_t>(at >> 8), static_cast<uint8_t>(at)};
        std::memcpy(buffer + 2, data + done, chunk);
        i2c_msg msg{address, 0, static_cast<uint16_t>(2 + chunk), buffer};
        i2c_rdwr_ioctl_data transfer{&msg, 1};
        const int result = ioctl(fd, I2C_RDWR, &transfer);
        std::this_thread::sleep_for(I2C_EEPROM_WRITE_CYCLE);
        if(result < 0) {
            return "write failed at offset " + std::to_string(at) + ": " + strerror(errno);
        }
        done += chunk;
    }
    return "";
}

std::tuple<bool, std::string, std::vector<uint8_t>> ccm_eeprom_read_raw(dai::CameraBoardSocket camSocket, size_t size, size_t offset) {
    std::string error;
    if(!ccm_eeprom_in_range(size, offset, error)) {
        return {false, error, {}};
    }
    const int fd = ccm_eeprom_open(camSocket, error);
    if(fd < 0) {
        return {false, error, {}};
    }
    std::vector<uint8_t> data(size);
    error = i2c_eeprom_read(fd, CameraSingleton::CCM_EEPROM_ADDRESS, data.data(), size, offset);
    close(fd);
    if(!error.empty()) {
        return {false, "CCM EEPROM " + error + "; the camera must be streaming so that its module is powered", {}};
    }
    return {true, "", data};
}

std::tuple<bool, std::string> ccm_eeprom_write_raw(dai::CameraBoardSocket camSocket, std::vector<uint8_t> data, size_t offset) {
    std::string error;
    if(!ccm_eeprom_in_range(data.size(), offset, error)) {
        return {false, error};
    }
    if(data.size() > CCM_EEPROM_MAX_WRITE_SIZE) {
        return {false,
                "CCM EEPROM write of " + std::to_string(data.size()) + " bytes exceeds " + std::to_string(CCM_EEPROM_MAX_WRITE_SIZE)
                    + " bytes per call; split it into multiple writes"};
    }
    const int fd = ccm_eeprom_open(camSocket, error);
    if(fd < 0) {
        return {false, error};
    }
    error = i2c_eeprom_write(fd, CameraSingleton::CCM_EEPROM_ADDRESS, data.data(), data.size(), offset);
    close(fd);
    if(!error.empty()) {
        return {false, "CCM EEPROM " + error + "; the camera must be streaming and the module EEPROM not write-protected"};
    }
    return {true, ""};
}

eeprom_version eeprom_get_version(eeprom_area area, dai::CameraBoardSocket camSocket) {
    eeprom_version eeprom_version = EEPROM_VERSION_UNPROGRAMMED;
    eeprom_read(&eeprom_version, EEPROM_VERSION_SIZE, 0, area, camSocket);
    return eeprom_version;
}

std::tuple<bool, std::string> eeprom_factory_reset(dai::CameraBoardSocket camSocket) {
    if(!eeprom_found(camSocket)) {
        return {false, "EEPROM not available to do a factory reset"};
    }

    dai::EepromData board_data_config;

    eeprom_status status = eeprom_load_config(board_data_config, EEPROM_AREA_FACTORY, camSocket);
    if(status == EEPROM_INVALID_DATA) {
        printf("Missing or corrupted factory default calibration");
        return {false, "Missing or corrupted factory default calibration"};
    } else if(status != EEPROM_OK) {
        return {false, "Coudln't read factory defaults from EEPROM, error: " + std::string(eeprom_status_to_str(status))};
    }

    uint8_t factory_default[SIZE_OF_CONFIGURATION];
    if((status = eeprom_read(factory_default, SIZE_OF_CONFIGURATION, 0, EEPROM_AREA_FACTORY, camSocket)) != EEPROM_OK) {
        printf("Couldn't read factory defaults from FACTORY area, ret: %d\n", status);
        return {false, "Couldn't read factory defaults from FACTORY area, error: " + std::string(eeprom_status_to_str(status))};
    }
    if((status = eeprom_store(factory_default, SIZE_OF_CONFIGURATION, 0, EEPROM_AREA_USER, camSocket)) != EEPROM_OK) {
        printf("Couldn't flash factory defaults to USER area\n");
        return {false, "Couldn't flash factory defaults to USER area, error: " + std::string(eeprom_status_to_str(status))};
    }

    return {true, ""};
}

eeprom_status eeprom_store_config(
    dai::EepromData& board_data_config, eeprom_area area, bool factory_permissions, bool protected_permissions, dai::CameraBoardSocket camSocket) {
    if(!eeprom_found(camSocket)) {
        return EEPROM_NOT_AVAILABLE;
    }

    eeprom_status ret = EEPROM_OK;

    if(area == EEPROM_AREA_FACTORY && !factory_permissions) {
        return EEPROM_NO_FACTORY_PERMISSIONS;
    }

    if(!protected_permissions) {
        dai::EepromData prev_board_config;
        if(eeprom_load_config(prev_board_config, area, camSocket) == EEPROM_OK
           && !eeprom_check_protected_fields_are_same(board_data_config, prev_board_config)) {
            return EEPROM_NO_PROTECTED_PERMISSIONS;
        }
    }

    auto matrix_to_1D = [](std::vector<std::vector<float>>& src, void* dest_unaligned) {
        float dest[3 * 3];
        for(int j = 0; j < 3; j++) {
            for(int k = 0; k < 3; k++) {
                if(src.size() == 0) {
                    dest[3 * j + k] = 0;
                } else {
                    dest[3 * j + k] = src[j][k];
                }
            }
        }
        memcpy(dest_unaligned, &dest, sizeof dest);
        return;
    };
    uint32_t version = board_data_config.version | 0x55AA0000;
    if(version == EEPROM_VERSION_V7 || version == EEPROM_VERSION_V6 || version == EEPROM_VERSION_V5) {
        gen2_eeprom_data board_data = {};

        if(version == EEPROM_VERSION_V5) {
            board_data.header.version = EEPROM_VERSION_V6;
        } else {
            board_data.header.version = version;
        }

        if(board_data_config.boardName.size() > sizeof(board_data.board_name) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::strncpy(board_data.board_name, board_data_config.boardName.c_str(), sizeof(board_data.board_name) - 1);

        if(board_data_config.boardRev.size() > sizeof(board_data.board_rev) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::strncpy(board_data.board_rev, board_data_config.boardRev.c_str(), sizeof(board_data.board_rev) - 1);

        if(board_data_config.cameraData.size() > MAX_CAMERAS) {
            return EEPROM_FIELD_TOO_LONG;
        }
        board_data.header.num_cameras = board_data_config.cameraData.size();

        int cam_idx = 0;
        for(auto& cam_it : board_data_config.cameraData) {
            printf("Iteration \n");

            dai::CameraInfo& cam_info = cam_it.second;
            board_data.camera_data[cam_idx].camera_id = static_cast<int8_t>(cam_it.first);

            matrix_to_1D(cam_info.intrinsicMatrix, board_data.camera_data[cam_idx].intrinsics);

            if(cam_info.distortionCoeff.size()
               > sizeof(board_data.camera_data[cam_idx].distortion_coeff) / sizeof(board_data.camera_data[cam_idx].distortion_coeff[0])) {
                return EEPROM_FIELD_TOO_LONG;
            }
            if(!cam_info.distortionCoeff.empty()) {
                memcpy(board_data.camera_data[cam_idx].distortion_coeff, cam_info.distortionCoeff.data(), cam_info.distortionCoeff.size() * sizeof(float));
            }
            board_data.camera_data[cam_idx].measured_fov_deg = cam_info.specHfovDeg;

            board_data.camera_data[cam_idx].extrinsics.translation.x = cam_info.extrinsics.translation.x;
            board_data.camera_data[cam_idx].extrinsics.translation.y = cam_info.extrinsics.translation.y;
            board_data.camera_data[cam_idx].extrinsics.translation.z = cam_info.extrinsics.translation.z;
            board_data.camera_data[cam_idx].extrinsics.measured_translation.x = cam_info.extrinsics.specTranslation.x;
            board_data.camera_data[cam_idx].extrinsics.measured_translation.y = cam_info.extrinsics.specTranslation.y;
            board_data.camera_data[cam_idx].extrinsics.measured_translation.z = cam_info.extrinsics.specTranslation.z;

            board_data.camera_data[cam_idx].height = cam_info.height;
            board_data.camera_data[cam_idx].width = cam_info.width;
            board_data.camera_data[cam_idx].lens_position = cam_info.lensPosition;

            matrix_to_1D(cam_info.extrinsics.rotationMatrix, board_data.camera_data[cam_idx].extrinsics.rotation);
            board_data.camera_data[cam_idx].extrinsics.to_camera_socket = static_cast<int8_t>(cam_info.extrinsics.toCameraSocket);

            if(version == EEPROM_VERSION_V6 || version == EEPROM_VERSION_V7) {
                board_data.camera_data[cam_idx].camera_type = static_cast<int8_t>(cam_info.cameraType);
            } else {
                board_data.camera_data[cam_idx].camera_type = static_cast<int8_t>(dai::CameraModel::Perspective);
            }
            cam_idx++;
        }

        matrix_to_1D(board_data_config.stereoRectificationData.rectifiedRotationLeft, board_data.stereo_data.rectified_rotation_left);
        matrix_to_1D(board_data_config.stereoRectificationData.rectifiedRotationRight, board_data.stereo_data.rectified_rotation_right);
        board_data.stereo_data.left_camera_socket = static_cast<int8_t>(board_data_config.stereoRectificationData.leftCameraSocket);
        board_data.stereo_data.right_camera_socket = static_cast<int8_t>(board_data_config.stereoRectificationData.rightCameraSocket);

        board_data.imu_extrinsics.translation.x = board_data_config.imuExtrinsics.translation.x;
        board_data.imu_extrinsics.translation.y = board_data_config.imuExtrinsics.translation.y;
        board_data.imu_extrinsics.translation.z = board_data_config.imuExtrinsics.translation.z;
        board_data.imu_extrinsics.measured_translation.x = board_data_config.imuExtrinsics.specTranslation.x;
        board_data.imu_extrinsics.measured_translation.y = board_data_config.imuExtrinsics.specTranslation.y;
        board_data.imu_extrinsics.measured_translation.z = board_data_config.imuExtrinsics.specTranslation.z;

        matrix_to_1D(board_data_config.imuExtrinsics.rotationMatrix, board_data.imu_extrinsics.rotation);
        board_data.imu_extrinsics.to_camera_socket = static_cast<int8_t>(board_data_config.imuExtrinsics.toCameraSocket);

        gen2_eeprom_ex_data ex_data = {};

        if(board_data_config.productName.size() > sizeof(ex_data.product_name) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::strncpy(ex_data.product_name, board_data_config.productName.c_str(), sizeof(ex_data.product_name) - 1);

        if(board_data_config.boardConf.size() > sizeof(ex_data.board_conf) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::strncpy(ex_data.board_conf, board_data_config.boardConf.c_str(), sizeof(ex_data.board_conf) - 1);

        if(board_data_config.hardwareConf.size() > sizeof(ex_data.hardware_conf) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::strncpy(ex_data.hardware_conf, board_data_config.hardwareConf.c_str(), sizeof(ex_data.hardware_conf) - 1);

        if(board_data_config.boardCustom.size() > sizeof(ex_data.board_custom) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::strncpy(ex_data.board_custom, board_data_config.boardCustom.c_str(), sizeof(ex_data.board_custom) - 1);

        ex_data.batch_time = board_data_config.batchTime;
        ex_data.board_options = board_data_config.boardOptions;

        if(board_data_config.deviceName.size() > sizeof(ex_data.batch2_name) - 1 + sizeof(ex_data.batch2_name) - 1) {
            return EEPROM_FIELD_TOO_LONG;
        }
        std::string deviceName = board_data_config.deviceName.substr(0, sizeof(ex_data.batch_name) - 1);

        std::string deviceName2;
        if(!board_data_config.deviceName.empty() && board_data_config.deviceName.size() - 1 >= sizeof(ex_data.batch_name)) {
            deviceName2 = board_data_config.deviceName.substr(sizeof(ex_data.batch_name) - 1);
        }
        std::strncpy(ex_data.batch_name, deviceName.c_str(), sizeof(ex_data.batch_name) - 1);
        std::strncpy(ex_data.batch2_name, deviceName2.c_str(), sizeof(ex_data.batch2_name) - 1);

        ex_data.housing_extrinsics.translation.x = board_data_config.housingExtrinsics.translation.x;
        ex_data.housing_extrinsics.translation.y = board_data_config.housingExtrinsics.translation.y;
        ex_data.housing_extrinsics.translation.z = board_data_config.housingExtrinsics.translation.z;
        ex_data.housing_extrinsics.measured_translation.x = board_data_config.housingExtrinsics.specTranslation.x;
        ex_data.housing_extrinsics.measured_translation.y = board_data_config.housingExtrinsics.specTranslation.y;
        ex_data.housing_extrinsics.measured_translation.z = board_data_config.housingExtrinsics.specTranslation.z;
        matrix_to_1D(board_data_config.housingExtrinsics.rotationMatrix, ex_data.housing_extrinsics.rotation);
        ex_data.housing_extrinsics.to_camera_socket = static_cast<int8_t>(board_data_config.housingExtrinsics.toCameraSocket);

        ex_data.vertical_camera_socket = static_cast<int8_t>(board_data_config.verticalCameraSocket);

        ex_data.bitfield = 0;                                                                                                       // bit 7 = 0, valid bitfield
        ex_data.bitfield |= board_data_config.stereoUseSpecTranslation ? 0 : EEPROM_BITFIELD_NOT_USE_SPEC_TRANSLATION;              // inverse logic
        ex_data.bitfield |= board_data_config.stereoEnableDistortionCorrection ? EEPROM_BITFIELD_ENABLE_DISTORTION_CORRECTION : 0;  // normal logic

        board_data.imu.params.magic = IMU_CALIB_PARAMS_MAGIC;

        const auto accelerometerCalibParams = flattenImuCalibrationMatrix(board_data_config.imuCalibrationParams.accelerometer);
        for(size_t i = 0; i < accelerometerCalibParams.size(); ++i) {
            if(i >= ARRAY_LEN(board_data.imu.params.acc_calib)) {
                return EEPROM_FIELD_TOO_LONG;
            }
            board_data.imu.params.acc_calib[i] = accelerometerCalibParams[i];
        }
        board_data.imu.params.acc_calib_count = accelerometerCalibParams.size();

        const auto gyroscopeCalibParams = flattenImuCalibrationMatrix(board_data_config.imuCalibrationParams.gyroscope);
        for(size_t i = 0; i < gyroscopeCalibParams.size(); ++i) {
            if(i >= ARRAY_LEN(board_data.imu.params.gyro_calib)) {
                return EEPROM_FIELD_TOO_LONG;
            }
            board_data.imu.params.gyro_calib[i] = gyroscopeCalibParams[i];
        }
        board_data.imu.params.gyro_calib_count = gyroscopeCalibParams.size();

        board_data.header.misc_data_size = 0;
        ret = eeprom_store(&board_data,
                           sizeof(board_data) - (((MAX_CAMERAS - board_data.header.num_cameras) * sizeof(lux_camera_info)) + sizeof(board_data.imu)),
                           0,
                           area,
                           camSocket);
        if(ret != EEPROM_OK) {
            return ret;
        }

        ret = eeprom_store(&board_data.imu, sizeof(board_data.imu.params), offsetof(gen2_eeprom_data, imu), area, camSocket);
        if(ret != EEPROM_OK) {
            return ret;
        }

        if(version == EEPROM_VERSION_V7) {
            ret = eeprom_store(&ex_data, sizeof(ex_data), sizeof(board_data), area, camSocket);
            if(ret != 0) {
                return ret;
            }
        }

        return EEPROM_OK;
    } else {
        eeprom_data board_data;
        board_data.version = version;
        switch(version) {
            case EEPROM_VERSION_V4:
                matrix_to_1D(board_data_config.stereoRectificationData.rectifiedRotationRight, board_data.calib_R2_right);
                matrix_to_1D(board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].intrinsicMatrix, board_data.calib_M1_left);
                matrix_to_1D(board_data_config.cameraData[dai::CameraBoardSocket::CAM_C].intrinsicMatrix, board_data.calib_M2_right);

                matrix_to_1D(board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].extrinsics.rotationMatrix, board_data.calib_R);
                board_data.calib_T[0] = board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].extrinsics.translation.x;
                board_data.calib_T[1] = board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].extrinsics.translation.y;
                board_data.calib_T[2] = board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].extrinsics.translation.z;
                __attribute__((fallthrough));
            case EEPROM_VERSION_V2:
                std::strcpy(board_data.board_name, board_data_config.boardName.c_str());
                std::strcpy(board_data.board_rev, board_data_config.boardRev.c_str());
                board_data.rgb_fov_deg = board_data_config.cameraData[dai::CameraBoardSocket::CAM_A].specHfovDeg;
                __attribute__((fallthrough));
            case EEPROM_VERSION_V1:
                board_data.left_fov_deg = board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].specHfovDeg;
                matrix_to_1D(board_data_config.stereoRectificationData.rectifiedRotationLeft, board_data.calib_R1_left);
                board_data.left_to_right_distance_m = board_data_config.cameraData[dai::CameraBoardSocket::CAM_B].extrinsics.specTranslation.x / 100;
                board_data.left_to_rgb_distance_m = board_data_config.cameraData[dai::CameraBoardSocket::CAM_C].extrinsics.specTranslation.x / 100;
        }
        switch(version) {
            case EEPROM_VERSION_V1:
                board_data.rgb_fov_deg = 0;
                memset(board_data.board_name, '\0', sizeof(board_data.board_name));
                memset(board_data.board_rev, '\0', sizeof(board_data.board_rev));
                __attribute__((fallthrough));
            case EEPROM_VERSION_V2:
                board_data.stereo_center_crop = true;
                __attribute__((fallthrough));
            case EEPROM_VERSION_V3:
                memset(board_data.calib_R2_right, 0, sizeof(board_data.calib_R2_right));
                memset(board_data.calib_M1_left, 0, sizeof(board_data.calib_M1_left));
                memset(board_data.calib_M2_right, 0, sizeof(board_data.calib_M2_right));
                memset(board_data.calib_R, 0, sizeof(board_data.calib_R));
                memset(board_data.calib_T, 0, sizeof(board_data.calib_T));
                __attribute__((fallthrough));
            case EEPROM_VERSION_V4:
                memset(board_data.calib_M3_rgb, 0, sizeof(board_data.calib_M3_rgb));
                memset(board_data.calib_R_rgb, 0, sizeof(board_data.calib_R_rgb));
                memset(board_data.calib_T_rgb, 0, sizeof(board_data.calib_T_rgb));
                memset(board_data.calib_d1_left, 0, sizeof(board_data.calib_d1_left));
                memset(board_data.calib_d2_right, 0, sizeof(board_data.calib_d2_right));
                memset(board_data.calib_d3_rgb, 0, sizeof(board_data.calib_d3_rgb));
        }

        if(eeprom_store(&board_data, sizeof(board_data), 0, area, camSocket) == 0) {
            return EEPROM_OK;
        } else {
            return EEPROM_ERROR;
        }
    }
}

eeprom_status eeprom_load_config(dai::EepromData& board_data_config, eeprom_area area, dai::CameraBoardSocket camSocket) {
    if(!eeprom_found(camSocket)) {
        return EEPROM_NOT_AVAILABLE;
    }

    eeprom_status ret = EEPROM_OK;

    uint32_t version = eeprom_get_version(area, camSocket);
    if(!eeprom_version_is_valid(version)) {
        return EEPROM_INVALID_DATA;
    }

    auto matrix_3x3_conv = [](const void* src, std::vector<std::vector<float>>& dest) {
        const std::vector<float> values = copyPackedFloats(src, 3 * 3);
        for(int j = 0; j < 3; j++) {
            std::vector<float> temp(values.begin() + 3 * j, values.begin() + 3 * (j + 1));
            dest.push_back(temp);
        }
        return;
    };

    board_data_config.version = version & 0xFF;
    if(version == EEPROM_VERSION_V6 || version == EEPROM_VERSION_V7) {
        if(version == EEPROM_VERSION_V6) {
            printf("Reading VERSION 6 --- ->\n");
        } else if(version == EEPROM_VERSION_V7) {
            printf("Reading VERSION 7 --- ->\n");
        }

        gen2_info header = {};
        ret = eeprom_read(&header, sizeof(gen2_info), 0, area, camSocket);
        if(ret != EEPROM_OK) return ret;
        if(header.num_cameras > MAX_CAMERAS) {
            return EEPROM_INVALID_DATA;
        }

        gen2_eeprom_data board_data = {};
        ret = eeprom_read(
            &board_data, sizeof(board_data) - (((MAX_CAMERAS - header.num_cameras) * sizeof(lux_camera_info)) + sizeof(board_data.imu)), 0, area, camSocket);
        if(ret != EEPROM_OK) return ret;

        ret = eeprom_read(&board_data.imu.params, sizeof(board_data.imu.params), offsetof(gen2_eeprom_data, imu), area, camSocket);
        if(ret != EEPROM_OK) return ret;

        if(header.misc_data_size > EEPROM_MEM_SIZE) header.misc_data_size = 0;  // Corrupted EEPROM data can make this a huge number

        std::vector<uint8_t> misc_data(header.misc_data_size);

        if(header.misc_data_size != 0) {
            size_t misc_offset = sizeof(board_data) - ((MAX_CAMERAS - header.num_cameras) * sizeof(lux_camera_info));
            if(version == EEPROM_VERSION_V7) {
                misc_offset = misc_offset + sizeof(gen2_eeprom_ex_data);
            }
            ret = eeprom_read(&misc_data, header.misc_data_size, misc_offset, area, camSocket);
            if(ret != EEPROM_OK) return ret;
        }

        board_data_config.miscellaneousData = misc_data;
        for(uint8_t i = 0; i < header.num_cameras; ++i) {
            lux_camera_info cam_data = board_data.camera_data[i];
            if(cam_data.intrinsics[0] == 0 && cam_data.lens_position != 0) {
                dai::CameraInfo camera;
                camera.lensPosition = cam_data.lens_position;
                board_data_config.cameraData[static_cast<dai::CameraBoardSocket>(cam_data.camera_id)] = camera;
                continue;
            } else if(cam_data.intrinsics[0] == 0) {
                continue;
            }

            if(board_data_config.cameraData.find(static_cast<dai::CameraBoardSocket>(cam_data.camera_id)) == board_data_config.cameraData.end()) {
                board_data_config.cameraData[static_cast<dai::CameraBoardSocket>(cam_data.camera_id)] = dai::CameraInfo();
            }
            dai::CameraInfo& camera = board_data_config.cameraData[static_cast<dai::CameraBoardSocket>(cam_data.camera_id)];
            camera.width = cam_data.width;
            camera.height = cam_data.height;
            camera.lensPosition = cam_data.lens_position;
            matrix_3x3_conv(cam_data.intrinsics, camera.intrinsicMatrix);
            matrix_3x3_conv(cam_data.extrinsics.rotation, camera.extrinsics.rotationMatrix);

            point temp_translation = cam_data.extrinsics.translation;
            camera.extrinsics.translation = dai::Point3f(temp_translation.x, temp_translation.y, temp_translation.z);

            temp_translation = cam_data.extrinsics.measured_translation;
            float translation_diff = (temp_translation.x * -1) - camera.extrinsics.translation.x;
            if(abs(translation_diff) < 0.7) {
                camera.extrinsics.specTranslation = dai::Point3f(-temp_translation.x, -temp_translation.y, -temp_translation.z);
            } else {
                camera.extrinsics.specTranslation = dai::Point3f(temp_translation.x, temp_translation.y, temp_translation.z);
            }
            camera.extrinsics.toCameraSocket = (dai::CameraBoardSocket)cam_data.extrinsics.to_camera_socket;
            camera.specHfovDeg = cam_data.measured_fov_deg;
            camera.cameraType = (dai::CameraModel)cam_data.camera_type;
            int8_t to_cam_id = cam_data.extrinsics.to_camera_socket;

            camera.distortionCoeff = copyPackedFloats(cam_data.distortion_coeff, TOTAL_DISTORTION_PARAMETERS);
            board_data_config.cameraData[static_cast<dai::CameraBoardSocket>(cam_data.camera_id)] = camera;
        }

        auto& ec = board_data_config.cameraData;  // Workaround: spec translation is wrong on some boards with RGB not centered (CM3/CM4)
        if(ec.find(dai::CameraBoardSocket::CAM_B) != ec.end() && ec.find(dai::CameraBoardSocket::CAM_C) != ec.end()) {
            float& actual_spec = ec[dai::CameraBoardSocket::CAM_C].extrinsics.specTranslation.x;
            float calib_val = ec[dai::CameraBoardSocket::CAM_C].extrinsics.translation.x;
            float diff_spec_to_calib = abs(actual_spec - calib_val);
            if((ec[dai::CameraBoardSocket::CAM_B].extrinsics.toCameraSocket == dai::CameraBoardSocket::CAM_C) && (diff_spec_to_calib > 0.5)) {
                // TODO check correctness here...
                float correct_spec = abs(ec[dai::CameraBoardSocket::CAM_B].extrinsics.specTranslation.x) - abs(actual_spec);
                float diff_correct_to_calib = abs(correct_spec - calib_val);
                if(diff_correct_to_calib < 0.5) {
                    actual_spec = correct_spec;
                }
            }
        }

        dai::StereoRectification rectification_data{};
        matrix_3x3_conv(board_data.stereo_data.rectified_rotation_left, rectification_data.rectifiedRotationLeft);
        matrix_3x3_conv(board_data.stereo_data.rectified_rotation_right, rectification_data.rectifiedRotationRight);
        rectification_data.leftCameraSocket = (dai::CameraBoardSocket)board_data.stereo_data.left_camera_socket;
        rectification_data.rightCameraSocket = (dai::CameraBoardSocket)board_data.stereo_data.right_camera_socket;
        board_data_config.stereoRectificationData = rectification_data;

        dai::Extrinsics imu_data{};
        matrix_3x3_conv(board_data.imu_extrinsics.rotation, imu_data.rotationMatrix);
        point temp_translation = board_data.imu_extrinsics.translation;
        imu_data.translation = dai::Point3f(temp_translation.x, temp_translation.y, temp_translation.z);
        temp_translation = board_data.imu_extrinsics.measured_translation;
        imu_data.specTranslation = dai::Point3f(temp_translation.x, temp_translation.y, temp_translation.z);
        imu_data.toCameraSocket = (dai::CameraBoardSocket)board_data.imu_extrinsics.to_camera_socket;
        board_data_config.imuExtrinsics = imu_data;

        if((board_data.imu.params.magic == IMU_CALIB_PARAMS_MAGIC) && (board_data.imu.params.acc_calib_count <= ARRAY_LEN(board_data.imu.params.acc_calib))
           && (board_data.imu.params.gyro_calib_count <= ARRAY_LEN(board_data.imu.params.gyro_calib))) {
            assignFlatImuCalibration(
                board_data_config.imuCalibrationParams.accelerometer, board_data.imu.params.acc_calib, board_data.imu.params.acc_calib_count);

            assignFlatImuCalibration(
                board_data_config.imuCalibrationParams.gyroscope, board_data.imu.params.gyro_calib, board_data.imu.params.gyro_calib_count);
        } else {
            printf("No IMU calib params available --- ->\n");

            board_data_config.imuCalibrationParams.accelerometer = kImuCalibrationMatrixIdentity;
            board_data_config.imuCalibrationParams.gyroscope = kImuCalibrationMatrixIdentity;
        }

        board_data_config.imuCalibrationParams.noise = dai::getDefaultImuNoiseParams();

        board_data_config.boardName.assign(board_data.board_name, std::find(board_data.board_name, board_data.board_name + 16, '\0'));
        board_data_config.boardRev.assign(board_data.board_rev, std::find(board_data.board_rev, board_data.board_rev + 10, '\0'));

        for(uint8_t i = 0; i < header.num_cameras; ++i) {  // Workaround: OAK-D-LITE R0M0E0/R1M1E1 have stereo width/height set to 1280x800
            lux_camera_info cam_data = board_data.camera_data[i];
            if(cam_data.intrinsics[0] == 0) {
                continue;
            }
            dai::CameraBoardSocket socket = static_cast<dai::CameraBoardSocket>(cam_data.camera_id);
            if(socket == rectification_data.leftCameraSocket || socket == rectification_data.rightCameraSocket) {
                if(board_data_config.boardName == "OAK-D-LITE") {
                    if(board_data_config.boardRev == "R0M0E0" || board_data_config.boardRev == "R1M1E1") {
                        dai::CameraInfo& camera = board_data_config.cameraData[socket];
                        if(camera.width > 640) camera.width = 640;
                        if(camera.height > 480) camera.height = 480;
                    }
                }
            }
        }

        if(version == EEPROM_VERSION_V7) {
            gen2_eeprom_ex_data ex_data = {};
            static_assert(sizeof(board_data) == 1810);  // The ex_data offset is correct only while gen2_eeprom_data keeps this size
            ret = eeprom_read(&ex_data, sizeof(ex_data), sizeof(board_data), area, camSocket);
            if(ret != EEPROM_OK) return ret;

            if(std::find(ex_data.product_name, ex_data.product_name + sizeof(ex_data.product_name), '\0')
               == ex_data.product_name + sizeof(ex_data.product_name))
                return EEPROM_INVALID_DATA;
            if(std::find(ex_data.board_conf, ex_data.board_conf + sizeof(ex_data.board_conf), '\0') == ex_data.board_conf + sizeof(ex_data.board_conf))
                return EEPROM_INVALID_DATA;
            if(std::find(ex_data.hardware_conf, ex_data.hardware_conf + sizeof(ex_data.hardware_conf), '\0')
               == ex_data.hardware_conf + sizeof(ex_data.hardware_conf))
                return EEPROM_INVALID_DATA;
            if(std::find(ex_data.batch_name, ex_data.batch_name + sizeof(ex_data.batch_name), '\0') == ex_data.batch_name + sizeof(ex_data.batch_name))
                return EEPROM_INVALID_DATA;
            if(std::find(ex_data.board_custom, ex_data.board_custom + sizeof(ex_data.board_custom), '\0')
               == ex_data.board_custom + sizeof(ex_data.board_custom))
                return EEPROM_INVALID_DATA;

            board_data_config.productName.assign(ex_data.product_name,
                                                 std::find(ex_data.product_name, ex_data.product_name + sizeof(ex_data.product_name), '\0'));
            board_data_config.boardConf.assign(ex_data.board_conf, std::find(ex_data.board_conf, ex_data.board_conf + sizeof(ex_data.board_conf), '\0'));
            board_data_config.hardwareConf.assign(ex_data.hardware_conf,
                                                  std::find(ex_data.hardware_conf, ex_data.hardware_conf + sizeof(ex_data.hardware_conf), '\0'));
            board_data_config.boardCustom.assign(ex_data.board_custom,
                                                 std::find(ex_data.board_custom, ex_data.board_custom + sizeof(ex_data.board_custom), '\0'));

            board_data_config.batchTime = ex_data.batch_time;
            board_data_config.boardOptions = ex_data.board_options;

            std::string batchName;
            batchName.assign(ex_data.batch_name, std::find(ex_data.batch_name, ex_data.batch_name + sizeof(ex_data.batch_name), '\0'));
            std::string batchName2;
            batchName2.assign(ex_data.batch2_name, std::find(ex_data.batch2_name, ex_data.batch2_name + sizeof(ex_data.batch2_name), '\0'));
            if(!batchName.empty() && (uint8_t)batchName[0] != 0xFF && (uint8_t)batchName2[0] != 0xFF) {
                board_data_config.deviceName = batchName + batchName2;
            }

            dai::Extrinsics housing_extrinsics{};
            if(ex_data.housing_extrinsics.to_camera_socket >= 0
               && ex_data.housing_extrinsics.to_camera_socket <= static_cast<int>(dai::CameraBoardSocket::CAM_J)
               && !std::isnan(ex_data.housing_extrinsics.measured_translation.x) && !std::isnan(ex_data.housing_extrinsics.measured_translation.y)
               && !std::isnan(ex_data.housing_extrinsics.measured_translation.z)) {
                matrix_3x3_conv(ex_data.housing_extrinsics.rotation, housing_extrinsics.rotationMatrix);
                point temp_translation = ex_data.housing_extrinsics.translation;
                housing_extrinsics.translation = dai::Point3f(temp_translation.x, temp_translation.y, temp_translation.z);
                temp_translation = ex_data.housing_extrinsics.measured_translation;
                housing_extrinsics.specTranslation = dai::Point3f(temp_translation.x, temp_translation.y, temp_translation.z);
                housing_extrinsics.toCameraSocket = (dai::CameraBoardSocket)ex_data.housing_extrinsics.to_camera_socket;
            }
            board_data_config.housingExtrinsics = housing_extrinsics;

            if(ex_data.vertical_camera_socket >= 0 && ex_data.vertical_camera_socket <= static_cast<int>(dai::CameraBoardSocket::CAM_J)) {
                board_data_config.verticalCameraSocket = (dai::CameraBoardSocket)ex_data.vertical_camera_socket;
            } else {
                board_data_config.verticalCameraSocket = dai::CameraBoardSocket::AUTO;
            }

            if(!(ex_data.bitfield & EEPROM_BITFIELD_NOT_VALID)) {
                board_data_config.stereoUseSpecTranslation = (ex_data.bitfield & EEPROM_BITFIELD_NOT_USE_SPEC_TRANSLATION) ? false : true;  // inverse logic
                board_data_config.stereoEnableDistortionCorrection =
                    (ex_data.bitfield & EEPROM_BITFIELD_ENABLE_DISTORTION_CORRECTION) ? true : false;  // normal logic
            }
        }

    } else {
        eeprom_data board_data = {};
        int readStatus = eeprom_read(&board_data, sizeof(board_data), 0, area, camSocket);
        printf("EEPROM read status %d\n", readStatus);

        printf("EEPROM data version: %u\n", board_data_config.version);

        dai::CameraInfo rgb_camera{}, left_camera{}, right_camera{};
        right_camera.height = 800;
        right_camera.width = 1280;

        left_camera.height = 800;
        left_camera.width = 1280;

        rgb_camera.height = 1080;
        rgb_camera.width = 1920;
        dai::CameraBoardSocket left = dai::CameraBoardSocket::CAM_B;
        dai::CameraBoardSocket right = dai::CameraBoardSocket::CAM_C;

        bool swapLeftRightCam = board_data.swap_left_and_right_cameras;

        if(!swapLeftRightCam) {
            right = dai::CameraBoardSocket::CAM_B;
            left = dai::CameraBoardSocket::CAM_C;
        }

        switch(board_data.version) {
            case EEPROM_VERSION_V5:
                matrix_3x3_conv(board_data.calib_M3_rgb, rgb_camera.intrinsicMatrix);
                matrix_3x3_conv(board_data.calib_R_rgb, right_camera.extrinsics.rotationMatrix);
                {
                    float temp = right_camera.extrinsics.rotationMatrix[0][1];
                    right_camera.extrinsics.rotationMatrix[0][1] = right_camera.extrinsics.rotationMatrix[1][0];
                    right_camera.extrinsics.rotationMatrix[1][0] = temp;

                    temp = right_camera.extrinsics.rotationMatrix[0][2];
                    right_camera.extrinsics.rotationMatrix[0][2] = right_camera.extrinsics.rotationMatrix[2][0];
                    right_camera.extrinsics.rotationMatrix[2][0] = temp;

                    temp = right_camera.extrinsics.rotationMatrix[1][2];
                    right_camera.extrinsics.rotationMatrix[1][2] = right_camera.extrinsics.rotationMatrix[2][1];
                    right_camera.extrinsics.rotationMatrix[2][1] = temp;
                }
                right_camera.extrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_A;
                rgb_camera.cameraType = dai::CameraModel::Perspective;

                rgb_camera.distortionCoeff = copyPackedFloats(board_data.calib_d3_rgb, TOTAL_DISTORTION_PARAMETERS);
                right_camera.extrinsics.translation = dai::Point3f(-board_data.calib_T_rgb[0], -board_data.calib_T_rgb[1], -board_data.calib_T_rgb[2]);

                left_camera.distortionCoeff = copyPackedFloats(board_data.calib_d1_left, TOTAL_DISTORTION_PARAMETERS);
                right_camera.distortionCoeff = copyPackedFloats(board_data.calib_d2_right, TOTAL_DISTORTION_PARAMETERS);
                __attribute__((fallthrough));
            case EEPROM_VERSION_V4:
                matrix_3x3_conv(board_data.calib_M2_right, right_camera.intrinsicMatrix);
                matrix_3x3_conv(board_data.calib_M1_left, left_camera.intrinsicMatrix);
                matrix_3x3_conv(board_data.calib_R, left_camera.extrinsics.rotationMatrix);
                right_camera.cameraType = dai::CameraModel::Perspective;
                left_camera.cameraType = dai::CameraModel::Perspective;

                left_camera.extrinsics.translation = dai::Point3f(board_data.calib_T[0], board_data.calib_T[1], board_data.calib_T[2]);
                matrix_3x3_conv(board_data.calib_R2_right, board_data_config.stereoRectificationData.rectifiedRotationRight);
                board_data_config.stereoRectificationData.rightCameraSocket = right;
                left_camera.extrinsics.toCameraSocket = right;
                __attribute__((fallthrough));
            case EEPROM_VERSION_V2:
                board_data_config.boardName.assign(board_data.board_name, std::find(board_data.board_name, board_data.board_name + 16, '\0'));
                board_data_config.boardRev.assign(board_data.board_rev, std::find(board_data.board_rev, board_data.board_rev + 10, '\0'));
                rgb_camera.specHfovDeg = board_data.rgb_fov_deg;
                __attribute__((fallthrough));
            case EEPROM_VERSION_V1:
                matrix_3x3_conv(board_data.calib_R1_left, board_data_config.stereoRectificationData.rectifiedRotationLeft);
                board_data_config.stereoRectificationData.leftCameraSocket = left;
                left_camera.extrinsics.specTranslation = dai::Point3f(-board_data.left_to_right_distance_m * 100, 0, 0);
                right_camera.extrinsics.specTranslation = dai::Point3f((board_data.left_to_right_distance_m - board_data.left_to_rgb_distance_m) * 100, 0, 0);
                left_camera.specHfovDeg = board_data.left_fov_deg;
                right_camera.specHfovDeg = board_data.left_fov_deg;
        }

        board_data_config.cameraData[left] = left_camera;
        board_data_config.cameraData[right] = right_camera;
        board_data_config.cameraData[dai::CameraBoardSocket::CAM_A] = rgb_camera;
        board_data_config.cameraData[dai::CameraBoardSocket::CAM_A].lensPosition = 135;
    }

    return EEPROM_OK;
}

std::tuple<bool, std::string> eeprom_clear(eeprom_area area, bool protectedPermissions, bool factoryPermissions, dai::CameraBoardSocket camSocket) {
    if(!protectedPermissions) {
        return {false, "No Protected permissions to clear EEPROM"};
    }
    if(area == EEPROM_AREA_FACTORY && !factoryPermissions) {
        return {false, "No Factory permissions to clear factory area EEPROM"};
    }

    uint8_t eeprom_erase_data[SIZE_OF_CONFIGURATION];
    for(unsigned i = 0; i < SIZE_OF_CONFIGURATION; i++) {
        eeprom_erase_data[i] = 0xFF;
    }
    eeprom_status status = eeprom_store(eeprom_erase_data, SIZE_OF_CONFIGURATION, 0, area, camSocket);

    switch(status) {
        case EEPROM_OK:
            return {true, ""};
        case EEPROM_NOT_AVAILABLE:
            printf("EEPROM not available to perform a clear\n");
            return {false, "EEPROM not available to perform a clear"};
        default:
            printf("Couldn't flash factory defaults to USER area\n");
            return {false, "Couldn't clear EEPROM contents, error: " + std::string(eeprom_status_to_str(status))};
    }
}

eeprom_status eeprom_load_board_options(uint32_t& options, eeprom_area area, dai::CameraBoardSocket camSocket) {
    uint32_t version = eeprom_get_version(area, camSocket);
    if(version == EEPROM_VERSION_V7 && version < EEPROM_VERSION_UNPROGRAMMED) {
        gen2_eeprom_ex_data ex_data;
        static_assert(sizeof(gen2_eeprom_data) == 1810);  // The ex_data offset is correct only while gen2_eeprom_data keeps this size
        int ret = eeprom_read(&ex_data, sizeof(ex_data), sizeof(gen2_eeprom_data), area, camSocket);
        if(ret) {
            return EEPROM_ERROR;
        }

        if(ex_data.board_options != UINT32_MAX) {
            options = ex_data.board_options;
            return EEPROM_OK;
        }
    }

    return EEPROM_ERROR;
}

eeprom_status eeprom_get_board_config_area(eeprom_area& area, dai::CameraBoardSocket camSocket) {
    area = EEPROM_AREA_FACTORY;

    uint32_t boardOptions = 0;

    if(eeprom_load_board_options(boardOptions, EEPROM_AREA_USER, camSocket) == EEPROM_OK) {
        if(boardOptions & EEPROM_BOARD_OPTIONS_USE_BOARD_CONFIG_USER_AREA) {
            area = EEPROM_AREA_USER;
            return EEPROM_OK;
        }
    }

    if(eeprom_load_board_options(boardOptions, EEPROM_AREA_FACTORY, camSocket) == EEPROM_OK) {
        area = EEPROM_AREA_FACTORY;
        return EEPROM_OK;
    } else {
        return EEPROM_NO_BOARD_CONFIG;
    }
}
