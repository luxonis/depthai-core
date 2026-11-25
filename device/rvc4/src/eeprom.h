#pragma once

#include <stdint.h>
#include "depthai/common/EepromData.hpp"

// Reserve the last two legacy camera slots for IMU calibration parameters.
#define MAX_CAMERAS (10 - 2)

// The EEPROM layout version is not changed by ading the IMU calibration parameters
// Use a magic number to identify if the IMU calibration parameters are present in the EEPROM
#define IMU_CALIB_PARAMS_MAGIC 0xC0DE
#define IMU_CALIB_PARAMS_SIZE 12

enum eeprom_area { EEPROM_AREA_USER, EEPROM_AREA_FACTORY };

enum eeprom_status {
    EEPROM_OK = 0,
    EEPROM_ERROR = -1,
    EEPROM_NO_FACTORY_PERMISSIONS = -2,
    EEPROM_NO_PROTECTED_PERMISSIONS = -3,
    EEPROM_INVALID_DATA = -4,
    EEPROM_INVALID_AREA = -5,
    EEPROM_NOT_AVAILABLE = -6,
    EEPROM_OUT_OF_BOUNDS = -7,
    EEPROM_FIELD_TOO_LONG = -8,
    EEPROM_NO_BOARD_CONFIG = -9,
};

enum eeprom_version {
    EEPROM_VERSION_UNPROGRAMMED = 0xFFFFFFFF,
    EEPROM_VERSION_V1 = 0x55AA0001,
    EEPROM_VERSION_V2,
    EEPROM_VERSION_V3,
    EEPROM_VERSION_V4,
    EEPROM_VERSION_V5,
    EEPROM_VERSION_V6,
    EEPROM_VERSION_V7,
    EEPROM_VERSION_LAST,
    EEPROM_VERSION_CURRENT = EEPROM_VERSION_LAST - 1
};

/*
 * EEPROM board options, bit index:
 * 33222222222211111111110000000000
 * 10987654321098765432109876543210
 *  |         |    |   |||||___||||_0: PCIe clock internal
 *  |         |    |   || |  |  ||__1: USB3 clock internal (no USB3 signals exposed)
 *  |         |    |   || |  |  |___2: MIPI all 3 ports 4-lane
 *  |         |    |   || |  |______7..3: Camera config (0x00-0x1F): 0x01: KeemBay-like (OAK-D-SR),
 *  |         |    |   || |                                          0x10: I2C different busses for all cams
 *  |         |    |   || |_________9..8: ETH Config (0x0-0x3): 0x00: Default, 0x01: PERST39(AH), ISOLATE17(AL)
 *  |         |    |   ||___________10: Dual-port device (Ethernet and USB, e.g: OAK-D-LR)
 *  |         |    |___|____________15..11: Default boot mode (0x00-0x1F): 0x00: GPIO bootstrap, else as defined by MyriadX.
 *  |         |_____________________20: 5bits PMIC config
 *  |_______________________________30: EEPROM use BoardConfig from User area (not Factory)
 */

enum eeprom_board_options {
    EEPROM_BOARD_OPTIONS_PCIE_CLOCK_INTERNAL = 0x1,
    EEPROM_BOARD_OPTIONS_USB3_CLOCK_INTERNAL = 0x2,
    EEPROM_BOARD_OPTIONS_MIPI_4LANE_RGB = 1U << 2,
    EEPROM_BOARD_OPTIONS_CAM_CONFIGURATION_MASK = 0b11111U << 3,  // bitfield, 5 bits
    EEPROM_BOARD_OPTIONS_CAM_I2C_SEPARATE_BUSES = 1U << 7,
    EEPROM_BOARD_OPTIONS_ETH_CONFIGURATION_MASK = 0b11U << 8,  // bitfield, 2 bits
    EEPROM_BOARD_OPTIONS_DUAL_PORT_DEVICE = 1U << 10,
    EEPROM_BOARD_OPTIONS_DEFAULT_BOOT_MODE_MASK = 0b11111U << 11,   // bitfield 5 bits
    EEPROM_BOARD_OPTIONS_PMIC_CONFIGURATION_MASK = 0b11111U << 20,  // bitfield 5 bits
    EEPROM_BOARD_OPTIONS_USE_BOARD_CONFIG_USER_AREA = 1U << 30,
};

enum eeprom_cam_configuration {
    EEPROM_BOARD_OPTIONS_CAM_CONF_1 = 1U << 3,
};

enum eeprom_eth_configuration {
    EEPROM_BOARD_OPTIONS_ETH_CONF_1 = 1U << 8,
};

enum eeprom_pmic_configuration {
    EEPROM_BOARD_OPTIONS_PMIC_CONF_1 = 1U << 20,
    EEPROM_BOARD_OPTIONS_PMIC_CONF_2 = 2U << 20,
    EEPROM_BOARD_OPTIONS_PMIC_CONF_3 = 3U << 20,
    EEPROM_BOARD_OPTIONS_PMIC_CONF_4 = 4U << 20,  // OV9782 1.2V DVDD on center port (instead of IMX378 1.05V DVDD)
};

enum eeprom_bitfield : uint8_t {
    EEPROM_BITFIELD_NOT_USE_SPEC_TRANSLATION = 1U << 0,
    EEPROM_BITFIELD_ENABLE_DISTORTION_CORRECTION = 1U << 1,
    EEPROM_BITFIELD_NOT_VALID = 1U << 7,
};

struct eeprom_data {
    // EEPROM_VERSION_V1
    uint32_t version;
    float calib_R1_left[9];  // calib_homography_right_to_left for <=V3, calib_H1_left for V4
    float left_fov_deg;
    float left_to_right_distance_m;
    float left_to_rgb_distance_m;
    bool swap_left_and_right_cameras;
    // EEPROM_VERSION_V2 additional fields
    float rgb_fov_deg;
    char board_name[16];
    char board_rev[10];
    // EEPROM_VERSION_V3 additional fields
    bool stereo_center_crop;
    // EEPROM_VERSION_V4 additional fields
    float calib_R2_right[9];  // calib_H2_right for V4
    float calib_M1_left[9];
    float calib_M2_right[9];
    float calib_R[9];
    float calib_T[3];
    // EEPROM_VERSION_V5 additional fields
    float calib_M3_rgb[9];
    float calib_R_rgb[9];
    float calib_T_rgb[3];
    float calib_d1_left[14];  // distortion coeffs
    float calib_d2_right[14];
    float calib_d3_rgb[14];
} __attribute__((packed));

struct point {
    float x;
    float y;
    float z;
} __attribute__((packed));

struct raw_extrinsics {
    float rotation[9];
    // (x, y, z) pose of dest_camera_socket w.r.t current_camera_socket obtained through calibration
    point translation;
    // (x, y, z) pose of dest_camera_socket w.r.t current_camera_socket measured through CAD design
    point measured_translation;
    int8_t to_camera_socket;
} __attribute__((packed));

struct stereo_rectification {
    float rectified_rotation_left[9];
    float rectified_rotation_right[9];
    uint8_t left_camera_socket;
    uint8_t right_camera_socket;
    // float baseline; // in mtrs ? or cm ?
} __attribute__((packed));

struct lux_camera_info {
    uint16_t height, width;
    uint8_t lens_position;
    float intrinsics[9];
    float distortion_coeff[14];
    float measured_fov_deg;
    int8_t camera_type;
    int8_t camera_id;
    raw_extrinsics extrinsics;
} __attribute__((packed));

struct gen2_info {
    uint32_t version;
    uint8_t num_cameras;
    uint32_t misc_data_size;
} __attribute__((packed));

struct gen2_eeprom_ex_data {
    char product_name[32];  // TBD, could also be mapped
    char board_conf[16];
    char hardware_conf[16];
    char batch_name[16];
    uint64_t batch_time;
    char board_custom[8];
    uint32_t board_options;
    char batch2_name[10];
    raw_extrinsics housing_extrinsics;
    int8_t vertical_camera_socket;
    uint8_t bitfield;
} __attribute__((packed));

struct gen2_eeprom_data {
    gen2_info header;
    char board_name[16];
    char board_rev[10];
    stereo_rectification stereo_data;
    raw_extrinsics imu_extrinsics;
    lux_camera_info camera_data[MAX_CAMERAS];
    union {
        uint8_t raw[2 * sizeof(lux_camera_info)];
        struct {
            uint16_t magic;
            uint8_t acc_calib_count;
            uint8_t gyro_calib_count;
            float acc_calib[IMU_CALIB_PARAMS_SIZE];
            float gyro_calib[IMU_CALIB_PARAMS_SIZE];
        } params;
    } imu;

} __attribute__((packed));

#define EEPROM_VERSION_SIZE sizeof(uint32_t)

int eeprom_open(dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);
// Opens the CBA EEPROM at `address` on camera I2C bus `bus` as the EEPROM of `camSocket`. Doesn't need CameraSingleton,
// so it also works while CameraSingleton is still initializing
int eeprom_open_cba(dai::CameraBoardSocket camSocket, int bus, uint16_t address);
int eeprom_close(dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);
bool eeprom_found(dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);

// eeprom_data access
eeprom_version eeprom_get_version(eeprom_area area, dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);
eeprom_status eeprom_store_config(dai::EepromData& board_data_config,
                                  eeprom_area area,
                                  bool factory_permissions,
                                  bool protected_permissions,
                                  dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO,
                                  std::string* errorDetail = nullptr);
eeprom_status eeprom_load_config(dai::EepromData& board_data_config, eeprom_area area, dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);
std::tuple<bool, std::string> eeprom_factory_reset(dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);
const char* eeprom_status_to_str(eeprom_status status);
eeprom_status eeprom_load_board_options(uint32_t& options, eeprom_area area, dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);
eeprom_status eeprom_get_board_config_area(eeprom_area& area, dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);

std::tuple<bool, std::string, std::vector<uint8_t>> eeprom_read_raw(eeprom_area area, dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);

// Direct access to a 24Cxx EEPROM (2-byte memory address) on a camera I2C bus through /dev/i2c-<bus>.
// Writes are chunked within pages and wait out each write cycle, so the busy EEPROM is never addressed:
// on the camera buses a NACK resets the I2C master shared with the streaming sensor.
// i2c_eeprom_open returns the fd, or -1 with `error` set; read/write return an empty string on success
int i2c_eeprom_open(int bus, uint16_t address, std::string& error);
std::string i2c_eeprom_read(int fd, uint16_t address, uint8_t* data, size_t size, size_t offset);
std::string i2c_eeprom_write(int fd, uint16_t address, const uint8_t* data, size_t size, size_t offset);

// Raw access to the camera-module (CCM) EEPROM; supported on RVC4 only
std::tuple<bool, std::string, std::vector<uint8_t>> ccm_eeprom_read_raw(dai::CameraBoardSocket camSocket, size_t size, size_t offset);
std::tuple<bool, std::string> ccm_eeprom_write_raw(dai::CameraBoardSocket camSocket, std::vector<uint8_t> data, size_t offset);

std::tuple<bool, std::string> eeprom_clear(eeprom_area area,
                                           bool protectedPermissions,
                                           bool factoryPermissions,
                                           dai::CameraBoardSocket camSocket = dai::CameraBoardSocket::AUTO);

// // Raw read access
// int eeprom_store(const void *data, size_t size, size_t add_offset);
// int eeprom_read(void *data, size_t size, size_t add_offset);

static inline bool eeprom_version_is_valid(uint32_t version) {
    return version >= EEPROM_VERSION_V1 && version < EEPROM_VERSION_LAST;
}
