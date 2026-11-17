#include "CameraSingleton.hpp"

#include <camera/CameraMetadata.h>
#include <camera/VendorTagDescriptor.h>
#include <color_metadata.h>
#include <cutils/properties.h>
#include <fcntl.h>
#include <gbm_priv.h>
#include <hardware/camera.h>
#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "LuxonisSidechannel.hpp"
#include "eeprom.h"
#include "utilities/CbaSensorDriver.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/OSUtils.hpp"
#include "utilities/StereoPairs.hpp"

using namespace android;
using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
using VendorTagDescriptor = android::hardware::camera::common::V1_0::helper::VendorTagDescriptor;

extern dai::EepromData getCalibration();

// #define ENABLE_DEBUG
#ifdef ENALBE_DEBUG
    #define DEBUG(...) fprintf(stderr, ##__VA_ARGS__)
#else
    #define DEBUG(...)
#endif

namespace {

constexpr float FSYNC_FPS_TOLERANCE = 0.05f;
constexpr std::string_view DT_SOC_PATH = "/sys/firmware/devicetree/base/soc";
constexpr std::string_view DT_ALIASES_PATH = "/sys/firmware/devicetree/base/aliases";
constexpr std::string_view SYS_SOC_PATH = "/sys/devices/platform/soc";

bool fpsClose(float lhs, float rhs) {
    return std::fabs(lhs - rhs) <= FSYNC_FPS_TOLERANCE;
}

bool isStereoPair(const std::vector<dai::StereoPair>& stereoPairs, dai::CameraBoardSocket first, dai::CameraBoardSocket second) {
    return std::find_if(
               stereoPairs.begin(),
               stereoPairs.end(),
               [first, second](const auto& pair) { return (pair.left == first && pair.right == second) || (pair.left == second && pair.right == first); })
           != stereoPairs.end();
}

std::optional<uint32_t> readBe32(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input.is_open()) {
        return std::nullopt;
    }

    std::uint8_t bytes[4] = {};
    input.read(reinterpret_cast<char*>(bytes), sizeof(bytes));
    if(input.gcount() != static_cast<std::streamsize>(sizeof(bytes))) {
        return std::nullopt;
    }

    return (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) | (static_cast<uint32_t>(bytes[2]) << 8)
           | static_cast<uint32_t>(bytes[3]);
}

std::optional<std::string> readDtString(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if(!input.is_open()) {
        return std::nullopt;
    }

    std::stringstream buffer;
    buffer << input.rdbuf();
    return std::string(buffer.str().c_str());
}

std::optional<std::pair<int, std::string>> parseCciNode(const std::string& cciNodeName) {
    constexpr std::string_view prefix = "qcom,cci";
    const auto prefixPos = cciNodeName.find(prefix);
    const auto atPos = cciNodeName.find('@');
    if(prefixPos == std::string::npos || atPos == std::string::npos || atPos <= prefixPos + prefix.size()) {
        return std::nullopt;
    }

    try {
        const int cciIndex = std::stoi(cciNodeName.substr(prefixPos + prefix.size(), atPos - (prefixPos + prefix.size())));
        return std::pair<int, std::string>{cciIndex, cciNodeName.substr(atPos + 1)};
    } catch(const std::exception&) {
        return std::nullopt;
    }
}

std::vector<int> getI2cBusesForCci(const std::filesystem::path& cciSysPath) {
    std::vector<int> buses;
    if(!std::filesystem::exists(cciSysPath)) {
        return buses;
    }

    for(const auto& entry : std::filesystem::directory_iterator(cciSysPath)) {
        if(!entry.is_directory()) {
            continue;
        }
        const auto name = entry.path().filename().string();
        constexpr std::string_view prefix = "i2c-";
        if(name.rfind(prefix.data(), 0) != 0) {
            continue;
        }
        try {
            buses.push_back(std::stoi(name.substr(prefix.size())));
        } catch(const std::exception&) {
        }
    }

    std::sort(buses.begin(), buses.end());
    return buses;
}

// I2C bus of every camera port (CSI) in the device tree, whether or not a camera is connected to it
std::map<uint32_t, int> findI2cBusesForCsis() {
    namespace fs = std::filesystem;

    std::map<uint32_t, int> busesForCsis;
    const fs::path dtSoc(DT_SOC_PATH);
    if(!fs::exists(dtSoc)) {
        return busesForCsis;
    }

    for(const auto& cciEntry : fs::directory_iterator(dtSoc)) {
        if(!cciEntry.is_directory()) {
            continue;
        }
        const auto parsedCci = parseCciNode(cciEntry.path().filename().string());
        if(!parsedCci.has_value()) {
            continue;
        }

        const auto& [cciIndex, addr] = *parsedCci;
        const fs::path cciSys = fs::path(SYS_SOC_PATH) / (addr + ".qcom,cci" + std::to_string(cciIndex));
        const auto buses = getI2cBusesForCci(cciSys);
        if(buses.empty()) {
            continue;
        }

        for(const auto& sensorEntry : fs::directory_iterator(cciEntry.path())) {
            if(!sensorEntry.is_directory()) {
                continue;
            }
            const auto sensorName = sensorEntry.path().filename().string();
            if(sensorName.rfind("qcom,cam-sensor", 0) != 0) {
                continue;
            }

            const auto csiIndex = readBe32(sensorEntry.path() / "csiphy-sd-index");
            const auto masterIndex = readBe32(sensorEntry.path() / "cci-master");
            if(!csiIndex.has_value() || !masterIndex.has_value()) {
                continue;
            }
            if(*masterIndex >= buses.size()) {
                DAI_WARN("CSI{} uses CCI master {} but only found buses {}", *csiIndex, *masterIndex, buses.size());
                continue;
            }
            busesForCsis.emplace(*csiIndex, buses[*masterIndex]);
        }
    }

    return busesForCsis;
}

// Camera ports (CSI) per I2C bus as wired in the device tree, whether or not cameras are connected. EEPROMs at
// the same address on a bus shared by several ports can't be told apart
std::map<int, std::vector<uint32_t>> findCsisPerI2cBus() {
    std::map<int, std::vector<uint32_t>> csisPerBus;
    for(const auto& [csi, bus] : findI2cBusesForCsis()) {
        csisPerBus[bus].push_back(csi);
    }
    return csisPerBus;
}

std::optional<int> findI2cBusForCsi(uint32_t targetCsi) {
    const auto busesForCsis = findI2cBusesForCsis();
    const auto entry = busesForCsis.find(targetCsi);
    if(entry == busesForCsis.end()) {
        return std::nullopt;
    }
    return entry->second;
}

std::optional<uint32_t> findCsiForSocket(dai::CameraBoardSocket socket) {
    namespace fs = std::filesystem;

    const fs::path aliases(DT_ALIASES_PATH);
    if(!fs::exists(aliases)) {
        return std::nullopt;
    }

    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    if(sidechannel == nullptr) {
        return std::nullopt;
    }

    auto cb = sidechannel->ops.camera_socket_for_csi_get;
    if(cb == nullptr) {
        return std::nullopt;
    }

    for(const auto& aliasEntry : fs::directory_iterator(aliases)) {
        if(!aliasEntry.is_regular_file()) {
            continue;
        }
        const auto filename = aliasEntry.path().filename().string();
        if(filename.rfind("cam_", 0) != 0) {
            continue;
        }

        const auto aliasTarget = readDtString(aliasEntry.path());
        if(!aliasTarget.has_value()) {
            continue;
        }

        const auto csiIndex = readBe32(fs::path("/sys/firmware/devicetree/base" + *aliasTarget) / "csiphy-sd-index");
        if(!csiIndex.has_value()) {
            continue;
        }

        const auto result = static_cast<lux_sc_camera_socket_for_csi_get_ret_t>(cb(*csiIndex));
        if(result.found && static_cast<dai::CameraBoardSocket>(result.depthai_socket) == socket) {
            return csiIndex;
        }
    }

    return std::nullopt;
}

// Camera socket names of the camera ports (CSI); "CSI<n>" for a port without a socket
std::string cameraSocketNames(const std::vector<uint32_t>& csis) {
    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    const auto socketForCsi = sidechannel != nullptr ? sidechannel->ops.camera_socket_for_csi_get : nullptr;
    std::string names;
    for(const auto csi : csis) {
        if(!names.empty()) names += ", ";
        if(socketForCsi != nullptr) {
            const auto result = static_cast<lux_sc_camera_socket_for_csi_get_ret_t>(socketForCsi(csi));
            if(result.found) {
                names += dai::toString(static_cast<dai::CameraBoardSocket>(result.depthai_socket));
                continue;
            }
        }
        names += fmt::format("CSI{}", csi);
    }
    return names;
}

// Removes an at24 device that earlier firmware registered at `address`. A bound kernel driver would block
// /dev/i2c access at that address and, on a shared bus, expose every module's EEPROM at once.
void deleteAt24Device(int bus, uint16_t address) {
    const auto busPath = std::filesystem::path("/sys/bus/i2c/devices") / fmt::format("i2c-{}", bus);
    if(std::filesystem::exists(busPath / fmt::format("{}-{:04x}", bus, address))) {
        std::ofstream(busPath / "delete_device") << fmt::format("0x{:02x}\n", address);
    }
}

// Reads one byte through /dev/i2c-<bus> to check whether an EEPROM answers at `address`
bool i2cEepromResponds(int bus, uint16_t address) {
    std::string error;
    const int fd = i2c_eeprom_open(bus, address, error);
    if(fd < 0) {
        DAI_LOG("I2C EEPROM probe on bus {} at 0x{:02x} failed: {}", bus, address, error);
        return false;
    }
    uint8_t byte = 0;
    error = i2c_eeprom_read(fd, address, &byte, 1, 0);
    close(fd);
    return error.empty();
}

// Address a CBA EEPROM on `bus` answers at, if any. One EEPROM per bus, and which of its two addresses it is at
// depends on the state of the sensor's RESET line, so probe both
std::optional<uint16_t> findCbaEepromAddress(int bus) {
    for(const auto address : CameraSingleton::CBA_EEPROM_ADDRESSES) {
        deleteAt24Device(bus, address);
        if(i2cEepromResponds(bus, address)) {
            return address;
        }
    }
    return std::nullopt;
}

// Board config of the CBA EEPROM at `address` on `bus`. Runs inside CameraSingleton::init() (initializeCameraModule()),
// so calling CameraSingleton::getInstance() from here would re-enter the singleton's construction. That is why the
// EEPROM is opened by bus and address directly
std::optional<dai::EepromData> readCbaBoardConfig(int bus, uint16_t address) {
    // eeprom.cpp keeps open EEPROMs per socket. None can be open yet because eeprom_open(socket) needs CameraSingleton, so CAM_A's slot is free
    constexpr auto socket = dai::CameraBoardSocket::CAM_A;
    if(eeprom_open_cba(socket, bus, address) < 0) {
        return std::nullopt;
    }
    std::optional<dai::EepromData> boardConfig;
    eeprom_area area = EEPROM_AREA_FACTORY;
    if(eeprom_get_board_config_area(area, socket) != EEPROM_OK) {
        DAI_LOG("CBA EEPROM at 0x{:02x} on I2C bus {} has no board config", address, bus);
    } else {
        dai::EepromData data;
        const auto status = eeprom_load_config(data, area, socket);
        if(status == EEPROM_OK) {
            boardConfig = std::move(data);
        } else {
            DAI_WARN("Couldn't load the board config of the CBA EEPROM at 0x{:02x} on I2C bus {}: {}", address, bus, eeprom_status_to_str(status));
        }
    }
    eeprom_close(socket);
    return boardConfig;
}

struct CbaSensorSelection {
    int bus;
    uint16_t address;
    std::string boardConf;
    std::string_view driver;
};

// The sensor driver that the CBA EEPROM on `bus`, the I2C bus of camera port `csi`, selects. Only the device tree,
// sysfs and /dev/i2c are used, CamX isn't loaded yet
std::optional<CbaSensorSelection> selectSensorDriverFromCbaEeprom(uint32_t csi, int bus, const std::map<uint32_t, int>& busesForCsis) {
    // EEPROMs at the same address on a shared bus can't be told apart
    if(std::count_if(busesForCsis.begin(), busesForCsis.end(), [bus](const auto& csiBus) { return csiBus.second == bus; }) > 1) {
        DAI_LOG("Not selecting a sensor driver for CSI{}: its CBA EEPROM shares I2C bus {} with other camera ports", csi, bus);
        return std::nullopt;
    }

    const auto address = findCbaEepromAddress(bus);
    if(!address.has_value()) {
        DAI_LOG("Not selecting a sensor driver for CSI{}: no CBA EEPROM on I2C bus {}", csi, bus);
        return std::nullopt;
    }
    auto boardConfig = readCbaBoardConfig(bus, *address);
    if(!boardConfig.has_value()) {
        return std::nullopt;
    }
    const auto driver = dai::gate::cbaSensorDriverForBoardConf(boardConfig->boardConf);
    if(!driver.has_value()) {
        DAI_LOG("Not selecting a sensor driver for CSI{}: CBA EEPROM board config '{}' is not a known camera board", csi, boardConfig->boardConf);
        return std::nullopt;
    }
    return CbaSensorSelection{bus, *address, std::move(boardConfig->boardConf), *driver};
}

// Value of LUX_SC_CLIENT_SENSOR_SELECT: "<csi>:<driver>" for every camera port whose CBA EEPROM selects a sensor
// driver, comma separated
std::string selectSensorDriversFromCbaEeproms() {
    const auto busesForCsis = findI2cBusesForCsis();
    std::string value;
    for(const auto& [csi, bus] : busesForCsis) {
        const auto selection = selectSensorDriverFromCbaEeprom(csi, bus, busesForCsis);
        if(!selection.has_value()) {
            continue;
        }
        // The firmware's early log lines don't always reach the host, stdout ends up in the depthai_gate journal. Flush so the
        // line is there even if the process dies while CamX is still starting
        printf("CBA EEPROM csi %u bus %d addr 0x%02x board config %s selects sensor driver %.*s\n",
               csi,
               selection->bus,
               static_cast<unsigned>(selection->address),
               selection->boardConf.c_str(),
               static_cast<int>(selection->driver.size()),
               selection->driver.data());
        fflush(stdout);
        if(!value.empty()) {
            value += ',';
        }
        value += fmt::format("{}:{}", csi, selection->driver);
    }
    return value;
}

// CamX reads the same name, like LUX_SC_CLIENT_VERSION_MAJOR and LUX_SC_CLIENT_VERSION_MINOR
constexpr const char* SENSOR_SELECT_ENV = "LUX_SC_CLIENT_SENSOR_SELECT";

// CamX enumerates and probes the sensors while camera.qcom.so is being loaded by hw_get_module(), so the selection has to
// be in the environment by then. It is only a preference, CamX falls back to probing for the sensor's chip ID. Always
// set, so a value inherited from the parent process never leaks in
void setSensorSelectionEnv() {
    std::string value;
    try {
        value = selectSensorDriversFromCbaEeproms();
    } catch(const std::exception& e) {
        DAI_WARN("Not selecting any sensor driver, reading the CBA EEPROMs failed: {}", e.what());
    }
    if(setenv(SENSOR_SELECT_ENV, value.c_str(), 1) != 0) {
        DAI_WARN("Couldn't set {}", SENSOR_SELECT_ENV);
    }
    printf("%s=%s\n", SENSOR_SELECT_ENV, value.c_str());
    fflush(stdout);
}

}  // namespace

camera_module_t* CameraSingleton::initializeCameraModule() {
    camera_module_t* camM{nullptr};
    vendor_tag_ops_t vendor_tag_ops_;

    auto deviceState = dai::gate::getDeviceState();
    if(deviceState == dai::gate::DeviceState::SETUP || deviceState == dai::gate::DeviceState::ERROR) {
        throw std::runtime_error(
            "The OAK4 has not been setup yet. Please complete the setup by following the instructions at https://setup.luxonis.com/ before using DepthAI.");
    }

    LuxonisSidechannel::earlyInit();
    setSensorSelectionEnv();
    auto rc = hw_get_module(CAMERA_HARDWARE_MODULE_ID, (const hw_module_t**)&camM);
    if(rc != 0 || camM == nullptr) {
        printf("HW loading failed\n");
        return nullptr;
    }
    DEBUG("Camera Module author: %s, version: %d name: %s\n", camM->common.author, camM->common.hal_api_version, camM->common.name);
    LuxonisSidechannel::initialize(camM->common.dso);
    auto numCameras = camM->get_number_of_cameras();
    DEBUG("Number of cameras: %d\n", numCameras);
    if(NULL != camM->init) {
        auto res = camM->init();
        if(0 != res) {
            printf("Failed to initialize Camera Hal module!");
            return nullptr;
        }
    }
    if(camM->get_vendor_tag_ops) {
        vendor_tag_ops_ = vendor_tag_ops_t();
        camM->get_vendor_tag_ops(&vendor_tag_ops_);

        sp<VendorTagDescriptor> vendor_tag_desc;
        auto err = VendorTagDescriptor::createDescriptorFromOps(&vendor_tag_ops_, vendor_tag_desc);

        if(0 != err) {
            printf(
                "Could not generate descriptor from vendor tag operations,"
                "received error %s (%d). Camera clients will not be able to use"
                "vendor tags",
                strerror(err),
                err);
            return nullptr;
        }

        err = VendorTagDescriptor::setAsGlobalVendorTagDescriptor(vendor_tag_desc);

        if(0 != err) {
            printf(
                "Could not set vendor tag descriptor, "
                "received error %s (%d). \n",
                strerror(-err),
                err);
            return nullptr;
        }
    }
    cameraInfos.resize(numCameras);
    for(int i = 0; i < numCameras; i++) {
        auto rc = camM->get_camera_info(i, &cameraInfos[i]);
        if(!rc) {
            uint32_t tagId = 0;
            char* sensorName;
            sp<VendorTagDescriptor> vTags = VendorTagDescriptor::getGlobalVendorTagDescriptor();
            CameraMetadata::getTagFromName("org.codeaurora.qcamera3.sensorName.sensorName", vTags.get(), &tagId);
            camera_metadata_entry tagEntry = {};
            int ret = find_camera_metadata_entry(const_cast<camera_metadata_t*>(cameraInfos[i].static_camera_characteristics), tagId, &tagEntry);
            if(ret == 0) {
                sensorName = (char*)tagEntry.data.u8;
                DEBUG("Camera: %d face:%d name:%s\n", i, cameraInfos[i].facing, sensorName);
            } else {
                DEBUG("Camera: %d face:%d\n", i, cameraInfos[i].facing);
            }
        } else {
            printf("Error Get Camera:%d Info \n", i);
            return nullptr;
        }
    }
    DEBUG("Hello world 2\n");
    return camM;
}

void CameraSingleton::init() {
    DEBUG("Camera init");
    cameraModule = initializeCameraModule();
    DAI_CHECK_IN(cameraModule);
    probeCbaEeproms();
    checkCcmEepromBuses();
}

void CameraSingleton::registerSensorConfig(dai::CameraBoardSocket socket,
                                           dai::CameraControl::FrameSyncMode frameSyncMode,
                                           const SensorConfigInfo& sensorConfig) {
    std::lock_guard<std::mutex> lock(registeredSensorConfigsMtx);

    const RegisteredSensorConfig newConfig{socket, frameSyncMode, sensorConfig};
    auto existingConfig =
        std::find_if(registeredSensorConfigs.begin(), registeredSensorConfigs.end(), [socket](const auto& config) { return config.socket == socket; });
    if(existingConfig != registeredSensorConfigs.end()) {
        *existingConfig = newConfig;
    } else {
        registeredSensorConfigs.push_back(newConfig);
    }
}

bool CameraSingleton::isFsyncEnabled(dai::CameraBoardSocket socket) {
    std::lock_guard<std::mutex> lock(registeredSensorConfigsMtx);

    auto currentConfig =
        std::find_if(registeredSensorConfigs.begin(), registeredSensorConfigs.end(), [socket](const auto& config) { return config.socket == socket; });
    if(currentConfig == registeredSensorConfigs.end()) {
        throw std::runtime_error("Camera socket was not registered before fsync evaluation.");
    }

    for(const auto& config : registeredSensorConfigs) {
        if(config.frameSyncMode == dai::CameraControl::FrameSyncMode::OUTPUT) {
            throw std::runtime_error("Frame sync OUTPUT mode is unsupported on RVC4. Please contact support if you need this functionality.");
        }
    }

    std::vector<size_t> activeConfigs;
    std::vector<size_t> explicitInputConfigs;
    activeConfigs.reserve(registeredSensorConfigs.size());
    explicitInputConfigs.reserve(registeredSensorConfigs.size());
    for(size_t index = 0; index < registeredSensorConfigs.size(); ++index) {
        const auto mode = registeredSensorConfigs[index].frameSyncMode;
        if(mode == dai::CameraControl::FrameSyncMode::OFF) {
            continue;
        }

        activeConfigs.push_back(index);
        if(mode == dai::CameraControl::FrameSyncMode::INPUT) {
            explicitInputConfigs.push_back(index);
        }
    }

    std::set<size_t> enabledConfigs;
    std::optional<float> selectedFps;
    if(!explicitInputConfigs.empty()) {
        selectedFps = registeredSensorConfigs[explicitInputConfigs.front()].sensorConfig.fps;
        for(const auto index : explicitInputConfigs) {
            if(!fpsClose(registeredSensorConfigs[index].sensorConfig.fps, *selectedFps)) {
                throw std::runtime_error(
                    "This configuration is not supported as only 1 fsync fps is supported for all sensors and they should disable fsync or switch fps to be "
                    "the same.");
            }
            enabledConfigs.insert(index);
        }
    }

    const auto stereoPairs = dai::gate::getStereoPairs(getCalibration());
    struct StereoCandidate {
        size_t firstIndex = 0;
        size_t leftIndex = 0;
        size_t rightIndex = 0;
    };
    std::vector<StereoCandidate> stereoCandidates;
    stereoCandidates.reserve(stereoPairs.size());
    for(size_t leftIndex = 0; leftIndex < activeConfigs.size(); ++leftIndex) {
        for(size_t rightIndex = leftIndex + 1; rightIndex < activeConfigs.size(); ++rightIndex) {
            const auto firstConfigIndex = activeConfigs[leftIndex];
            const auto secondConfigIndex = activeConfigs[rightIndex];
            if(isStereoPair(stereoPairs, registeredSensorConfigs[firstConfigIndex].socket, registeredSensorConfigs[secondConfigIndex].socket)) {
                stereoCandidates.push_back({std::min(firstConfigIndex, secondConfigIndex), firstConfigIndex, secondConfigIndex});
            }
        }
    }
    std::sort(stereoCandidates.begin(), stereoCandidates.end(), [](const auto& first, const auto& second) { return first.firstIndex < second.firstIndex; });

    if(!selectedFps.has_value() && !stereoCandidates.empty()) {
        const auto& prioritizedStereo = stereoCandidates.front();
        const auto leftFps = registeredSensorConfigs[prioritizedStereo.leftIndex].sensorConfig.fps;
        const auto rightFps = registeredSensorConfigs[prioritizedStereo.rightIndex].sensorConfig.fps;
        const auto hasMatchingFpsWithAnotherSensor = [&](size_t candidateIndex) {
            const auto candidateFps = registeredSensorConfigs[candidateIndex].sensorConfig.fps;
            return std::find_if(activeConfigs.begin(),
                                activeConfigs.end(),
                                [&](size_t otherIndex) {
                                    return otherIndex != candidateIndex && fpsClose(candidateFps, registeredSensorConfigs[otherIndex].sensorConfig.fps);
                                })
                   != activeConfigs.end();
        };
        const auto leftHasExternalMatch = hasMatchingFpsWithAnotherSensor(prioritizedStereo.leftIndex);
        const auto rightHasExternalMatch = hasMatchingFpsWithAnotherSensor(prioritizedStereo.rightIndex);

        if(fpsClose(leftFps, rightFps)) {
            selectedFps = leftFps;
            enabledConfigs.insert(prioritizedStereo.leftIndex);
            enabledConfigs.insert(prioritizedStereo.rightIndex);
        } else if(leftHasExternalMatch != rightHasExternalMatch) {
            selectedFps = leftHasExternalMatch ? leftFps : rightFps;
        } else {
            selectedFps = leftFps;
        }
    }

    for(const auto& candidate : stereoCandidates) {
        const auto leftFps = registeredSensorConfigs[candidate.leftIndex].sensorConfig.fps;
        const auto rightFps = registeredSensorConfigs[candidate.rightIndex].sensorConfig.fps;
        if(!fpsClose(leftFps, rightFps)) {
            continue;
        }

        if(!selectedFps.has_value()) {
            selectedFps = leftFps;
        }
        if(fpsClose(leftFps, *selectedFps) && fpsClose(rightFps, *selectedFps)) {
            enabledConfigs.insert(candidate.leftIndex);
            enabledConfigs.insert(candidate.rightIndex);
        }
    }

    for(const auto index : activeConfigs) {
        if(enabledConfigs.count(index) > 0) {
            continue;
        }

        const auto fps = registeredSensorConfigs[index].sensorConfig.fps;
        if(!selectedFps.has_value()) {
            selectedFps = fps;
        }
        if(fpsClose(fps, *selectedFps)) {
            enabledConfigs.insert(index);
        }
    }

    for(const auto index : explicitInputConfigs) {
        if(enabledConfigs.count(index) == 0) {
            throw std::runtime_error(
                "This configuration is not supported as only 1 fsync fps is supported across all sensors. "
                "Disable fsync for some sensors or switch fps to on the sensors to be the same.");
        }
    }

    if(currentConfig->frameSyncMode == dai::CameraControl::FrameSyncMode::OFF) {
        return false;
    }

    const auto currentIndex = static_cast<size_t>(std::distance(registeredSensorConfigs.begin(), currentConfig));
    return enabledConfigs.count(currentIndex) > 0;
}

void CameraSingleton::checkCcmEepromBuses() {
    try {
        for(const auto& [bus, csis] : findCsisPerI2cBus()) {
            deleteAt24Device(bus, CCM_EEPROM_ADDRESS);
            if(csis.size() > 1) {
                DAI_INFO("CCM EEPROM on I2C bus {}, shared by camera sockets {}, is accessible only while one of them is active", bus, cameraSocketNames(csis));
            }
        }
    } catch(const std::exception& ex) {
        // Optional EEPROM access must not prevent camera use.
        DAI_ERROR("CCM EEPROM bus check failed: {}", ex.what());
    }
}

int CameraSingleton::getCcmEepromBus(dai::CameraBoardSocket socket) {
    const auto csi = findCsiForSocket(socket);
    if(csi.has_value()) {
        for(const auto& [bus, csis] : findCsisPerI2cBus()) {
            if(std::find(csis.begin(), csis.end(), *csi) == csis.end()) {
                continue;
            }
            if(csis.size() > 1) {
                // On a shared bus 0x50 is the EEPROM of whichever modules are powered, i.e. of the active cameras
                std::vector<uint32_t> activeCsis;
                for(const auto active : getActiveCameras()) {
                    const auto activeCsi = findCsiForSocket(active);
                    if(activeCsi.has_value() && std::find(csis.begin(), csis.end(), *activeCsi) != csis.end()) {
                        activeCsis.push_back(*activeCsi);
                    }
                }
                if(activeCsis != std::vector<uint32_t>{*csi}) {
                    throw std::runtime_error(
                        fmt::format("CCM EEPROM of {} is not accessible: I2C bus {} is shared by {}, so it must be the only active camera on it (active: {})",
                                    dai::toString(socket),
                                    bus,
                                    cameraSocketNames(csis),
                                    activeCsis.empty() ? "none" : cameraSocketNames(activeCsis)));
                }
            }
            return bus;
        }
    }
    throw std::runtime_error(fmt::format("CCM EEPROM of {} is not accessible: its I2C bus isn't in the device tree", dai::toString(socket)));
}

void CameraSingleton::setCameraActive(dai::CameraBoardSocket socket, bool active) {
    std::lock_guard<std::mutex> lock(activeCamerasMtx);
    if(active) {
        activeCameras.insert(socket);
    } else {
        activeCameras.erase(socket);
    }
}

std::set<dai::CameraBoardSocket> CameraSingleton::getActiveCameras() {
    std::lock_guard<std::mutex> lock(activeCamerasMtx);
    return activeCameras;
}

void CameraSingleton::probeCbaEeproms() {
    std::set<dai::CameraBoardSocket> socketsToProbe;

    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    if(sidechannel == nullptr) {
        return;
    }

    const auto sensorsGet = sidechannel->ops.sensors_get;
    if(sensorsGet != nullptr) {
        lux_sc_sensors_get_result_t result{};
        if(sensorsGet(&result) == 0) {
            const auto count = std::min<int>(result.sensors_len, LUX_SC_MAX_CAMERA_SOCKETS);
            for(int index = 0; index < count; ++index) {
                const auto socket = static_cast<dai::CameraBoardSocket>(result.sensors[index].socket);
                if(socket != dai::CameraBoardSocket::AUTO) {
                    socketsToProbe.insert(socket);
                }
            }
        } else {
            DAI_WARN("Failed to query sensors for CBA EEPROM probing");
        }
    }

    if(socketsToProbe.empty() && sidechannel->ops.camera_socket_for_csi_get != nullptr) {
        for(int csiIndex = 0; csiIndex < LUX_SC_MAX_CAMERA_SOCKETS; ++csiIndex) {
            const auto result = static_cast<lux_sc_camera_socket_for_csi_get_ret_t>(sidechannel->ops.camera_socket_for_csi_get(csiIndex));
            if(result.found) {
                socketsToProbe.insert(static_cast<dai::CameraBoardSocket>(result.depthai_socket));
            }
        }
    }

    // One CBA EEPROM per bus, shared by the sockets on it
    std::map<int, std::optional<uint16_t>> addressByBus;
    for(const auto socket : socketsToProbe) {
        const auto csiIndex = findCsiForSocket(socket);
        if(!csiIndex.has_value()) {
            continue;
        }

        const auto bus = findI2cBusForCsi(*csiIndex);
        if(!bus.has_value()) {
            DAI_LOG("Skipping optional CBA EEPROM probing for socket {}: couldn't determine I2C bus for CSI{}", dai::toString(socket), *csiIndex);
            continue;
        }

        if(addressByBus.find(*bus) == addressByBus.end()) {
            addressByBus[*bus] = findCbaEepromAddress(*bus);
        }
        const auto address = addressByBus[*bus];
        if(!address.has_value()) {
            DAI_LOG("No optional CBA EEPROM detected for socket {} on bus {}", dai::toString(socket), *bus);
            continue;
        }
        DAI_LOG("CBA EEPROM for socket {} detected at 0x{:02x} on /dev/i2c-{}", dai::toString(socket), *address, *bus);
        std::lock_guard<std::mutex> lock(cbaEepromsMtx);
        cbaEeproms[socket] = {*bus, *address};
    }
}

std::optional<CameraSingleton::CbaEeprom> CameraSingleton::getCbaEeprom(dai::CameraBoardSocket socket) {
    std::lock_guard<std::mutex> lock(cbaEepromsMtx);
    const auto entry = cbaEeproms.find(socket);
    if(entry == cbaEeproms.end()) {
        return std::nullopt;
    }
    return entry->second;
}
