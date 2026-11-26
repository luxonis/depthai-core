#include "CameraDiscovery.hpp"

#include <algorithm>
#include <mutex>
#include <utility>

#include <camera/CameraMetadata.h>
#include <hardware/camera3.h>
#include <spdlog/spdlog.h>

#ifdef __LIBGBM__
    #include <gbm.h>
#endif

#include <fstream>
#include <string>
#include <subprocess.hpp>
#include <unordered_map>

#include "camera/CameraSingleton.hpp"
#include "camera/LuxonisSidechannel.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/OSUtils.hpp"
#include "utilities/String.hpp"

#define LUX_CAM_DBG
#ifdef LUX_CAM_DBG
    #define DBG(A, ...) printf(A "\n", ##__VA_ARGS__)
#else
    #define DBG(A, ...)
#endif

namespace dai {
namespace gate {

CameraDiscovery::CameraDiscovery() {
    maybeInit();
}

CameraDiscovery& CameraDiscovery::getInstance() {
    static CameraDiscovery instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

#include <unistd.h>

using json = nlohmann::json;

static const char* cameraDescFile = "/tmp/camera_list.json";

typedef struct {
    int width;
    int height;
    float minFps;
    float maxFps;
    int fovX;
    int fovY;
    int fovW;
    int fovH;
    uint32_t capabilities;
} cam_config_t;

void from_json(const json& j, cam_config_t& config) {
    j.at("width").get_to(config.width);
    j.at("height").get_to(config.height);
    config.minFps = j.value("minFps", 10.0f);
    config.maxFps = j.value("maxFps", 30.0f);
    config.fovX = 0;
    config.fovY = 0;
    config.fovW = 0;
    config.fovH = 0;
    config.capabilities = 0;
}

typedef struct {
    int index;
    int socket;
    std::string name;
    std::string sensorName;
    int maxWidth;
    int maxHeight;
    float maxFPS;
    float minFPS;
    int autoFocus;
    int numConfigs;
    std::vector<cam_config_t> configs;
} cam_desc_t;

static void from_json(const json& j, cam_desc_t& desc) {
    j.at("index").get_to(desc.index);
    desc.socket = j.value("socket", -1);
    desc.name = j.value("socketName", j.value("name", ""));
    desc.sensorName = j.value("sensorName", desc.name);
    desc.autoFocus = j.value("autoFocus", 1);  // true by default for old os same as in old versions of depthai-device
    j.at("maxWidth").get_to(desc.maxWidth);
    j.at("maxHeight").get_to(desc.maxHeight);
    j.at("maxFPS").get_to(desc.maxFPS);
    j.at("minFPS").get_to(desc.minFPS);
    j.at("numConfigs").get_to(desc.numConfigs);

    auto configs = j.at("configs").get<std::vector<cam_config_t>>();
    desc.configs = std::move(configs);
}

static bool tryGetSensorsFromSidechannel(std::vector<cam_desc_t>& cameraArray) {
    auto& sc = LuxonisSidechannel::getInstance();
    auto cb = sc.ops.sensors_get;
    if(cb == nullptr) {
        return false;
    }

    lux_sc_sensors_get_result_t result{};
    if(cb(&result) != 0) {
        DAI_ERROR("lux_sc_sensors_get failed");
        return false;
    }

    int count = result.sensors_len;
    if(count <= 0) {
        return false;
    }
    count = std::min<int>(count, LUX_SC_MAX_CAMERA_SOCKETS);

    auto capsToString = [](uint32_t caps) {
        std::string s;
        if(caps & LUX_SC_SENSOR_HFR) {
            s += "HFR|";
        }
        if(caps & LUX_SC_SENSOR_HDR) {
            s += "HDR|";
        }
        if(!s.empty()) {
            s.pop_back();
        } else {
            s = "NONE";
        }
        return s;
    };

    cameraArray.clear();
    cameraArray.reserve(count);
    for(int i = 0; i < count; ++i) {
        const auto& s = result.sensors[i];
        cam_desc_t desc{};
        desc.index = s.index;
        desc.socket = s.socket;
        desc.name = std::string(s.socket_name);
        desc.sensorName = s.sensor_name[0] != '\0' ? std::string(s.sensor_name) : std::string(s.name);
        desc.maxWidth = s.max_width;
        desc.maxHeight = s.max_height;
        desc.maxFPS = s.max_fps;
        desc.minFPS = s.min_fps;
        desc.autoFocus = s.auto_focus;
        desc.numConfigs = s.configs_len;

        DAI_LOG("SC sensor: idx={} socket={} name={} sensor={} max={}x{} fps=[{:.2f}..{:.2f}] autofocus={} modes={}",
                s.index,
                s.socket,
                s.socket_name,
                s.sensor_name,
                s.max_width,
                s.max_height,
                s.min_fps,
                s.max_fps,
                s.auto_focus,
                s.configs_len);

        const auto cfgCount = std::min<uint32_t>(s.configs_len, 16);
        desc.configs.reserve(cfgCount);
        for(uint32_t j = 0; j < cfgCount; ++j) {
            const auto& cfg = s.configs[j];
            cam_config_t out{};
            out.width = static_cast<int>(cfg.width);
            out.height = static_cast<int>(cfg.height);
            out.minFps = cfg.min_fps;
            out.maxFps = cfg.max_fps;
            out.fovX = static_cast<int>(cfg.fov_x);
            out.fovY = static_cast<int>(cfg.fov_y);
            out.fovW = static_cast<int>(cfg.fov_width);
            out.fovH = static_cast<int>(cfg.fov_height);
            out.capabilities = cfg.capabilities;
            desc.configs.push_back(out);
            DAI_LOG("SC sensor mode: idx={} socket={} name={} sensor={} mode={}x{} fps=[{:.2f}..{:.2f}] fov=[{},{},{},{}] caps=0x{:08x} ({})",
                    s.index,
                    s.socket,
                    s.socket_name,
                    s.sensor_name,
                    cfg.width,
                    cfg.height,
                    cfg.min_fps,
                    cfg.max_fps,
                    cfg.fov_x,
                    cfg.fov_y,
                    cfg.fov_width,
                    cfg.fov_height,
                    cfg.capabilities,
                    capsToString(cfg.capabilities));
        }

        cameraArray.push_back(std::move(desc));
    }

    return true;
}

void CameraDiscovery::forceCameraDiscovery() {
    DBG("Forcing camera discovery");
    CameraSingleton::getInstance();
    return;
    // DBG("Forcing camera discovery DISABLED");
    // return;
}

std::vector<CameraDescription> CameraDiscovery::forceEnumerateCameras() {
    forceCameraDiscovery();
    std::vector<cam_desc_t> cameraArray;
    const bool gotSidechannel = tryGetSensorsFromSidechannel(cameraArray);
    if(!gotSidechannel) {
        std::ifstream input(cameraDescFile);
        if(!input.is_open()) {
            spdlog::error("Error opening file {}.\n", cameraDescFile);
            return cameras;
        }

        json j;
        input >> j;
        cameraArray = j.get<std::vector<cam_desc_t>>();
    }

    for(const auto& camera : cameraArray) {
        CameraDescription desc;
        std::string prefix = "LUXONIS_";
        std::string cameraName = std::string(camera.name);
        std::string sensorName = toUpperCase(camera.sensorName);

        if(sensorName.compare(0, prefix.length(), prefix) == 0) {
            sensorName.erase(0, prefix.length());
        }

        desc.camera.sensorName = sensorName;
        desc.camera.width = camera.maxWidth;
        desc.camera.height = camera.maxHeight;
        if(camera.socket == -1) {
            // would be nice to have a global logging instance / function
            // logger->warn("");
            std::cout
                << "Warning: couldn't find camera socket information. Maybe running an old OS version or something else went wrong. Fallbacking to camera id.\n"
                << std::flush;
            desc.camera.socket = static_cast<dai::CameraBoardSocket>(camera.index);
        } else {
            desc.camera.socket = static_cast<dai::CameraBoardSocket>(camera.socket);
        }
        desc.id = camera.index;
        desc.v4lpath.camera = "";
        desc.v4lpath.sensor = "";
        desc.v4lpath.lens = "";
        desc.camera.name = cameraName;
        desc.camera.hasAutofocus = (camera.autoFocus == 1);
        desc.camera.hasAutofocusIC = desc.camera.hasAutofocus;  // legacy support always same as hasAutofocus

        dai::CameraSensorType type = dai::CameraSensorType::COLOR;

        // The camera type info is not included in the json file.
        // Look on sensor name.
        if(sensorName.find("OV9282") != std::string::npos) {
            if(desc.camera.width == 0 || desc.camera.height == 0) {
                // os r851.0.25 support
                desc.camera.width = 1280;
                desc.camera.height = 800;
            }
            type = dai::CameraSensorType::MONO;
        } else if(sensorName.find("OG05B1B") != std::string::npos || sensorName.find("OG01A1B") != std::string::npos) {
            type = dai::CameraSensorType::MONO;
        } else if(sensorName.find("SAMSUNG33D") != std::string::npos) {
            type = dai::CameraSensorType::TOF;
        } else if(sensorName.find("IMX316") != std::string::npos) {
            type = dai::CameraSensorType::TOF;
        } else if(sensorName.find("VD55H1") != std::string::npos) {
            type = dai::CameraSensorType::TOF;
        } else if(sensorName.find("EAGLE") != std::string::npos) {
            type = dai::CameraSensorType::THERMAL;  // Seek Eagle: only the Thermal node streams it
        }

        // os < r851.1.1 support
        if(dai::gate::getOSVersion() <= dai::Version(1, 1, 0) && sensorName.find("IMX586") != std::string::npos) {
            // Override the resolution, as the actual full resolution is not supported yet
            desc.camera.width = 4000;
            desc.camera.height = 3000;
        }
        desc.camera.supportedTypes.push_back(type);
        struct ModeKey {
            int width;
            int height;
            float maxFps;
        };
        std::vector<ModeKey> modeSeen;
        // Seek Eagle: the 648x492 sensor mode only captures the core's flash passthrough for the Seek SDK (SeekEagle.cpp)
        const bool eagle = sensorName.find("EAGLE") != std::string::npos;
        const auto eagleFlashMode = [eagle](const cam_config_t& c) { return eagle && c.width == 648 && c.height == 492; };
        for(const auto& config : camera.configs) {
            if(!eagle || eagleFlashMode(config)) continue;
            desc.camera.width = config.width;
            desc.camera.height = config.height;
            break;
        }
        for(const auto& config : camera.configs) {
            if(eagleFlashMode(config)) continue;
            dai::CameraSensorConfig sensorConfig;
            sensorConfig.width = config.width;
            sensorConfig.height = config.height;
            sensorConfig.minFps = config.minFps;
            sensorConfig.maxFps = config.maxFps;
            sensorConfig.type = type;
            const auto fovW = (config.fovW > 0) ? config.fovW : desc.camera.width;
            const auto fovH = (config.fovH > 0) ? config.fovH : desc.camera.height;
            const auto fovX = (config.fovW > 0) ? config.fovX : 0;
            const auto fovY = (config.fovH > 0) ? config.fovY : 0;
            sensorConfig.fov = {static_cast<float>(fovX), static_cast<float>(fovY), static_cast<float>(fovW), static_cast<float>(fovH)};
            sensorConfig.hdr = (config.capabilities & LUX_SC_SENSOR_HDR) != 0;
            sensorConfig.hfr = (config.capabilities & LUX_SC_SENSOR_HFR) != 0;
            if(!gotSidechannel && config.maxFps > 120.0f) {
                sensorConfig.hfr = true;
            }
            if(!gotSidechannel) {
                int count = 0;
                for(const auto& m : modeSeen) {
                    if(m.width == config.width && m.height == config.height && m.maxFps == config.maxFps) {
                        count++;
                    }
                }
                if(count == 1) {
                    sensorConfig.hdr = true;
                }
                modeSeen.push_back({config.width, config.height, config.maxFps});
            }
            desc.camera.configs.push_back(sensorConfig);
        }
        cameras.push_back(desc);
    }

    // Create other types of views
    cameraSockets = {};
    for(auto cam : cameras) {
        cameraSockets.push_back(cam.camera.socket);
    }
    cameraFeatures = {};
    for(auto cam : cameras) {
        cameraFeatures.push_back(cam.camera);
    }
    cameraNames = {};
    for(auto cam : cameras) {
        cameraNames[cam.camera.socket] = cam.camera.sensorName;
    }

    // Still return cameras if needed
    return cameras;
}  // namespace gate

// Lock for this length of operation
std::vector<CameraDescription> CameraDiscovery::getConnectedCameraDescriptions() const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    return cameras;
}
std::vector<dai::CameraBoardSocket> CameraDiscovery::getConnectedCameras() const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    return cameraSockets;
}
std::vector<dai::CameraFeatures> CameraDiscovery::getConnectedCameraFeatures() const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    return cameraFeatures;
}
std::unordered_map<dai::CameraBoardSocket, std::string> CameraDiscovery::getConnectedCameraSensorNames() const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    return cameraNames;
}

/// Returns corresponding socket or AUTO if not available
dai::CameraBoardSocket CameraDiscovery::getCameraByName(std::string name) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    for(const auto& cam : cameras) {
        if(cam.camera.name == name) {
            return cam.camera.socket;
        }
    }
    return dai::CameraBoardSocket::AUTO;
}
/// Returns corresponding id or -1 if not available
int CameraDiscovery::getCameraId(std::string name) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    for(const auto& cam : cameras) {
        if(cam.camera.name == name) {
            return cam.id;
        }
    }
    return -1;
}
/// Returns corresponding id or -1 if not available
int CameraDiscovery::getCameraId(dai::CameraBoardSocket socket) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    for(const auto& cam : cameras) {
        if(cam.camera.socket == socket) {
            return cam.id;
        }
    }
    return -1;
}
// Returns corresponding id or -1 if not available
int CameraDiscovery::getCameraId(std::string name, dai::CameraBoardSocket socket) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    for(const auto& cam : cameras) {
        if(!name.empty() && cam.camera.name == name) {
            return cam.id;
        }
        if(socket != dai::CameraBoardSocket::AUTO && cam.camera.socket == socket) {
            return cam.id;
        }
    }
    return -1;
}

// Returns corresponding id or -1 if not available
dai::CameraFeatures CameraDiscovery::getCameraFeatures(std::string name, dai::CameraBoardSocket socket) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    for(const auto& cam : cameras) {
        if(!name.empty() && cam.camera.name == name) {
            return cam.camera;
        }
        if(socket != dai::CameraBoardSocket::AUTO && cam.camera.socket == socket) {
            return cam.camera;
        }
    }
    return dai::CameraFeatures{.socket = dai::CameraBoardSocket::AUTO};
}

// Returns corresponding id or -1 if not available
CameraDescription CameraDiscovery::getCameraDescription(std::string name, dai::CameraBoardSocket socket) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    for(const auto& cam : cameras) {
        if(!name.empty() && cam.camera.name == name) {
            return cam;
        }
        if(socket != dai::CameraBoardSocket::AUTO && cam.camera.socket == socket) {
            return cam;
        }
    }
    return CameraDescription{.id = -1};
}

std::vector<std::pair<uint32_t, uint32_t>> CameraDiscovery::getAvailableISPScalers(dai::CameraBoardSocket socket) const {
    std::unique_lock<std::mutex> lock(mtx);
    DAI_CHECK_IN(initialized);
    int cameraId = -1;
    for(const auto& cam : cameras) {
        if(socket != dai::CameraBoardSocket::AUTO && cam.camera.socket == socket) {
            cameraId = cam.id;
            break;
        }
    }
    lock.unlock();

    if(cameraId < 0) {
        return {};
    }

    using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
    CameraMetadata static_meta;
    static_meta = CameraSingleton::getInstance().cameraInfos[cameraId].static_camera_characteristics;

    camera_metadata_entry_t entry{};
    bool has_entry = false;

    if(static_meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
        entry = static_meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
        has_entry = true;
    }

    if(!has_entry) {
        if(!static_meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
            return {};
        }
        entry = static_meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
    }

    std::vector<std::pair<uint32_t, uint32_t>> out;
    out.reserve(entry.count / 4);

#ifdef __LIBGBM__
    constexpr int32_t kImplFormat = GBM_FORMAT_IMPLEMENTATION_DEFINED;
#else
    constexpr int32_t kImplFormat = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
#endif

    for(uint32_t i = 0; i + 3 < entry.count; i += 4) {
        const int32_t format = entry.data.i32[i + 0];
        const int32_t width = entry.data.i32[i + 1];
        const int32_t height = entry.data.i32[i + 2];
        const int32_t dir = entry.data.i32[i + 3];
        if(format == kImplFormat && dir == ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT) {
            out.emplace_back(static_cast<uint32_t>(width), static_cast<uint32_t>(height));
        }
    }

    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return (a.first < b.first) || (a.first == b.first && a.second < b.second); });
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

void CameraDiscovery::overrideConnectedCameraFeatures(std::vector<dai::CameraFeatures> features) {
    std::unique_lock<std::mutex> lock(mtx);
    this->cameraFeatures = std::move(features);
    this->cameraSockets.clear();
    this->cameraNames.clear();
    this->cameras.clear();
    this->cameraSockets.reserve(this->cameraFeatures.size());
    this->cameras.reserve(this->cameraFeatures.size());
    int cameraIdx = 0;
    for(const auto& feature : this->cameraFeatures) {
        this->cameraSockets.push_back(feature.socket);
        this->cameras.push_back(CameraDescription{feature, cameraIdx++, -1, {"", "", ""}});
        this->cameraNames[feature.socket] = feature.sensorName;
    }
}

void CameraDiscovery::maybeInit() {
    std::lock_guard lock(mtx);
    if(!initialized) {
        forceEnumerateCameras();
        initialized = true;
    }
}

}  // namespace gate
}  // namespace dai
