// std
#include <signal.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <sstream>
#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>

// Generated
#include "depthai/build/version.hpp"

// shared
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/BoardConfig.hpp"
#include "depthai/log/LogLevel.hpp"
#include "depthai/pipeline/Assets.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"
#include "depthai/properties/GlobalProperties.hpp"
#include "depthai/utility/Checksum.hpp"
#include "depthai/utility/Initialization.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

// libraries
#include "XLink/XLink.h"
#include "pipeline/node/NeuralNetwork/NeuralNetwork.hpp"
#include "pipeline/node/NeuralNetwork/rvc4/UdoRuntime.hpp"
#include "spdlog/fmt/chrono.h"
#include "spdlog/spdlog.h"

// project
#include "IMU.hpp"
#include "ImuExtrinsicsData.hpp"
#include "Logging.hpp"
#include "Timesync.hpp"
#include "XLinkRpc.hpp"
#include "eeprom.h"
#include "pipeline/MultiDeviceCalibrationStore.hpp"
#include "pipeline/PipelineBuilder.hpp"
#include "utilities/MatrixUtils.hpp"
#include "utilities/AnonymousTelemetryId.hpp"
#include "utilities/CalibrationAccess.hpp"
#include "utilities/CameraDiscovery.hpp"
#include "utilities/DeviceTelemetry.hpp"
#include "utilities/DeviceModel.hpp"
#include "utilities/GpuAvailability.hpp"
#include "utilities/IMUDiscovery.hpp"
#include "utilities/OSUtils.hpp"
#include "utilities/StereoPairs.hpp"
#include "vendor/dai_device_calibration.h"

// #include "pipeline/PipelineRequirements.hpp"
#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"
#include "pipeline/node/Measurements.hpp"

#include "pipeline/Pool.hpp"
#include "utilities/FsyncController.hpp"
#include "camera/LuxonisSidechannel.hpp"
#include "luxonis_sidechannel_data.h"

namespace {
template <typename R, typename... Args>
auto wrapRpcFunction(std::function<R(Args...)> fn) {
    return [fn = std::move(fn)](Args... args) {
        try {
            if constexpr(std::is_same_v<R, void>) {
                fn(std::forward<Args>(args)...);
                return std::tuple<bool, std::string>{true, {}};
            } else {
                return std::tuple<bool, std::string, R>{true, {}, fn(std::forward<Args>(args)...)};
            }
        } catch(const std::exception& e) {
            if constexpr(std::is_same_v<R, void>) {
                return std::tuple<bool, std::string>{false, e.what()};
            } else {
                return std::tuple<bool, std::string, R>{false, e.what(), R{}};
            }
        }
    };
}

template <typename Fn>
auto wrapRpc(Fn&& fn) {
    using FnT = std::decay_t<Fn>;
    using Func = typename nanorpc::core::detail::memfun_type<decltype(&FnT::operator())>::type;
    return wrapRpcFunction(Func{std::forward<Fn>(fn)});
}

template <typename R, typename... Args>
auto wrapRpc(std::function<R(Args...)> fn) {
    return wrapRpcFunction(std::move(fn));
}

template <typename R, typename... Args>
auto wrapRpc(R (*fn)(Args...)) {
    return wrapRpcFunction(std::function<R(Args...)>(fn));
}

template <typename Fn>
void rpcHandle(XLinkRpc& rpc, const std::string& name, Fn&& fn) {
    rpc.getServer().handle(name, wrapRpc(std::forward<Fn>(fn)));
}

nlohmann::json rvc4DeviceUsageProperties(const dai::SystemInformationRVC4& info) {
    nlohmann::json cpuUsages = nlohmann::json::array();
    for(const auto& cpuUsage : info.cpuUsages) {
        cpuUsages.push_back({
            {"average", cpuUsage.average},
            {"ms_time", cpuUsage.msTime},
        });
    }

    return {
        {"ddr_memory_usage",
         {
             {"remaining", info.ddrMemoryUsage.remaining},
             {"used", info.ddrMemoryUsage.used},
             {"total", info.ddrMemoryUsage.total},
         }},
        {"process_memory_usage", info.processMemoryUsage},
        {"cpu_avg_usage",
         {
             {"average", info.cpuAvgUsage.average},
             {"ms_time", info.cpuAvgUsage.msTime},
         }},
        {"process_cpu_avg_usage",
         {
             {"average", info.processCpuAvgUsage.average},
             {"ms_time", info.processCpuAvgUsage.msTime},
         }},
        {"cpu_usages", std::move(cpuUsages)},
        {"chip_temperature",
         {
             {"cpuss", info.chipTemperature.cpuss},
             {"gpuss", info.chipTemperature.gpuss},
             {"mdmss", info.chipTemperature.mdmss},
             {"video", info.chipTemperature.video},
             {"ddr", info.chipTemperature.ddr},
             {"camera", info.chipTemperature.camera},
             {"average", info.chipTemperature.average},
         }},
    };
}
}  // namespace

static std::atomic<bool> isFlashed{false};
bool isFlashBootedMode() {
    return isFlashed;
}

XLinkGlobalHandler_t g_xlink_glob_handler = {
    .profEnable = 0,
    .profilingData = {0.0f, 0.0f, 0ul, 0ul, 0ul, 0.0f},
    .loglevel = 0,
    .protocol = X_LINK_TCP_IP,
};

dai::PipelineSchema pipelineSchema;
dai::Assets assets;
dai::EepromData eepromData;
// eepromData is written by the RPC handlers and by the on-device DynamicCalibration node (flash), each on its own thread.
std::mutex eepromDataMtx;
dai::gate::PipelineBuilder& pipelineBuilder{dai::gate::PipelineBuilder::getInstance()};
// std::shared_ptr<dai::Pipeline> pipeline;
dai::Pipeline pipeline = dai::Pipeline(false);
dai::DeviceProperties deviceProperties;

dai::EepromData getCalibration() {
    if(auto calibData = pipeline.getEepromData()) {
        return *calibData;
    }
    std::lock_guard<std::mutex> lock(eepromDataMtx);
    return eepromData;
}

std::unique_ptr<XLinkRpc> mainRpc;
std::unique_ptr<Logging> logging;
std::shared_ptr<std::vector<uint8_t>> assetStorageVector;
std::unique_ptr<Timesync> timesync;

// Support functions (Common between RPC and flashed operation)
std::function<void(std::vector<std::uint8_t>)> parsePipeline = [](std::vector<std::uint8_t> binary) {
    // deserialize to pipeline schema
    dai::utility::deserialize(binary, pipelineSchema);
};

// Support functions (Common between RPC and flashed operation)
std::function<void(dai::PipelineSchema)> setPipelineSchema = [](dai::PipelineSchema s) {
    // set schema
    pipelineSchema = s;
};

std::function<void(std::vector<dai::CameraFeatures>, std::string imu)> overrideCameraFeatures = [](std::vector<dai::CameraFeatures> features, std::string imu) {
    dai::gate::CameraDiscovery::getInstance().overrideConnectedCameraFeatures(std::move(features));
    if(!imu.empty()) {
        dai::gate::IMUDiscovery::getInstance().overrideConnectedIMU(imu);
        dai::gate::IMU::overrideConnectedIMU(imu);
    }
};

std::function<void()> onInit = []() {
    static const dai::Version minimumOsVersion(1, 20, 5);
    static constexpr auto OAK4_OS_UPDATE_URL = "https://docs.luxonis.com/oak4-os-update";

    auto logger = pipelineBuilder.getSystemLogger();
    try {
        const auto osVersion = dai::gate::getOSVersion();
        logger->info("Detected OAK4 OS version: {}", osVersion.toString());
        if(osVersion < minimumOsVersion) {
            logger->warn("Detected OAK4 OS version {}, which is not officialy supported anymore, the oldest supported version is {}.\n",
                         osVersion.toString(),
                         minimumOsVersion.toString());
            logger->warn("Please update the OS to the latest version available at {} to ensure best performance and stability.", OAK4_OS_UPDATE_URL);
        }
    } catch(const std::exception& ex) {
        logger->warn("Failed to determine OAK4 OS version during onInit: {}", ex.what());
    }

    if(auto profile = dai::gate::getPowerProfile()) {
        logger->info("Power profile: {}", *profile);
    }
};

// Support functions (Common between RPC and flashed operation)
std::function<bool(dai::CameraBoardSocket)> isEepromAvailable = [](dai::CameraBoardSocket skt) {
    return eeprom_found(skt);
};

void patchIMUCalibration(dai::EepromData& board_data_config) {
    const auto& rotationMatrix = board_data_config.imuExtrinsics.rotationMatrix;
    if(dai::is3x3Matrix(rotationMatrix) && !dai::isIdentityRotationMatrix(rotationMatrix) && !dai::isZeroRotationMatrix(rotationMatrix)) {
        pipelineBuilder.getSystemLogger()->debug("Preserving non-identity IMU extrinsics from EEPROM/runtime calibration");
        return;
    }

    const dai::BoardConfig boardConfig = pipeline.getBoardConfig();
    if(boardConfig.defaultImuExtr.empty()) {
        if(!dai::is3x3Matrix(rotationMatrix) || dai::isZeroRotationMatrix(rotationMatrix)) {
            board_data_config.imuExtrinsics.rotationMatrix = dai::kIdentityRotationMatrix;
        }
        pipelineBuilder.getSystemLogger()->trace("Skipping IMU board-config patch because runtime BoardConfig has no IMU extrinsics defaults");
        return;
    }

    const auto imuDescription = dai::gate::IMUDiscovery::getInstance().getIMUDescription();
    auto it = boardConfig.defaultImuExtr.find(imuDescription.imuType);
    const bool hasBoardConfigExtrinsics = it != boardConfig.defaultImuExtr.end();
    const auto fwExtrinsics = dai::getFwImuExtrinsicsOverride(dai::gate::DeviceModel::get(), imuDescription.imuType);
    const bool hasFwSpecTranslation = fwExtrinsics && !dai::isZeroTranslation(fwExtrinsics->specTranslation);

    if(!hasBoardConfigExtrinsics && !hasFwSpecTranslation) {
        board_data_config.imuExtrinsics.rotationMatrix = dai::kIdentityRotationMatrix;
        pipelineBuilder.getSystemLogger()->trace("No default IMU extrinsics found for IMU type: {}", imuDescription.imuType);
        return;
    }

    dai::Extrinsics imuExtrinsics = hasBoardConfigExtrinsics ? it->second : dai::Extrinsics{};
    if(hasFwSpecTranslation) {
        if(!hasBoardConfigExtrinsics) {
            pipelineBuilder.getSystemLogger()->debug(
                "No board-config IMU extrinsics found for IMU type: {}. Using local FW IMU specTranslation override only.",
                imuDescription.imuType);
        } else {
            pipelineBuilder.getSystemLogger()->debug(
                "Using local FW IMU specTranslation override for device model and IMU type: {}", imuDescription.imuType);
        }
        imuExtrinsics.specTranslation = fwExtrinsics->specTranslation;
        imuExtrinsics.translation = fwExtrinsics->specTranslation;
        imuExtrinsics.toCameraSocket = fwExtrinsics->toCameraSocket;
    }

    pipelineBuilder.getSystemLogger()->debug("Patching IMU extrinsics for IMU type: {}", imuDescription.imuType);
    board_data_config.imuExtrinsics.specTranslation = imuExtrinsics.specTranslation;
    board_data_config.imuExtrinsics.translation = imuExtrinsics.translation;
    if(!dai::is3x3Matrix(imuExtrinsics.rotationMatrix) || dai::isZeroRotationMatrix(imuExtrinsics.rotationMatrix)) {
        board_data_config.imuExtrinsics.rotationMatrix = dai::kIdentityRotationMatrix;
    } else {
        board_data_config.imuExtrinsics.rotationMatrix = imuExtrinsics.rotationMatrix;
    }
    board_data_config.imuExtrinsics.toCameraSocket = imuExtrinsics.toCameraSocket;
}

void mergeCBACalibrations(dai::EepromData& runtimeCalibration) {
    try {
        for(const auto& camera : dai::gate::CameraDiscovery::getInstance().getConnectedCameraFeatures()) {
            const auto cameraSocket = camera.socket;
            if(cameraSocket == dai::CameraBoardSocket::AUTO || cameraSocket == dai::CameraBoardSocket::CBA) {
                continue;
            }

            if(runtimeCalibration.cameraData.find(cameraSocket) != runtimeCalibration.cameraData.end()) {
                continue;
            }
            if(!eeprom_found(cameraSocket)) {
                continue;
            }

            dai::EepromData cbaData;
            const auto status = eeprom_load_config(cbaData, EEPROM_AREA_USER, cameraSocket);
            if(status != EEPROM_OK || cbaData.cameraData.size() != 1) {
                continue;
            }

            const auto cameraDataIt = cbaData.cameraData.begin();
            runtimeCalibration.cameraData.emplace(cameraSocket, cameraDataIt->second);
        }
    } catch(const std::exception& ex) {
        pipelineBuilder.getSystemLogger()->debug("Skipping CBA calibration merge: {}", ex.what());
    }
}

std::function<std::tuple<bool, std::string>(std::optional<dai::EepromData>)> setCalibration =
    [](std::optional<dai::EepromData> newEepromData) -> std::tuple<bool, std::string> {
    try {
        dai::EepromData runtimeCalibration;
        if(newEepromData) {
            runtimeCalibration = *newEepromData;
        } else {
            std::lock_guard<std::mutex> lock(eepromDataMtx);
            runtimeCalibration = eepromData;
        }
        if(!newEepromData.has_value()) {
            mergeCBACalibrations(runtimeCalibration);
        }

        patchIMUCalibration(runtimeCalibration);
        // A local calibration whose origin no longer matches the installed multi-device graph would
        // leave source nodes unable to rebase, so reject it instead of silently publishing local poses.
        dai::gate::MultiDeviceCalibrationStore::getInstance().validateLocalCalibration(dai::CalibrationHandler(runtimeCalibration),
                                                                                    dai::gate::getSerialNumber());
        pipeline.setEepromData(runtimeCalibration);
    } catch(const std::exception& ex) {
        return {false, ex.what()};
    }
    return {true, ""};
};

// Runtime replacement of the cross-device calibration graph (mirrors setCalibration for local calibration).
std::function<std::tuple<bool, std::string>(std::optional<std::vector<dai::MultiDeviceExtrinsics>>)> setMultiDeviceCalibration =
    [](std::optional<std::vector<dai::MultiDeviceExtrinsics>> graph) -> std::tuple<bool, std::string> {
    try {
        std::optional<dai::CalibrationHandler> localCalibration;
        if(pipeline.isCalibrationDataAvailable()) {
            localCalibration = pipeline.getCalibrationData();
        }
        dai::gate::MultiDeviceCalibrationStore::getInstance().set(graph, localCalibration, dai::gate::getSerialNumber());
        // Keep the device pipeline's global properties in sync with what the nodes now use.
        auto globalProperties = pipeline.getGlobalProperties();
        globalProperties.multiDeviceCalibration = graph;
        pipeline.setGlobalProperties(globalProperties);
        pipelineBuilder.getSystemLogger()->info("Multi-device calibration {}", graph ? "updated (" + std::to_string(graph->size()) + " edges)" : "removed");
    } catch(const std::exception& ex) {
        return {false, ex.what()};
    }
    return {true, ""};
};

// Support functions (Common between RPC and flashed operation)
std::function<std::tuple<bool, std::string>(dai::EepromData, bool, bool, dai::CameraBoardSocket)> storeToEeprom =
    [](dai::EepromData localEepromData, bool factoryPermissions, bool protectedPermissions, dai::CameraBoardSocket skt) -> std::tuple<bool, std::string> {
    pipelineBuilder.getSystemLogger()->info("Store to EEPROM");
    eeprom_status res = eeprom_store_config(localEepromData, EEPROM_AREA_USER, factoryPermissions, protectedPermissions, skt);
    if(res == EEPROM_NO_PROTECTED_PERMISSIONS) {
        return {false, "No PROTECTED permissions to override protected EEPROM fields"};
    }

    if(res == 0) {
        if(skt == dai::CameraBoardSocket::AUTO) {
            std::lock_guard<std::mutex> lock(eepromDataMtx);
            eepromData = localEepromData;
        }
        bool success;
        std::string errorMsg;
        std::tie(success, errorMsg) = setCalibration(std::nullopt);
        if(!success) return {false, errorMsg};
        pipelineBuilder.getSystemLogger()->info("Setting EEPROM data succesfull");
        return {true, ""};
    } else {
        pipelineBuilder.getSystemLogger()->warn("Setting EEPROM data unsuccesfull, return code: {}", eeprom_status_to_str(res));
        return {false, "Error flashing calibration to USER area, error: " + std::string(eeprom_status_to_str(res))};
    }
};

std::function<std::tuple<bool, std::string>(dai::EepromData, bool, bool, dai::CameraBoardSocket)> storeToEepromFactory =
    [](dai::EepromData localEepromData, bool factoryPermissions, bool protectedPermissions, dai::CameraBoardSocket skt) -> std::tuple<bool, std::string> {
    pipelineBuilder.getSystemLogger()->info("Storing factory eeprom data");

    eeprom_status res = eeprom_store_config(localEepromData, EEPROM_AREA_FACTORY, factoryPermissions, protectedPermissions, skt);
    if(res == EEPROM_OK) {
        return {true, ""};
    } else if(res == EEPROM_NO_PROTECTED_PERMISSIONS) {
        return {false, "No PROTECTED permissions to override protected EEPROM fields"};
    } else if(res == EEPROM_NO_FACTORY_PERMISSIONS) {
        return {false, "No FACTORY permissions to flash FACTORY area"};
    } else {
        return {false, "Error flashing calibration to FACTORY area, error: " + std::string(eeprom_status_to_str(res))};
    }
};

std::function<std::tuple<bool, std::string, dai::EepromData>(dai::CameraBoardSocket)> readFromEeprom =
    [](dai::CameraBoardSocket skt) -> std::tuple<bool, std::string, dai::EepromData> {
    pipelineBuilder.getSystemLogger()->info("Read from EEPROM");
    dai::EepromData board_data_config;
    eeprom_status status = eeprom_load_config(board_data_config, EEPROM_AREA_USER, skt);
    if(status != EEPROM_OK) {
        pipelineBuilder.getSystemLogger()->info("No or invalid EEPROM configuration flashed.");
        return {false, "No or invalid EEPROM configuration flashed, error: " + std::string(eeprom_status_to_str(status)), {}};
    }
    patchIMUCalibration(board_data_config);
    return {true, "", board_data_config};
};

std::function<std::tuple<bool, std::string, dai::EepromData>(dai::CameraBoardSocket)> readFromEepromFactory =
    [](dai::CameraBoardSocket skt) -> std::tuple<bool, std::string, dai::EepromData> {
    pipelineBuilder.getSystemLogger()->info("Reading from Factory EEPROM contents");
    dai::EepromData board_data_config;
    eeprom_status status = eeprom_load_config(board_data_config, EEPROM_AREA_FACTORY, skt);
    if(status != EEPROM_OK) {
        return {false, "No or invalid EEPROM configuration flashed, error: " + std::string(eeprom_status_to_str(status)), {}};
    }
    patchIMUCalibration(board_data_config);
    return {true, "", board_data_config};
};

std::function<std::tuple<bool, std::string>(dai::CameraBoardSocket)> eepromFactoryReset =
    [](dai::CameraBoardSocket skt) -> std::tuple<bool, std::string> {
    return eeprom_factory_reset(skt);
};

std::function<std::tuple<bool, std::string>(bool, bool, dai::CameraBoardSocket)> eepromClear =
    [](bool protectedPermissions, bool factoryPermissions, dai::CameraBoardSocket skt) -> std::tuple<bool, std::string> {
    return eeprom_clear(EEPROM_AREA_USER, protectedPermissions, factoryPermissions, skt);
};

std::function<std::tuple<bool, std::string>(bool, bool, dai::CameraBoardSocket)> eepromFactoryClear =
    [](bool protectedPermissions, bool factoryPermissions, dai::CameraBoardSocket skt) -> std::tuple<bool, std::string> {
    return eeprom_clear(EEPROM_AREA_FACTORY, protectedPermissions, factoryPermissions, skt);
};

std::function<std::tuple<bool, std::string, std::vector<uint8_t>>(dai::CameraBoardSocket)> readFromEepromFactoryRaw =
    [](dai::CameraBoardSocket skt) {
        printf("Reading raw FACTORY EEPROM contents...\n");
        return eeprom_read_raw(EEPROM_AREA_FACTORY, skt);
    };

std::function<std::tuple<bool, std::string, std::vector<uint8_t>>(dai::CameraBoardSocket)> readFromEepromRaw =
    [](dai::CameraBoardSocket skt) {
        printf("Reading raw USER EEPROM contents...\n");
        return eeprom_read_raw(EEPROM_AREA_USER, skt);
    };

std::function<std::tuple<bool, std::string, std::vector<uint8_t>>(dai::CameraBoardSocket, int, int)> readCcmEepromRaw =
    [](dai::CameraBoardSocket skt, int size, int offset) -> std::tuple<bool, std::string, std::vector<uint8_t>> {
    if(size < 0 || offset < 0) {
        return {false, "CCM EEPROM size and offset must be non-negative", {}};
    }
    pipelineBuilder.getSystemLogger()->info("Reading {} bytes at offset {} from CCM EEPROM on {}", size, offset, dai::toString(skt));
    return ccm_eeprom_read_raw(skt, size, offset);
};

std::function<std::tuple<bool, std::string>(bool, bool, dai::CameraBoardSocket, std::vector<uint8_t>, int)> writeCcmEepromRaw =
    [](bool protectedPermissions, bool factoryPermissions, dai::CameraBoardSocket skt, std::vector<uint8_t> data, int offset) -> std::tuple<bool, std::string> {
    if(!protectedPermissions || !factoryPermissions) {
        return {false, "Writing CCM EEPROM requires FACTORY and PROTECTED permissions"};
    }
    if(offset < 0) {
        return {false, "CCM EEPROM offset must be non-negative"};
    }
    pipelineBuilder.getSystemLogger()->info("Writing {} bytes at offset {} to CCM EEPROM on {}", data.size(), offset, dai::toString(skt));
    return ccm_eeprom_write_raw(skt, std::move(data), offset);
};

std::function<void(dai::Assets)> setAssets = [](dai::Assets a) {
    // set Assets
    pipelineBuilder.getSystemLogger()->info("Setting assets");
    assets = a;
};

std::function<void(std::vector<std::uint8_t>)> parseAssets = [](std::vector<std::uint8_t> binary) {
    // deserialize to assets
    printf("TODO - parsing assets\n");
    dai::utility::deserialize(binary, assets);

    // // parse on LeonRT as well
    // ipc.getClient().call("parseAssets", binary);
};

dai::Asset parseAssetView(const std::string assetName, dai::AssetView assetView) {
    dai::Asset generatedAsset{assetName};
    std::vector<uint8_t> dataVector(assetView.data, assetView.data + assetView.size);
    generatedAsset.data = std::move(dataVector);
    return generatedAsset;
}

void parseAssetsToAssetManager(dai::Pipeline& pipeline, dai::Assets& assets) {
    auto& pipelineAssetManager = pipeline.getAssetManager();
    auto assetsMutable = assets.getAll();
    // std::cout << "Print all nodes in the pipeline!\n";

    // Traverse through the assets
    for(auto& assetView : assetsMutable) {
        auto assetName = assetView.first;
        // std::cout << assetName << std::endl;
        std::string parent;

        assetName = assetName.substr(1);  // Remove the first slash
        int identifierPosition = assetName.find('/');
        std::string identifier = assetName.substr(0, identifierPosition);
        // std::cout << "Identifier: " << identifier << std::endl;

        if(identifier.find("pipeline") != std::string::npos) {
            // std::cout << "Identified as a pipeline!\n";
            std::string assetNameLocal = assetName.substr(identifierPosition + 1);
            // std::cout << "Asset name: " << assetNameLocal << std::endl;
            auto builtAsset = parseAssetView(assetNameLocal, assetView.second);
            pipelineAssetManager.set(builtAsset);

        } else if(identifier.find("node") != std::string::npos) {
            // std::cout << "Identified as a node!\n";
            std::string charNodeId = assetName.substr(identifierPosition + 1);
            dai::Node::Id nodeID = std::stoi(charNodeId);

            // Find the next delimiter
            int delimiterAfterNodeID = assetName.find('/', identifierPosition + 1);
            std::string assetNameLocal = assetName.substr(delimiterAfterNodeID + 1);
            // std::cout << "Asset name: " << assetNameLocal << std::endl;
            auto node = pipeline.getNode(nodeID);
            auto neuralNetwork = std::dynamic_pointer_cast<dai::gate::NeuralNetwork>(node);
            if(neuralNetwork != nullptr && assetNameLocal == "__model") {
                neuralNetwork->setModelDataView(assetView.second.data,
                                                assetView.second.size,
                                                assetStorageVector);
            } else {
                auto& assetManager = node->getAssetManager();
                auto builtAsset = parseAssetView(assetNameLocal, assetView.second);
                assetManager.set(builtAsset);
            }
        } else {
            std::cout << "Invalid identifier!\n";
        }
    }
}

std::function<std::tuple<bool, std::string>()> waitForDeviceReady = []() -> std::tuple<bool, std::string>  {
    try {
        if (dai::gate::FsyncController::getInstance().supportsExternalFrameSyncMode() &&
            dai::gate::FsyncController::getInstance().isOk())
        {
            auto ret = dai::gate::FsyncController::getInstance().getExternalFrameSyncRole();

            if (ret == dai::ExternalFrameSyncRole::SLAVE) {
                for (auto start = std::chrono::system_clock::now();
                    std::chrono::system_clock::now() - start < std::chrono::seconds(20);)
                {
                    if(dai::gate::FsyncController::getInstance().isExternalFrameSyncSignalPresent()) {
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                if(!dai::gate::FsyncController::getInstance().isExternalFrameSyncSignalPresent()) {
                    return {false, "Slave was unable to detect or lock onto external frame sync signal."};
                }
            }
        }
    } catch (std::exception& ex) {
        return {false, ex.what()};
    }
    return {true, ""};
};

std::function<std::tuple<bool, std::string>(dai::ExternalFrameSyncRole)> setExternalFrameSyncRole =
[](dai::ExternalFrameSyncRole role) -> std::tuple<bool, std::string> {
    try {
        dai::gate::FsyncController::getInstance().setExternalFrameSyncRole(role, true);
    } catch (const std::exception& ex) {
        return {false, ex.what()};
    }
    return {true, ""};
};

std::function<dai::ExternalFrameSyncRole()> getExternalFrameSyncRole =
[]() -> dai::ExternalFrameSyncRole {
    return dai::gate::FsyncController::getInstance().getExternalFrameSyncRole();
};

std::function<std::tuple<bool, std::string>(float, float)> setExternalStrobeRelativeLimits =
[](float minDutyCycle, float maxDutyCycle) -> std::tuple<bool, std::string> {
    try {
        dai::gate::FsyncController::getInstance().setExternalStrobeRelativeLimits(minDutyCycle, maxDutyCycle);
    } catch (const std::exception& ex) {
        return {false, ex.what()};
    }
    return {true, ""};
};

std::function<void(bool)> setExternalStrobeEnable = [](bool enable) {
    return dai::gate::FsyncController::getInstance().setExternalStrobeEnable(enable);
};

std::function<void(dai::CameraBoardSocket)> setExternalStrobeEnableExposureMaster = [](dai::CameraBoardSocket socket) {
    dai::gate::FsyncController::getInstance().setExternalStrobeEnable(true);
    return dai::gate::FsyncController::getInstance().setExternalStrobeExposureMasterCamera(socket);
};

std::function<std::vector<std::tuple<std::string, int, int>>()> getIrDrivers =
[]() -> std::vector<std::tuple<std::string, int, int>> {
    std::vector<std::tuple<std::string, int, int>> drivers;

    bool success = false;
    std::string errorMsg;
    dai::EepromData eepromData;

    std::tie(success, errorMsg, eepromData) = readFromEeprom(dai::CameraBoardSocket::AUTO);
    if(!success) {
        DAI_ERROR("Couldn't read defaults from EEPROM, error message: {}", errorMsg);
        return drivers;
    }

    if(eepromData.boardConf.rfind("IR", 0) == 0) {
        drivers.emplace_back("stm-dot", 0, 0);
        drivers.emplace_back("stm-flood", 0, 0);
    }

    return drivers;
};

std::function<std::tuple<bool, std::string>()> buildPipeline = []() -> std::tuple<bool, std::string> {
    using namespace std::chrono_literals;
    using namespace std::chrono;
    auto deviceState = dai::gate::getDeviceState();
    if(deviceState == dai::gate::DeviceState::SETUP || deviceState == dai::gate::DeviceState::ERROR) {
        return {false,
                "The OAK4 has not been setup yet. Please complete the setup by following the instructions at https://setup.luxonis.com/ before using DepthAI."};
    }

    try {
        auto t1 = steady_clock::now();
        bool success = true;
        pipelineBuilder.build(pipelineSchema, pipeline);
        parseAssetsToAssetManager(pipeline, assets);
        spdlog::info("Time taken to build the pipeline: {}ms", duration_cast<milliseconds>(steady_clock::now() - t1).count());
    } catch(const std::exception& ex) {
        return {false, ex.what()};
    }

    return {true, ""};
};

// WARNING! This function should only be used in tests for collecting crash dumps and so,
// and for nothing else. It intentionally dereferences a null pointer to crash the device.
std::function<void(void)> crashDevice = []() {
    int* p = nullptr;
    std::cout << "My nullpointer point to" << *p << std::endl;
};

std::function<void(void)> startPipeline = []() {
    // // Disable initial watchdog delay
    // watchdog_cancel_initial_delay();
    pipelineBuilder.start(pipeline);
};

std::function<void(std::uint32_t, std::uint32_t)> setAssetStorage = [](std::uint32_t storageAddress, std::uint32_t size) {
    printf("TODO - implement set asset storage\n");
    // auto pStorage = (std::uint8_t*) storageAddress;
    // assets.setStorage(pStorage);
};

std::function<void(void)> watchdogResetTimer = []() {
    // TODO(themarpe)
    // watchdog_keepalive();
};

std::chrono::milliseconds actualWatchdogTimeout{1000};

std::function<void(void)> watchdogKeepalive = []() {
    using namespace std::chrono;
    using namespace std::chrono_literals;

    watchdogResetTimer();
};

/*

// TODO(themarpe) - use IPC Ping to detect if LeonRT crashed (timeout/etc..) and
call shutdown
// Shutdown and reset. Graceful specifies whether it was intentional or not
(error or not) auto shutdownReset = [](bool graceful, bool stayInBootloader){

    printf("About to shutdown and reset the device. Is this a graceful shutdown:
%d, xlink_allow_soft_reset: %d\n", (int) graceful, xlink_allow_soft_reset);

    if(stayInBootloader){
        // Else sets stay in bootloader flag
        *bootFlags = (*bootFlags) | STAY_IN_BOOTLOADER;
    } else {
        // disable STAY_IN_BOOTLOADER flag
        *bootFlags = (*bootFlags) &~ STAY_IN_BOOTLOADER;
    }

    // Do easy to do things first
    if(graceful){

        // Set previous BL CMX config
        // Modify with any additional found DHCPd settings
        {
            auto& op1 = parsedBootloaderConfig.options1.config;
            // Apply IP options
            uint32_t ipv4Gateway = 0;
            std::tie(op1.ipv4, op1.ipv4Mask, ipv4Gateway) =
network::getIpConfiguration();
            // If gateway was not set by DHCP (either static or prevCmxConfig)
            // No re0.lease will be available to parse it from, just use
prevCmxConfig if(ipv4Gateway) parsedBootloaderConfig.options2.config.ipv4Gateway
= ipv4Gateway;
            // Set config direction flag
            parsedBootloaderConfig.options2.config.flags |=
BootloaderConfig::Options2::DIRECTION_CONFIG_APP_TO_BL;
            // Finalize Config structure
            BootloaderConfig::finalize(parsedBootloaderConfig);

            printf("Finalized following bootloader config:\n");
            printf("ipv4: %08X mask: %08X, gateway: %08X\n", op1.ipv4,
op1.ipv4Mask, parsedBootloaderConfig.options2.config.ipv4Gateway);

            // Apply BL config
            cmxBootloaderConfig = parsedBootloaderConfig;
        }

        // Shutdown XLink connection
        XLinkDisconnect(0);

        // Checks if usb inited, and stop the device (lets OS know it was
removed, allows for cleaner consecutive reconnects))
        if(UsbPumpDataPumpRtemsApi_GetDevice() != nullptr){
            printf("Disconnecting USB\n");
            UsbPumpDataPumpRtemsApi_DeviceStop();
        } else {
            printf("USB not preset (inited), no need to disconnect\n");
        }

        // TODO(themarpe) - shutdown other things like volatile storage, etc..
(SDCard, ...) } else {
        // Reset as soon as possible
        // TODO(themarpe) - shutdown other things like volatile storage, etc..
(SDCard, ...)
    }


    printf("Graceful shutdown finished...\n");

    // if (xlink_allow_soft_reset) {
    //     // Reset afterwards
    //     SET_REG_WORD(CPR_MAS_RESET_ADR, 0);
    // } else {
    //     printf("Ignoring soft reset request. Halting!\n");
    //     asm("ta 1");
    // }
    // while(1);

};

*/

std::atomic<bool> isRunning{true};

static std::string appFullPath = "";
static std::string appParentFolderFullPath = "";

// RESET / SHUTDOWN mechanism if host requests it
void onLinkDownCallback(linkId_t linkId) {
    printf("onLinkDownCallback!\n");

    // Host is gone, disable crash dumps to prevent large core files from filling up RAM tmpfs
    dai::gate::enableCrashDump(false);

    dai::gate::FsyncController::getInstance().cleanup();

    isRunning = false;
    dai::gate::FsyncController::getInstance().stop();
    XLinkResetRemoteTimeout(0, 1000);
}

// Custom log handler
static void custom_log_handler(const gchar* log_domain, GLogLevelFlags log_level, const gchar* message, gpointer user_data) {
    // Handle only critical messages
    if(log_level & G_LOG_LEVEL_CRITICAL) {
        std::cerr << "GStreamer-CRITICAL: " << message << std::endl;
        // Terminate the application
        // std::exit(EXIT_FAILURE);
        if(pipeline.isRunning()) {
            pipeline.stop();
        }
    }
}

// Crash handler that guarantees a kernel core dump. A plain re-raise (what
// backward-cpp does) is not enough: the signal catcher of the camera HAL re-registers these
// signals at camera open reusing one sigaction whose sa_mask blocks the signal
// itself inside its own handler, so the re-raise would stay pending and the
// process would die without a core. sigaction() and pthread_sigmask() are
// async-signal-safe, so undo all of that here before delegating.
static struct sigaction previousCrashActions[NSIG];
static void crashHandler(int sig, siginfo_t* info, void* ctx) {
    // Make the signal lethal again: default disposition and not blocked.
    signal(sig, SIG_DFL);
    sigset_t unblock;
    sigemptyset(&unblock);
    sigaddset(&unblock, sig);
    pthread_sigmask(SIG_UNBLOCK, &unblock, nullptr);
    // Print the stack trace through backward's handler if it was installed.
    // Its trailing raise() now lands on SIG_DFL and dumps core.
    const struct sigaction& prev = previousCrashActions[sig];
    if((prev.sa_flags & SA_SIGINFO) && prev.sa_sigaction != nullptr) {
        prev.sa_sigaction(sig, info, ctx);
    }
    raise(sig);
    _exit(128 + sig);
}

static void installCrashHandler() {
    const int crashSignals[] = {SIGSEGV, SIGABRT, SIGBUS, SIGFPE, SIGILL, SIGSYS, SIGTRAP};
    struct sigaction act = {};
    act.sa_flags = SA_SIGINFO | SA_ONSTACK;
    act.sa_sigaction = &crashHandler;
    for(int sig : crashSignals) {
        sigfillset(&act.sa_mask);
        sigdelset(&act.sa_mask, sig);
        sigaction(sig, &act, &previousCrashActions[sig]);
    }
}

int main(int argc, char** args) {
    // Install backward-cpp crash handlers before anything else touches depthai-core:
    // initialize() is a run-once static and XLink bring-up calls the no-arg overload
    // with installSignalHandler=false, so a later opt-in would be silently ignored.
    // The signal catcher of the camera HAL then chains into these handlers instead of SIG_DFL(=NULL).
    dai::initialize("", /*installSignalHandler=*/true, nullptr);

    // A linked library drops PR_SET_DUMPABLE at startup which makes the kernel
    // silently skip core dumps for this process - re-assert it so crashes dump.
    prctl(PR_SET_DUMPABLE, 1);

    // Wrap backward's handlers so the crash path always ends in a core dump,
    // also when invoked through the signal catcher chain of the camera HAL (see crashHandler).
    installCrashHandler();

    // Watchdog
    std::atomic<bool> isWatchdogRunning{true};
    std::thread watchdogThread;
    // System log
    std::atomic<bool> isSystemLogRunning{true};
    std::thread systemLogThread;
    // Low memory check
    std::atomic<bool> isLowMemoryCheckRunning{true};
    std::thread lowMemoryCheckThread;
    using namespace std::chrono;
    std::atomic<float> loggingRateHz{1.0f};
    std::mutex loggingRateHzMtx;
    std::condition_variable loggingRateHzCv;
    std::mutex systemLogSleepMutex;
    std::condition_variable systemLogSleepCv;

    pipeline.setDefaultDevicePropertiesRef(&deviceProperties);

    if(argc >= 1) {
        appFullPath = std::string(args[0]);
        std::size_t lastSlashPos = appFullPath.find_last_of('/');

        if(lastSlashPos != std::string::npos) {
            appParentFolderFullPath = appFullPath.substr(0, lastSlashPos);
        } else {
            appParentFolderFullPath = appFullPath;
        }
        std::cout << "App full path: " << appFullPath << std::endl;
        std::cout << "App parent folder full path: " << appParentFolderFullPath << std::endl;

        const char* env_p;

        if(0) {
            env_p = std::getenv("DSP_LIBRARY_PATH");

            // Check if DSP_LIBRARY_PATH exists
            if(env_p) {
                // Prepend the new path and set the updated environment variable
                std::string newPath = std::string(env_p) + ";" + appParentFolderFullPath;
                setenv("DSP_LIBRARY_PATH", newPath.c_str(), 1);
                std::cout << "Updated DSP_LIBRARY_PATH: " << newPath << '\n';
            } else {
                // If it doesn't exist, set DSP_LIBRARY_PATH to the new path
                setenv("DSP_LIBRARY_PATH", appParentFolderFullPath.c_str(), 1);
                std::cout << "Set DSP_LIBRARY_PATH to: " << appParentFolderFullPath << '\n';
            }
        }

        env_p = std::getenv("ADSP_LIBRARY_PATH");

        // Check if ADSP_LIBRARY_PATH exists
        if(env_p) {
            // Prepend the new path and set the updated environment variable
            std::string newPath = std::string(env_p) + ";" + appParentFolderFullPath;
            setenv("ADSP_LIBRARY_PATH", newPath.c_str(), 1);
            std::cout << "Updated ADSP_LIBRARY_PATH: " << newPath << '\n';
        } else {
            // If it doesn't exist, set ADSP_LIBRARY_PATH to the new path
            setenv("ADSP_LIBRARY_PATH", appParentFolderFullPath.c_str(), 1);
            std::cout << "Set ADSP_LIBRARY_PATH to: " << appParentFolderFullPath << '\n';
        }
    }
    // Add the NNArchive UDO staging directory to ADSP_LIBRARY_PATH next to the
    // setenv above: before any pipeline is built, so no DSP session exists yet.
    dai::gate::UdoRuntime::get();

    printf("=== DepthAI Device RVC4 ===\n");

    {
        // // Set some env
        // setenv("G_DEBUG", "fatal_warnings", 1);

        // Init gst
        gst_init(0, NULL);

        // Set the custom log handler
        g_log_set_handler("GStreamer", G_LOG_LEVEL_CRITICAL, (GLogFunc)custom_log_handler, NULL);

        // Set logging level to GST_LEVEL_ERROR
        gst_debug_set_default_threshold(GST_LEVEL_ERROR);
    }

    // UNUSED(args);
    // const char *name = "main";
    // rtems_object_set_name(RTEMS_SELF, name);
    // pthread_setname_np(pthread_self(), name);

    // // Set custom assert reset callback
    // setCustomAssertResetCallback([](){
    //     shutdownReset(false, true);
    // });

    // TMP TMP
    bool networkBootloader = true;

    // Initialize watchdog
    using namespace std::chrono_literals;
    auto WATCHDOG_INITIAL_DELAY = 8000ms;
    auto WATCHDOG_TIMEOUT = dai::device::XLINK_USB_WATCHDOG_TIMEOUT;
    // Network mode and timeout isn't set
    if(networkBootloader) {
        // Network case
        WATCHDOG_INITIAL_DELAY = 15s;
        WATCHDOG_TIMEOUT = dai::device::XLINK_TCP_WATCHDOG_TIMEOUT;
    }

    // Set from ENV
    try {
        auto* env_p = std::getenv("DEPTHAI_DEVICE_WATCHDOG");
        if(env_p) {
            WATCHDOG_TIMEOUT = milliseconds(std::stoi(env_p));
        }
    } catch(...) {
        // TODO(themarpe) - display the issue
    }

    //     // // Initialize watchdog
    //     // actualWatchdogTimeout = watchdog_init(WATCHDOG_INITIAL_DELAY,
    //     // WATCHDOG_TIMEOUT);

    //     // //set default
    //     // mvLogDefaultLevelSet(MVLOG_WARN);

    // EEPROM
    eeprom_open();
    eeprom_version eeprom_version = eeprom_get_version(EEPROM_AREA_USER);
    if(eeprom_version_is_valid(eeprom_version)) {
        printf("eeprom configuration version: %08X\n", eeprom_version);
    } else {
        if(eeprom_found()) {
            printf("eeprom configuration missing\n");
        } else {
            printf("no eeprom available\n");
        }
    }
    eeprom_status eeprom_status = eeprom_load_config(eepromData, EEPROM_AREA_USER);
    printf("eeprom configuration load from user area, status: %d\n", eeprom_status);

    // If invalid or non existing config, set data version to invalid
    if(eeprom_status != EEPROM_OK) {
        eepromData.version = 0;
        printf("EEPROM data not set");
    } else {
        pipeline.setEepromData(eepromData);
    }

    std::string serialNumber = dai::gate::getSerialNumber();
    if(const char* calibrationOverride = dai_device_calibration_override(serialNumber.c_str())) {
        try {
            // printEepromData(eepromData);

            auto calibJson = nlohmann::json::parse(calibrationOverride);
            std::cout << calibJson << std::endl;
            dai::CalibrationHandler calibHandler = dai::CalibrationHandler::fromJson(calibJson);
            eepromData = calibHandler.getEepromData();
            std::cout << "Calibration data loaded from map for serial: " << serialNumber << std::endl;

        } catch(const std::exception& ex) {
            std::cerr << "Error while loading calibration data: " << ex.what() << std::endl;
        }
    }

    g_xlink_glob_handler.protocol = X_LINK_TCP_IP;

    pipelineBuilder.setNodeHandler(pipelineBuilder.getDefaultNodeHandler());

    if(!isFlashed) {
        int is_connected_via_usb = 0;

        // Initialize XLink (wait for host to connect)
        {
            // Add handler first
            XLinkAddLinkDownCb(onLinkDownCallback);

            printf("Initializing XLink...\n");
            auto status = XLinkInitialize(&g_xlink_glob_handler);
            if(X_LINK_SUCCESS != status) {
                return 0;
            }

            printf("Creating XLink server and waiting for connection...\n");

            const char* env = std::getenv("DEPTHAI_GATE_USB_CONNECTION");
            is_connected_via_usb = (env != nullptr) ? std::stoi(env) : 0;
            XLinkHandler_t handler;

            std::string path;
            if (is_connected_via_usb == 0) {
                printf("Starting XLink via TCP/IP...\n");
                path = "0.0.0.0";
                handler.devicePath = (char*)path.c_str();
                handler.protocol = X_LINK_TCP_IP_OR_LOCAL_SHDMEM;
            } else {
                printf("Starting XLink via USB EP...\n");
                path = "/dev/usb-ffs/device";
                handler.devicePath = (char*)path.c_str();
                handler.protocol = X_LINK_USB_EP;
            }

            status = XLinkServer(&handler, "xlinkserver", X_LINK_BOOTED, X_LINK_MYRIAD_X);
            if(X_LINK_SUCCESS != status) {
                return 0;
            }

            printf("Done!\n");
        }

        // Open watchdog stream first
        // Start watchdog thread, which checks both LeonOS and LeonRT are still
        // alive
        watchdogThread = std::thread([&isWatchdogRunning]() {
            streamId_t streamId = XLinkOpenStream(0, dai::device::XLINK_CHANNEL_WATCHDOG, 128);

            streamPacketDesc_t* packet = nullptr;
            int watchdogFd = -1;
            const char* watchdogFdName = std::getenv("DEPTHAI_GATE_WATCHDOG_FD");
            if(watchdogFdName) {
                try {
                    watchdogFd = std::stoi(watchdogFdName);
                } catch(const std::invalid_argument& ex) {
                    std::cout << "Cannot convert DEPTHAI_GATE_WATCHDOG_FD to a valid file descriptor.\n";
                }
            } else {
                std::cout << "DEPTHAI_GATE_WATCHDOG_FD not set.\n";
            }
            while(isWatchdogRunning) {
                auto status = XLinkReadData(streamId, &packet);
                if(status == X_LINK_SUCCESS) {
                    XLinkReleaseData(streamId);
                    if(watchdogFd != -1) {
                        std::string message{"ALIVE"};
                        ssize_t ret = write(watchdogFd, message.c_str(), message.size());  // Bump gate
                        if(ret <= 0) {
                            std::cout << "Bumping the watchdog failed. Couldn't write to file, error: " << ret << std::endl;
                        }
                    }
                } else {
                    using namespace std::chrono_literals;
                    std::this_thread::sleep_for(10ms);
                    break;
                }
            }
        });

        // Prepare logger
        logging = std::make_unique<Logging>();
        logging->start();

        // // Prepare the callback
        pipelineBuilder.setLogEventCallback([](dai::LogMessage log) { logging->addLog(std::move(log)); });
        pipelineBuilder.init();

        if(is_connected_via_usb) {
            auto actualUsbSpeed = dai::gate::getUsbSpeed();
            // TODO only warn if the host didn't set the speed explicitly low (BoardConfig?)
            if(actualUsbSpeed < dai::UsbSpeed::SUPER) {
                std::ostringstream speedStream;
                speedStream << actualUsbSpeed;
                pipelineBuilder.getSystemLogger()->warn("USB connection speed: {} / USB2. Performance may be degraded", speedStream.str());
            }
        }

        // Prepare timesync
        // TODO(themarpe) - expose implementation
        timesync = std::make_unique<Timesync>(dai::device::XLINK_CHANNEL_TIMESYNC, Timesync::DEFAULT);

        // auto actualUsbSpeed = dai::UsbSpeed::UNKNOWN;
        // // TODO - fix getUSBSpeed instead
        // if(!networkAvailable){
        //     // After logging is active, check usb speed set explicitly and actual
        //     actualUsbSpeed = getUSBSpeed();
        // }

        // Main rpc
        mainRpc = std::make_unique<XLinkRpc>(dai::device::XLINK_CHANNEL_MAIN_RPC, pipelineBuilder.getSystemLogger());
        dai::DeviceTelemetry::getInstance().start();

        rpcHandle(*mainRpc, "hello", []() {
            printf("hello called!\n");
            return "Hello World!";
        });

        rpcHandle(*mainRpc, "sum", [](std::vector<int> numbers) {
            printf("sum called!\n");
            int sum = 0;
            for(const auto& i : numbers) sum += i;
            return sum;
        });

        rpcHandle(*mainRpc, "isRunning", []() { return true; });
        rpcHandle(*mainRpc, "hasCrashDump", []() { return false; });

        rpcHandle(*mainRpc, "crashDevice", crashDevice);
        rpcHandle(*mainRpc, "getMxId", []() { return dai::gate::getSerialNumber(); });
        rpcHandle(*mainRpc, "getAnonymousTelemetryId", []() { return dai::getAnonymousTelemetryId(); });
        rpcHandle(*mainRpc, "getUsbSpeed", []() { return dai::gate::getUsbSpeed(); });
        rpcHandle(*mainRpc, "getEthernetLinkSpeed", [](std::string address) { return dai::gate::getEthernetLinkSpeed(address); });
        rpcHandle(*mainRpc, "getOSVersion", []() { return dai::gate::getOSVersionString(); });
        rpcHandle(*mainRpc, "onInit", onInit);

        // TODO(themarpe) - deprecate
        // rpcHandle(*mainRpc, "memAlloc", [](uint32_t size){
        //     return (std::uint32_t) lpb::Ddr::malloc(size);
        // });

        // TODO(themarpe) - deprecate
        rpcHandle(*mainRpc, "readFromXLink", [](std::string streamName, std::uint32_t mem, std::uint32_t size) {
            std::uint8_t* dest = (std::uint8_t*)mem;

            auto streamId = XLinkOpenStream(0, streamName.c_str(), 1);
            std::uint32_t offset = 0;
            do {
                std::uint32_t toRead = std::min((std::uint32_t)dai::device::XLINK_USB_BUFFER_MAX_SIZE, size - offset);
                streamPacketDesc_t* packet = nullptr;
                auto status = XLinkReadData(streamId, &packet);
                if(status == X_LINK_SUCCESS) {
                    // rpc handlers are not preemptive, if one is taking too much time
                    // the watchdog will expire, since the handler will not be executed
                    watchdogKeepalive();
                    memcpy(dest + offset, packet->data, packet->length);
                    XLinkReleaseData(streamId);
                    offset += toRead;
                } else {
                    using namespace std::chrono_literals;
                    std::this_thread::sleep_for(10ms);
                    continue;
                }
            } while(offset < size);
        });

        std::function<void(std::string, std::uint32_t)> readAssetStorageFromXLink = [](std::string streamName, std::uint32_t size) {
            assetStorageVector = std::make_shared<std::vector<uint8_t>>(std::vector<uint8_t>(size));
            std::uint8_t* dest = (std::uint8_t*)assetStorageVector->data();

            auto streamId = XLinkOpenStream(0, streamName.c_str(), 1);
            std::uint32_t offset = 0;
            do {
                std::uint32_t toRead = std::min((std::uint32_t)dai::device::XLINK_USB_BUFFER_MAX_SIZE, size - offset);
                streamPacketDesc_t* packet = nullptr;
                auto status = XLinkReadData(streamId, &packet);
                if(status == X_LINK_SUCCESS) {
                    // rpc handlers are not preemptive, if one is taking too much time
                    // the watchdog will expire, since the handler will not be executed
                    watchdogKeepalive();
                    memcpy(dest + offset, packet->data, packet->length);
                    XLinkReleaseData(streamId);
                    offset += toRead;
                } else {
                    using namespace std::chrono_literals;
                    std::this_thread::sleep_for(10ms);
                    continue;
                }
            } while(offset < size);
            assets.setStorage(dest);
        };

        rpcHandle(*mainRpc, "isPipelineRunning", []() { return pipeline.isRunning(); });

        rpcHandle(*mainRpc, "getConnectedCameras", []() { return dai::gate::CameraDiscovery::getInstance().getConnectedCameras(); });

        rpcHandle(*mainRpc, "getConnectedCameraFeatures", []() { return dai::gate::CameraDiscovery::getInstance().getConnectedCameraFeatures(); });

        rpcHandle(*mainRpc, "getCameraSensorNames", []() { return dai::gate::CameraDiscovery::getInstance().getConnectedCameraSensorNames(); });

        rpcHandle(*mainRpc, "setPipelineSchema", setPipelineSchema);
        rpcHandle(*mainRpc, "setAssets", setAssets);
        rpcHandle(*mainRpc, "setAssetStorage", setAssetStorage);
        mainRpc->getServer().handle("overrideCameraFeatures", overrideCameraFeatures);
        rpcHandle(*mainRpc, "waitForDeviceReady", waitForDeviceReady);
        rpcHandle(*mainRpc, "setExternalFrameSyncRole", setExternalFrameSyncRole);
        rpcHandle(*mainRpc, "getExternalFrameSyncRole", getExternalFrameSyncRole);
        rpcHandle(*mainRpc, "setExternalStrobeRelativeLimits", setExternalStrobeRelativeLimits);
        rpcHandle(*mainRpc, "setExternalStrobeEnable", setExternalStrobeEnable);
        rpcHandle(*mainRpc, "setExternalStrobeEnableExposureMaster", setExternalStrobeEnableExposureMaster);
        rpcHandle(*mainRpc, "buildPipeline", buildPipeline);
        rpcHandle(*mainRpc, "startPipeline", startPipeline);
        rpcHandle(*mainRpc, "storeToEeprom", storeToEeprom);
        rpcHandle(*mainRpc, "storeToEepromFactory", storeToEepromFactory);
        rpcHandle(*mainRpc, "readFromEeprom", readFromEeprom);
        rpcHandle(*mainRpc, "readFromEepromRaw", readFromEepromRaw);
        rpcHandle(*mainRpc, "readFromEepromFactory", readFromEepromFactory);
        rpcHandle(*mainRpc, "readFromEepromFactoryRaw", readFromEepromFactoryRaw);
        rpcHandle(*mainRpc, "eepromFactoryReset", eepromFactoryReset);
        rpcHandle(*mainRpc, "isEepromAvailable", isEepromAvailable);
        rpcHandle(*mainRpc, "eepromClear", eepromClear);
        rpcHandle(*mainRpc, "eepromFactoryClear", eepromFactoryClear);
        mainRpc->getServer().handle("readCcmEepromRaw", readCcmEepromRaw);
        mainRpc->getServer().handle("writeCcmEepromRaw", writeCcmEepromRaw);
        rpcHandle(*mainRpc, "readAssetStorageFromXLink", readAssetStorageFromXLink);

        rpcHandle(*mainRpc, "timestampEcho", [](std::uint64_t ts) { return ts; });
        rpcHandle(*mainRpc, "enableCrashDump", dai::gate::enableCrashDump);
        // after watchdog ends, this enables for quicker reconnects and link down
        // detection
        rpcHandle(*mainRpc, "reset", []() {
            // TODO(themarpe) - port

            // // Stay in bootloader for a little while in this case as well.
            // // Helps to not require jumping from FLASH_BOOTED to BL -> BOOTED if
            // timeout is set to a low value shutdownReset(true, true);
        });
        mainRpc->getServer().handle("setCalibration", setCalibration);
        mainRpc->getServer().handle("setMultiDeviceCalibration", setMultiDeviceCalibration);

        rpcHandle(*mainRpc, "getCalibration", []() -> std::tuple<bool, std::string, dai::EepromData> {
            auto calibData = pipeline.getEepromData();
            if(calibData) {
                return {true, "", *calibData};
            } else {
                return {false, "Calibration not set", {}};
            }
        });

        rpcHandle(*mainRpc, "isCalibrationAvailable", []() -> bool {
            return pipeline.getEepromData().has_value() && pipeline.getEepromData().value().version != 0;
        });
        rpcHandle(*mainRpc, "getStereoPairs", []() { return dai::gate::getStereoPairs(getCalibration()); });

        // Capability to select log level of nodes
        rpcHandle(*mainRpc, "setLogLevel", [](dai::LogLevel level) { pipelineBuilder.setLogLevelCallback(level); });
        rpcHandle(*mainRpc, "setNodeLogLevel", [](int64_t nodeId, dai::LogLevel level) { pipelineBuilder.setNodeLogLevelCallback(nodeId, level); });
        printf("Added setLogLevel to be handled\n");
        rpcHandle(*mainRpc, "getLogLevel", []() { return pipelineBuilder.getLogLevelCallback(); });
        rpcHandle(*mainRpc, "getNodeLogLevel", [](int64_t nodeId) { return pipelineBuilder.getNodeLogLevelCallback(nodeId); });
        mainRpc->getServer().handle("setProperties", [](dai::DeviceProperties properties) { if(!pipeline.isBuilt()) deviceProperties.setFrom(properties); });
        mainRpc->getServer().handle("getProperties", []() { return deviceProperties; });
        mainRpc->getServer().handle("setCameraTuningBlob", [](std::string uri, uint32_t size) {
            if(!pipeline.isBuilt()) {
                deviceProperties.cameraTuningBlobUri = uri;
                deviceProperties.cameraTuningBlobSize = size;
            }
        });
        mainRpc->getServer().handle("setCameraSocketTuningBlobs", [](std::unordered_map<dai::CameraBoardSocket, std::pair<std::string, uint32_t>> blobs) {
            if(!pipeline.isBuilt()) {
                for(const auto& [k, v] : blobs) {
                    deviceProperties.cameraSocketTuningBlobUri[k] = v.first;
                    deviceProperties.cameraSocketTuningBlobSize[k] = v.second;
                }
            }
        });
        rpcHandle(*mainRpc, "setTimesync", [](int period, int numSamples, bool random) {
            // TODO - port
            timesync->startSyncWait(std::chrono::milliseconds(period), numSamples, random);
        });
        rpcHandle(*mainRpc, "getState", []() {
            throw std::runtime_error("Getting current device state is not supported on RVC4");
        });
        rpcHandle(*mainRpc, "setXLinkChunkSize", [](int sizeBytes) {
            // TODO(themarpe) - port
            if(!pipeline.isBuilt()) deviceProperties.xlinkChunkSize = sizeBytes;
        });
        rpcHandle(*mainRpc, "getXLinkChunkSize", []() {
            // TODO(themarpe) - port
            return deviceProperties.xlinkChunkSize;
        });

        mainRpc->getServer().handle("setSippBufferSize", [](int sizeBytes) { if(!pipeline.isBuilt()) deviceProperties.sippBufferSize = sizeBytes; });

        mainRpc->getServer().handle("setSippDmaBufferSize", [](int sizeBytes) { if(!pipeline.isBuilt()) deviceProperties.sippDmaBufferSize = sizeBytes; });
        rpcHandle(*mainRpc, "getDdrUsage", []() { return Measurements(pipelineBuilder.getSystemLogger()).getMemoryInfo(); });
        rpcHandle(*mainRpc, "getCmxUsage", []() {
            // TODO(themarpe) - port
            // return Measurements::getInstance().getMemoryInfoCmx();
        });
        rpcHandle(*mainRpc, "getLeonCssHeapUsage", []() {
            // TODO(themarpe) - port
            // return Measurements::getInstance().getMemoryInfoLeonOs();
        });
        rpcHandle(*mainRpc, "getLeonMssHeapUsage", []() {
            // TODO(themarpe) - port
            // return Measurements::getInstance().getMemoryInfoLeonRt();
        });
        rpcHandle(*mainRpc, "getChipTemperature", []() {
            // TODO(themarpe) - port
            // return Measurements::getInstance().getChipTemperature();
        });
        rpcHandle(*mainRpc, "getLeonCssCpuUsage", []() {
            // TODO(themarpe) - port
            // return Measurements::getInstance().getCpuUsageLeonOs();
        });
        rpcHandle(*mainRpc, "getLeonMssCpuUsage", []() {
            // TODO(themarpe) - port
            // return Measurements::getInstance().getCpuUsageLeonRt();
        });
        rpcHandle(*mainRpc, "getProcessMemoryUsage", []() {
            const auto memory = dai::gate::getProcessMemoryUsage();
            if(!memory) pipelineBuilder.getSystemLogger()->error("Failed to read process memory usage");
            return memory.value_or(0);
        });
        rpcHandle(*mainRpc, "getSupportedDeviceModels", []() {
                  const std::array<dai::DeviceModelZoo, 11> modelOrder = {
                      dai::DeviceModelZoo::NEURAL_DEPTH_1248X780,
                      dai::DeviceModelZoo::NEURAL_DEPTH_1056X660,
                      dai::DeviceModelZoo::NEURAL_DEPTH_960X600,
                      dai::DeviceModelZoo::NEURAL_DEPTH_864X540,
                      dai::DeviceModelZoo::NEURAL_DEPTH_768X480,
                      dai::DeviceModelZoo::NEURAL_DEPTH_576X360,
                      dai::DeviceModelZoo::NEURAL_DEPTH_480X300,
                      dai::DeviceModelZoo::NEURAL_DEPTH_384X240,
                      dai::DeviceModelZoo::NEURAL_DEPTH_288X180,
                      dai::DeviceModelZoo::NEURAL_DEPTH_192X120,
                      dai::DeviceModelZoo::TOF_NEURAL_FUSION_672X804,
                    };
            std::vector<dai::DeviceModelZoo> supportedModels;
            supportedModels.reserve(modelOrder.size());
            for(const auto& model : modelOrder) {
                if(dai::gate::NeuralNetwork::isModelAvailable(model)) {
                    supportedModels.push_back(model);
                }
            }
            return supportedModels;
        });
        rpcHandle(*mainRpc, "isNeuralDepthSupported", []() {
            // Only check the models that were added originally in 1.20.5
            const std::array<dai::DeviceModelZoo, 4> requiredModels = {
                dai::DeviceModelZoo::NEURAL_DEPTH_768X480,
                dai::DeviceModelZoo::NEURAL_DEPTH_576X360,
                dai::DeviceModelZoo::NEURAL_DEPTH_480X300,
                dai::DeviceModelZoo::NEURAL_DEPTH_384X240,
            };

            return std::all_of(requiredModels.begin(), requiredModels.end(), [](const auto& model) {
                return dai::gate::NeuralNetwork::isModelAvailable(model);
            });
        });
        rpcHandle(*mainRpc, "hasGPU", []() { return dai::utils::hasGPU(); });
        // rpcHandle(*mainRpc, "startCamera", startCamera);

#if 0  // Note: these are no longer exposed in depthai-core !!!
        rpcHandle(*mainRpc, "irWriteReg", [](int reg, int value){
            ir_write(reg, value);
        });
        rpcHandle(*mainRpc, "irReadReg", [](int reg){
            return ir_read(reg);
        });
#endif
        rpcHandle(*mainRpc, "setIrLaserDotProjectorBrightness", [](float mAOrIntensity, int mask, bool useIntensity) {
            auto systemLogger = pipelineBuilder.getSystemLogger();
            float max = (useIntensity ? 1 : 1200);
            if(mAOrIntensity > max) {
                mAOrIntensity = max;
                systemLogger->warn("Capping IR dot projector brightness to {}{}", mAOrIntensity, (useIntensity ? "%" : " mA"));
            }
            if(!useIntensity) {
                systemLogger->error("RVC4 does not support setting IR dot projector brightness in mA");
                return false;
            }
            auto& led = dai::gate::FsyncController::getInstance().dotProjector;
            bool ok = led.setIntensity(mAOrIntensity);
            if(ok) {
                if(auto warning = led.checkBrightnessCeiling(mAOrIntensity)) systemLogger->warn("{}", *warning);
            }
            if(!ok) systemLogger->error("Error setting dot projector!");
            return ok;
        });

        rpcHandle(*mainRpc, "setIrFloodLightBrightness", [](float mAOrIntensity, int mask, bool useIntensity) {
            auto systemLogger = pipelineBuilder.getSystemLogger();
            float max = (useIntensity ? 1 : 1500);
            if(mAOrIntensity > max) {
                mAOrIntensity = max;
                systemLogger->warn("Capping IR flood light brightness to {}{}", mAOrIntensity, (useIntensity ? "%" : " mA"));
            }
            if(!useIntensity) {
                systemLogger->error("RVC4 does not support setting IR dot projector brightness in mA");
                return false;
            }
            auto& led = dai::gate::FsyncController::getInstance().floodLight;
            bool ok = led.setIntensity(mAOrIntensity);
            if(ok) {
                if(auto warning = led.checkBrightnessCeiling(mAOrIntensity)) systemLogger->warn("{}", *warning);
            }
            if(!ok) systemLogger->error("Error setting flood illuminator!");
            return ok;
        });

        rpcHandle(*mainRpc, "getIrDrivers", getIrDrivers);
        rpcHandle(*mainRpc, "getConnectedIMU", []() {
            auto imuDescription = dai::gate::IMUDiscovery::getInstance().getIMUDescription();
            std::string imuVersion = dai::gate::IMU::getIMUVersion();
            if(imuVersion == "NONE" && imuDescription.imuType != "NONE") {
                pipelineBuilder.getSystemLogger()->error("No IMU version detected, but specified in DTS. Update FIP.");
            }
            return imuVersion;
        });
        rpcHandle(*mainRpc, "getIMUFirmwareVersion", []() {
            std::string imuFirmwareVersion = dai::gate::IMU::getIMUFirmwareVersion();
            return imuFirmwareVersion;
        });
        rpcHandle(*mainRpc, "getEmbeddedIMUFirmwareVersion", []() {
            std::string latestAvailableImuFirmwareVersion = dai::gate::IMU::getEmbeddedIMUFirmwareVersion();
            return latestAvailableImuFirmwareVersion;
        });

        // Create a measurement logging thread
        rpcHandle(*mainRpc, "setSystemInformationLoggingRate", [&loggingRateHz, &loggingRateHzMtx, &loggingRateHzCv](float rateHz) {
            // Notifies logging thread about the rate change
            std::unique_lock<std::mutex> l(loggingRateHzMtx);
            loggingRateHz.store(rateHz);
            loggingRateHzCv.notify_all();
        });
        rpcHandle(*mainRpc, "getSystemInformationLoggingRate", [&loggingRateHz]() { return loggingRateHz.load(); });
        lowMemoryCheckThread = std::thread([&isLowMemoryCheckRunning]() {
            const auto systemLogger = pipelineBuilder.getSystemLogger();
            Measurements measurements(systemLogger);
            const auto LOW_DDR_MEMORY_CHECK_PERIOD = std::chrono::duration<float>(1.0f / 30.0f);
            constexpr auto LOW_DDR_MEMORY_WARNING_PERIOD = std::chrono::seconds(60);
            constexpr int64_t LOW_DDR_FREE_MEMORY_THRESHOLD_PERCENT = 5;
            auto lastLowMemoryWarning = std::chrono::steady_clock::now() - LOW_DDR_MEMORY_WARNING_PERIOD;

            while(isLowMemoryCheckRunning) {
                const auto ddr = measurements.getMemoryInfo();
                const auto now = std::chrono::steady_clock::now();
                constexpr float mB = 1.0f / 1024.f / 1024.f;
                const bool isLowDdrMemory = ddr.total > 0 && ddr.remaining >= 0 && ddr.remaining * 100 < ddr.total * LOW_DDR_FREE_MEMORY_THRESHOLD_PERCENT;
                if(isLowDdrMemory && now - lastLowMemoryWarning >= LOW_DDR_MEMORY_WARNING_PERIOD) {
                    systemLogger->warn(
                        "Low memory: {:.2f} MiB free out of {:.2f} MiB ({:.2f}% free).",
                        ddr.remaining * mB,
                        ddr.total * mB,
                        ddr.remaining * 100.0f / ddr.total);
                    lastLowMemoryWarning = now;
                }
                std::this_thread::sleep_for(LOW_DDR_MEMORY_CHECK_PERIOD);
            }
        });
        systemLogThread = std::thread([&isSystemLogRunning, &loggingRateHz, &loggingRateHzMtx, &loggingRateHzCv, &systemLogSleepCv, &systemLogSleepMutex]() {
            const auto systemLogger = pipelineBuilder.getSystemLogger();
            Measurements measurements(systemLogger);
            constexpr auto TELEMETRY_USAGE_PERIOD = std::chrono::minutes(5);
            auto lastTelemetryUsageEvent = std::chrono::steady_clock::now() - TELEMETRY_USAGE_PERIOD;

            std::vector<Measurements::cpuData> entriesStart;
            std::vector<Measurements::cpuData> entriesEnd;
            Measurements::cpuData totalStart;
            Measurements::cpuData totalEnd;

            bool isFirstRun = true;
            while(isSystemLogRunning) {
                // Retrieve current set rate
                float rateHz = loggingRateHz.load();

                bool wasWaiting = false;
                // if rate <= 0 - block until new change
                while(rateHz <= 0) {
                    wasWaiting = true;
                    // get lock and wait until a new change happens (rate > 0, predicate)
                    std::unique_lock<std::mutex> lock(loggingRateHzMtx);
                    loggingRateHzCv.wait(lock, [&loggingRateHz, &isSystemLogRunning]() { return loggingRateHz.load() > 0 || !isSystemLogRunning; });
                    if(!isSystemLogRunning) {
                        break;
                    }
                    rateHz = loggingRateHz.load();
                }
                // rateHz is now guranteed > 0, so below sleep function is okay to call
                if(isFirstRun || wasWaiting) {
                    isFirstRun = false;
                    measurements.readStatsCpu(entriesEnd, totalEnd);
                }

                // Measure CPU load
                entriesStart.swap(entriesEnd);
                totalStart = totalEnd;
                const std::chrono::duration<float> sleepPeriod(1.0f / rateHz);
                std::unique_lock<std::mutex> sleepLock(systemLogSleepMutex);
                systemLogSleepCv.wait_for(sleepLock, sleepPeriod);
                if(!isSystemLogRunning) {
                    break;
                }
                measurements.readStatsCpu(entriesEnd, totalEnd);

                dai::SystemInformationRVC4 pSysInfo;
                measurements.readSystemInformation(pSysInfo, sleepPeriod, entriesStart, entriesEnd, totalStart, totalEnd);
                const auto now = std::chrono::steady_clock::now();
                if(now - lastTelemetryUsageEvent >= TELEMETRY_USAGE_PERIOD) {
                    lastTelemetryUsageEvent = now;
                    dai::DeviceTelemetry::getInstance().event("depthai_rvc4_device_usage", rvc4DeviceUsageProperties(pSysInfo));
                }
                const float mB = 1.0f / 1024.f / 1024.f;
                systemLogger->info("Memory Usage - DDR: {:.2f} / {:.2f} MiB", pSysInfo.ddrMemoryUsage.used * mB, pSysInfo.ddrMemoryUsage.total * mB);
                systemLogger->info(
                    "Temperatures - Average: {:.2f} °C, cpuss {:.2f} °C, gpuss {:.2f} °C, mdmss {:.2f} °C, video {:.2f} °C, ddr {:.2f} °C, camera {:.2f} °C ",
                    pSysInfo.chipTemperature.average,
                    pSysInfo.chipTemperature.cpuss,
                    pSysInfo.chipTemperature.gpuss,
                    pSysInfo.chipTemperature.mdmss,
                    pSysInfo.chipTemperature.video,
                    pSysInfo.chipTemperature.ddr,
                    pSysInfo.chipTemperature.camera);
                std::string message = fmt::format("Cpu Usage - Average: {:.2f}%", pSysInfo.cpuAvgUsage.average * 100.0f);
                const size_t count = pSysInfo.cpuUsages.size();
                for(size_t i = 0; i < count; ++i) {
                    message += fmt::format(", [{}]: {:.2f}%", i, pSysInfo.cpuUsages[i].average * 100.0f);
                }
                systemLogger->info(message);
                systemLogger->info("Device Process Usage - Memory: {:.2f} MiB, Cpu: {:.2f}%",
                                   pSysInfo.processMemoryUsage / 1024.f,
                                   pSysInfo.processCpuAvgUsage.average * 100.0f);
            }
        });

        //  TODO(themarpe) - add get LEDs which describe each individual LED
        //  rpcHandle(*mainRpc, "getLeds", []() { return getLeds(); });

        /*
        rpcHandle(*mainRpc, "setSystemInformationLoggingRate", [](float rateHz) {
            // TODO(themarpe) - port
        });

                // Create a measurement logging thread
                auto loggingRateHz = std::make_shared<std::atomic<float>>(0);
                auto loggingRateHzMtx = std::make_shared<std::mutex>();
                auto loggingRateHzCv = std::make_shared<std::condition_variable>();
                rpcHandle(*mainRpc, "setSystemInformationLoggingRate",
           [loggingRateHz, loggingRateHzMtx, loggingRateHzCv](float rateHz){
                    // Notifies logging thread about the rate change
                    std::unique_lock<std::mutex> l(*loggingRateHzMtx);
                    *loggingRateHz = rateHz;
                    loggingRateHzCv->notify_all();
                });
                rpcHandle(*mainRpc, "getSystemInformationLoggingRate",
           [loggingRateHz](){ return loggingRateHz->load();
                });
                auto systemLogger = pipelineBuilder.getSystemLogger();
                if(eeprom_found()){
                    if (eeprom_version == EEPROM_VERSION_UNPROGRAMMED)
                    {
                        systemLogger->warn("Calibration Data on device is empty");
                    }
                    else if (eeprom_version >= EEPROM_VERSION_LAST){
                        systemLogger->warn("Device contains calibration data of
           version {} which is not supported in this version of depthai",
           eeprom_version);
                    }
                } else {
                    // Some OAK-1 don't have an EEPROM. Show the error only if at
           least one of L/R are connected if (detected_cams[1] || detected_cams[2])
           { systemLogger->error("EEPROM chip not detected");
                    }
                }
                measurementThread = std::thread([systemLogger, loggingRateHz,
           loggingRateHzMtx, loggingRateHzCv](){
                    // Get measurement instance
                    auto& measurementInstance = Measurements::getInstance();

                    while(true){

                        // Retrieve current set rate
                        float rateHz = *loggingRateHz;

                        // if rate <= 0 - block until new change
                        if(rateHz <= 0) {
                            // get lock and wait until a new change happens (rate >
           0, predicate) std::unique_lock<std::mutex> lock(*loggingRateHzMtx);
                            loggingRateHzCv->wait(lock, [&loggingRateHz](){ return
           *loggingRateHz > 0; });

                            // set new rate (guranteed to be > 0)
                            float rateHz = *loggingRateHz;
                        }

                        // rate is now guranteed > 0, so below sleep function is
           okay to call
                        std::this_thread::sleep_for(std::chrono::duration<float>(1.0f
           / rateHz));

                        auto ddr = measurementInstance.getMemoryInfoDdr();
                        auto cmx = measurementInstance.getMemoryInfoCmx();
                        auto leonOs = measurementInstance.getMemoryInfoLeonOs();
                        auto leonRt = measurementInstance.getMemoryInfoLeonRt();
                        auto temp = measurementInstance.getChipTemperature();
                        auto cpuLeonOs = measurementInstance.getCpuUsageLeonOs();
                        auto cpuLeonRt = measurementInstance.getCpuUsageLeonRt();

                        float ddrUsed = ddr.used / (float(1024*1024));
                        float ddrTotal = ddr.total / (float(1024*1024));

                        float cmxUsed = cmx.used / (float(1024*1024));
                        float cmxTotal = cmx.total / (float(1024*1024));

                        float leonOsUsed = leonOs.used / (float(1024*1024));
                        float leonOsTotal = leonOs.total / (float(1024*1024));

                        float leonRtUsed = leonRt.used / (float(1024*1024));
                        float leonRtTotal = leonRt.total / (float(1024*1024));

                        systemLogger->info(
                            "Memory Usage - DDR: {:.2f} / {:.2f} MiB, CMX: {:.2f} /
           {:.2f} MiB, LeonOS Heap: {:.2f} / {:.2f} MiB, LeonRT Heap: {:.2f} /
           {:.2f} MiB", ddrUsed, ddrTotal, cmxUsed, cmxTotal, leonOsUsed,
           leonOsTotal, leonRtUsed, leonRtTotal);

                        systemLogger->info(
                            "Temperatures - Average: {:.2f} °C, CSS: {:.2f} °C, MSS
           {:.2f} °C, UPA: {:.2f} °C, DSS: {:.2f} °C", temp.average, temp.css,
           temp.mss, temp.upa, temp.dss);

                        systemLogger->info(
                            "Cpu Usage - LeonOS {:.2f}%, LeonRT: {:.2f}%",
           cpuLeonOs.average * 100, cpuLeonRt.average * 100);

                    }
                });


                // Add a warning and graceful shutdown when temperature goes beyond
           junction temperatureProtectionThread = std::thread([systemLogger](){

                    // Get measurement instance
                    auto& measurementInstance = Measurements::getInstance();
                    int maxCounter = 0;
                    while(true){
                        // 1s pooling interval
                        std::this_thread::sleep_for(1s);

                        auto chipTemp = measurementInstance.getChipTemperature();

                        // MAX temperature 105C
                        // Maximum 4s (3 x critical -> shutdown)
                        bool overMax = false;
                        constexpr auto MAXIMUM_CHIP_TEMPERATURE = 105.0;
                        const auto& readings = {chipTemp.css, chipTemp.mss,
           chipTemp.upa, chipTemp.dss}; for(const auto& temp : readings){ if(temp >
           MAXIMUM_CHIP_TEMPERATURE){ overMax = true; break;
                            }
                        }
                        if(overMax) {
                            maxCounter++;
                            systemLogger->critical("Temperature above {}C. Device
           will shutdown if not reduced. Note: If this is unexpected please reach
           out to support@luxonis.com to discuss/debug.", MAXIMUM_CHIP_TEMPERATURE);
                        } else {
                            maxCounter = 0;
                        }

                        if(maxCounter >= 4){
                            shutdownReset(true, true);
                        }
                    }

                });

                */
        {
            try {
                for(const auto& camera : dai::gate::CameraDiscovery::getInstance().getConnectedCameraFeatures()) {
                    const auto cameraSocket = camera.socket;
                    if(cameraSocket == dai::CameraBoardSocket::AUTO || cameraSocket == dai::CameraBoardSocket::CBA || eeprom_found(cameraSocket)) {
                        continue;
                    }
                    eeprom_open(cameraSocket);
                }
            } catch(const std::exception& ex) {
                pipelineBuilder.getSystemLogger()->debug("Skipping CBA EEPROM open during startup: {}", ex.what());
            }

            bool success;
            std::string errorMsg;
            std::tie(success, errorMsg) = setCalibration(std::nullopt);
            if(!success) {
                pipelineBuilder.getSystemLogger()->warn("Failed to merge CBA calibration into runtime calibration: {}", errorMsg);
            }
        }

        mainRpc->start();

        //( The lambda below corrects the FREE RUNNING counter with the offset
        // supplied. )
        // Do not use except when SURE what is the intention
        // As modifying the monotonic counter will most likely lead to all sorts of
        // errors
        // auto updateFreeRunningCounter = [](const struct timespec& offset){
        //    // convert timespec to ticks
        //    int64_t ns = 0;
        //    MvTimespecToNanos(&ns, &offset);
        //    int64_t ticksOffset = ((ns/1000) * DrvCprGetSysClocksPerUs());

        //    // Update current ticks with offset count,
        //    for(const auto& id : {OS_DRV_FREE_RUN_CNT_CSS ,
        //    OS_DRV_FREE_RUN_CNT_MSS}){
        //        uint64_t ticks;
        //        OsDrvFreeRunCntRead(id, &ticks);
        //        ticks += (int64_t)ticksOffset;
        //        OsDrvFreeRunCntWrite(id, ticks);
        //    }
        //    // Test
        //    //{
        //    //    struct timespec current;
        //    //    MvGetTimestamp(&current);
        //    //    printf("New synced timestamp: %f\n", current.tv_sec +
        //    current.tv_nsec/1000000000.0);
        //    //}
        //};
        ////timesync.setSyncCallback(updateFreeRunningCounter);

        // TODO(themarpe) - port
        // // Start timesync (sync every 5s)
        // timesync.startSync(std::chrono::milliseconds(5000), true);

        // TODO(themarpe) - port
        // // Issue a warning if watchdog was capped
        // if(actualWatchdogTimeout != WATCHDOG_TIMEOUT){
        //     pipelineBuilder.getSystemLogger()->warn("Watchdog was capped from {}
        //     to {}", WATCHDOG_TIMEOUT, actualWatchdogTimeout);
        // }

    } else {
        /*

        // Start watchdog thread, which checks both LeonOS and LeonRT are still
        alive watchdogThread = std::thread([](){ if(actualWatchdogTimeout > 0ms){
                while(true){
                    watchdogKeepalive();
                    std::this_thread::sleep_for(actualWatchdogTimeout / 2);
                }
            }
        });
        sched_param priority_param;
        priority_param.sched_priority = 254;
        pthread_setschedparam(watchdogThread.native_handle(), SCHED_RR,
        &priority_param);

        // Steps to start depthai when flashed

        // Read raw SBR
        uint8_t rawSbr[SBR_RAW_SIZE];
        read_spi_flash(flashFd, rawSbr, SBR_RAW_SIZE, sbrOffset);

        // Parse SBR
        SBR sbr;
        sbr_parse(rawSbr, SBR_RAW_SIZE, &sbr);

        // TODO(themarpe) - depthai can define the section names
        (pipeline/assets/asset_storage)
        // Find all required sections
        SBR_SECTION *pPipelineSection = nullptr, *pAssetsSection = nullptr,
        *pAssetStorageSection = nullptr; for(int i = 0; i < SBR_MAX_NUM_SECTIONS;
        i++){ std::string n(sbr.sections[i].name);
            if(sbr_section_is_valid(&sbr.sections[i])){
                if(n == "pipeline"){
                    pPipelineSection = &sbr.sections[i];
                } else if(n == "assets"){
                    pAssetsSection = &sbr.sections[i];
                } else if(n == "asset_storage"){
                    pAssetStorageSection = &sbr.sections[i];
                }
            }
        }

        // One of the sections wasn't found (all are required even assetStorage if
        size = 0) if(pPipelineSection == nullptr || pAssetsSection == nullptr ||
        pAssetStorageSection == nullptr){
            // Graceful shutdown and stay in bootloader as this is an exception case
            printf("One of sections is a nullptr - pipeline: %08X, assets: %08X,
        assetStorage: %08X\n", pPipelineSection, pAssetsSection,
        pAssetStorageSection); shutdownReset(true, true);
        }

        // Read data
        // Pipeline description msgpack
        std::vector<std::uint8_t> pipelineDescMsgpack;
        pipelineDescMsgpack.resize(pPipelineSection->size);
        watchdog_enable(false);
        read_spi_flash(flashFd, pipelineDescMsgpack.data(), pPipelineSection->size,
        sbrOffset + pPipelineSection->offset); watchdog_enable(true);

        // Assets msgpack
        std::vector<std::uint8_t> assetsMsgpack;
        assetsMsgpack.resize(pAssetsSection->size);
        watchdog_enable(false);
        read_spi_flash(flashFd, assetsMsgpack.data(), assetsMsgpack.size(),
        sbrOffset + pAssetsSection->offset); watchdog_enable(true);


        // Allocate space for asset storage (if assets storage size > 0)
        std::uint8_t* assetStorage = nullptr;
        std::uint32_t assetStorageSize = pAssetStorageSection->size;
        if(pAssetStorageSection->size > 0){
            assetStorage = (std::uint8_t*)
        lpb::Ddr::malloc(pAssetStorageSection->size); watchdog_enable(false);
            read_spi_flash(flashFd, assetStorage, assetStorageSize, sbrOffset +
        pAssetStorageSection->offset); watchdog_enable(true);
        }

        // Parse msgpack binary data
        parsePipeline(pipelineDescMsgpack);
        parseAssets(assetsMsgpack);
        setAssetStorage((std::uint32_t) assetStorage, assetStorageSize);

        // Build and start pipeline
        bool success;
        std::string errorMsg;
        std::tie(success, errorMsg) = buildPipeline();
        if(success) startPipeline();
        else printf("Couldn't build pipeline, error message: %s\n",
        errorMsg.c_str());

        */
    }

    while(isRunning) {
        std::cout << "I'm alive" << std::endl;
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(3000ms);
        std::cout << "Still alive." << std::endl;
    }

    // Shutdown execution, etc...
    // Join the threads: destroying a joinable std::thread at the end of main calls std::terminate
    isSystemLogRunning = false;
    isLowMemoryCheckRunning = false;
    {
        // Notify under the lock, so the predicate wait cannot miss the stop
        std::lock_guard<std::mutex> lock(loggingRateHzMtx);
        loggingRateHzCv.notify_all();
    }
    systemLogSleepCv.notify_all();
    if(systemLogThread.joinable()) systemLogThread.join();
    if(lowMemoryCheckThread.joinable()) lowMemoryCheckThread.join();
    pipeline.stop();
    pipeline.wait();
    dai::gate::FsyncController::getInstance().stop();
    dai::DeviceTelemetry::getInstance().stop();

    mainRpc = nullptr;
    isWatchdogRunning = false;
    if(watchdogThread.joinable()) watchdogThread.join();
    // TODO - seems to block, bring back when exit is cleaner
    gst_deinit();
    // Node threads are joined, but their asynchronous log callbacks may still be queued.
    // Drain and join spdlog's worker while the callback's Logging target is still alive.
    spdlog::shutdown();
    logging = nullptr;

    // Bye bye
    printf("DepthAI Device KB - Bye Bye!\n");

    return 0;
}

/*
bool checkBootAndInitFlash(int& flashFd, uint32_t& sbrOffset){

    // check if booted from bootloader
    flashFd = -1;
    sbrOffset = 0;

    bool bootedByBootloader = *bootFlags & BOOTED_BY_BOOTLOADER;

    if(bootedByBootloader){
        // Booted from bootloader = false (TODO, SBR location: flash / emmc /
...)
        // Init SPI device, and read from SBR
        flashFd = init_spi_flash();
        sbrOffset = GET_REG_WORD_VAL(AON_RETENTION7_ADR);
        printf("Initing flash... (sbrOffset = 0x%08X)\n", (unsigned) sbrOffset);

        // if init failed
        if(flashFd < 0){
            bootedByBootloader = false;
        }
    }


    return bootedByBootloader;

}
*/
