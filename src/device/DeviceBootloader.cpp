#include "device/DeviceBootloader.hpp"

#include <fstream>

#include "XLink/XLink.h"
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/SBR.h"
#include "depthai-bootloader-shared/Structure.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
#include "spdlog/fmt/chrono.h"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/Platform.hpp"
#include "utility/Resources.hpp"
#include "utility/spdlog-fmt.hpp"

namespace dai {

namespace Request = bootloader::request;
namespace Response = bootloader::response;

constexpr const DeviceBootloader::Type DeviceBootloader::DEFAULT_TYPE;
constexpr const char* NETWORK_BOOTLOADER_FLASH_UTILITY_PATH = "depthai-core/utilities/flash_network_bootloader.py";
constexpr const char* NETWORK_BOOTLOADER_DOCS_URL = "https://docs.luxonis.com/software-v3/depthai/depthai-components/bootloader";

std::tuple<bool, DeviceInfo> DeviceBootloader::getFirstAvailableDevice() {
    auto devices = XLinkConnection::getAllConnectedDevices();
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED}) {
        for(const auto& device : devices) {
            if(device.state == searchState) {
                return {true, device};
            }
        }
    }
    return {false, {}};
}

std::vector<DeviceInfo> DeviceBootloader::getAllAvailableDevices() {
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices();
    for(const auto& d : connectedDevices) {
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init(true, {}, std::nullopt, false);
}

template <>
DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, bool allowFlashingBootloader) : deviceInfo(devInfo) {
    init(true, {}, std::nullopt, allowFlashingBootloader);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, Type type, bool allowFlashingBootloader) : deviceInfo(devInfo) {
    init(true, {}, type, allowFlashingBootloader);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, const fs::path& pathToBootloader, bool allowFlashingBootloader) : deviceInfo(devInfo) {
    init(false, pathToBootloader, std::nullopt, allowFlashingBootloader);
}

DeviceBootloader::DeviceBootloader(std::string nameOrDeviceId, bool allowFlashingBootloader) : deviceInfo(std::move(nameOrDeviceId)) {
    init(true, {}, std::nullopt, allowFlashingBootloader);
}

void DeviceBootloader::createWatchdog() {
    if(monitorThread.joinable() || watchdogThread.joinable()) {
        throw std::runtime_error("Watchdog already created. Destroy it first.");
    }
    watchdogRunning = true;
    {
        std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
        lastWatchdogPingTime = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    }
    monitorThread = std::thread([this]() {
        while(watchdogRunning) {
            std::this_thread::sleep_for(bootloader::XLINK_WATCHDOG_TIMEOUT);
            decltype(lastWatchdogPingTime) prevPingTime;
            {
                std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
                prevPingTime = lastWatchdogPingTime;
            }
            std::chrono::milliseconds watchdogTimeout = std::chrono::milliseconds(3000);  // Raised to avoid spurious warnings and closes
            if(watchdogRunning && std::chrono::steady_clock::now() - prevPingTime > watchdogTimeout * 2) {
                logger::warn("Monitor thread (device: {} [{}]) - ping was missed, closing the device connection", deviceInfo.deviceId, deviceInfo.name);
                watchdogRunning = false;
                connection->close();
            }
        }
    });

    watchdogThread = std::thread([this]() {
        try {
            XLinkStream stream(connection, bootloader::XLINK_CHANNEL_WATCHDOG, 64);  // Throws on quick start/stop: close() already closed the connection
            std::vector<uint8_t> watchdogKeepalive = {0, 0, 0, 0};
            std::vector<uint8_t> reset = {1, 0, 0, 0};
            while(watchdogRunning) {
                try {
                    stream.write(watchdogKeepalive);
                } catch(const std::exception&) {
                    break;
                }
                {
                    std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
                    lastWatchdogPingTime = std::chrono::steady_clock::now();
                }
                std::this_thread::sleep_for(bootloader::XLINK_WATCHDOG_TIMEOUT / 2);
            }

            try {
                stream.write(reset);
                const auto dummy = stream.readMove();  // Waits until the link falls down
            } catch(const std::exception&) {
            }
        } catch(const std::exception&) {
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // So that the device is not available anymore
    });
}

void DeviceBootloader::destroyWatchdog() {
    watchdogRunning = false;
    if(watchdogThread.joinable()) watchdogThread.join();
    if(monitorThread.joinable()) monitorThread.join();
}

void DeviceBootloader::init(bool embeddedMvcmd, const fs::path& pathToMvcmd, std::optional<bootloader::Type> type, bool allowBlFlash) {
    stream = nullptr;
    allowFlashingBootloader = allowBlFlash;

    bootloaderType = type.value_or(DEFAULT_TYPE);

    if((deviceInfo.state == X_LINK_ANY_STATE || deviceInfo.protocol == X_LINK_ANY_PROTOCOL) && (!deviceInfo.deviceId.empty() || !deviceInfo.name.empty())) {
        deviceDesc_t foundDesc;
        auto ret = XLinkConnection::findFirstSuitableDevice(deviceInfo, foundDesc);
        if(ret == X_LINK_SUCCESS) {
            deviceInfo = DeviceInfo(foundDesc);
            logger::debug("Found an actual device by given DeviceInfo: {}", deviceInfo.toString());
        } else {
            throw std::runtime_error("Specified device not found");
        }
    }

    try {
        if(deviceInfo.state == X_LINK_UNBOOTED) {
            if(embeddedMvcmd) {
                connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(bootloaderType), X_LINK_BOOTLOADER);
            } else {
                connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
            }

            stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

            createWatchdog();

            version = requestVersion();

            isEmbedded = true;
        } else if(deviceInfo.state == X_LINK_BOOTLOADER || deviceInfo.state == X_LINK_FLASH_BOOTED) {
            if(deviceInfo.state == X_LINK_FLASH_BOOTED) {
                deviceInfo = XLinkConnection::bootBootloader(deviceInfo);
            }

            connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_BOOTLOADER);

            createWatchdog();

            stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

            version = requestVersion();
            if(version >= Version(0, 0, 12)) {
                if(!sendRequest(Request::GetBootloaderType{})) {
                    throw std::runtime_error("Error trying to connect to device");
                }
                Response::BootloaderType runningBootloaderType;
                if(!receiveResponse(runningBootloaderType)) throw std::runtime_error("Error trying to connect to device");

                bootloaderType = runningBootloaderType.type;

                Type desiredBootloaderType = type.value_or(bootloaderType);

                if((desiredBootloaderType != bootloaderType) || allowFlashingBootloader) {
                    Request::BootMemory bootMemory;
                    auto binary = getEmbeddedBootloaderBinary(desiredBootloaderType);
                    bootMemory.totalSize = static_cast<uint32_t>(binary.size());
                    bootMemory.numPackets = ((static_cast<uint32_t>(binary.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
                    if(!sendRequest(bootMemory)) {
                        throw std::runtime_error("Error trying to connect to device");
                    }

                    stream->writeSplit(binary.data(), binary.size(), bootloader::XLINK_STREAM_MAX_SIZE);
                    stream = nullptr;
                    destroyWatchdog();
                    connection->close();

                    connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_BOOTLOADER);

                    createWatchdog();

                    stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

                    version = requestVersion();

                    bootloaderType = desiredBootloaderType;

                    isEmbedded = true;
                } else {
                    isEmbedded = false;
                }

            } else {
                Type desiredBootloaderType = type.value_or(Type::USB);  // Too old for an in memory boot: Bootloader -> USB ROM -> Boot

                if((desiredBootloaderType != Type::USB) || allowFlashingBootloader) {
                    if(!sendRequest(Request::UsbRomBoot{})) {
                        throw std::runtime_error("Error trying to connect to device");
                    }
                    stream = nullptr;
                    connection->close();

                    if(embeddedMvcmd) {
                        connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(desiredBootloaderType), X_LINK_BOOTLOADER);
                    } else {
                        connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
                    }

                    createWatchdog();

                    stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

                    version = requestVersion();

                    bootloaderType = desiredBootloaderType;

                    isEmbedded = true;

                } else {
                    bootloaderType = dai::bootloader::Type::USB;

                    isEmbedded = false;
                }
            }

        } else {
            throw std::runtime_error("Device not in UNBOOTED, BOOTLOADER or FLASH_BOOTED state");
        }

        deviceInfo.state = X_LINK_BOOTLOADER;

        logger::debug("Connected bootloader version {}", version.toString());
        if(getType() == Type::NETWORK && getEmbeddedBootloaderVersion() > version) {
            logger::warn(
                "An optional NETWORK bootloader update is available (installed: {}, available: {}). "
                "Updating is recommended to improve device discoverability. Run 'depthai --flash' or '{}', "
                "or see {} for more information.",
                version.toString(),
                getEmbeddedBootloaderVersion().toString(),
                NETWORK_BOOTLOADER_FLASH_UTILITY_PATH,
                NETWORK_BOOTLOADER_DOCS_URL);
        }

    } catch(...) {
        destroyWatchdog();  // Constructor context: close the threads here
        throw;
    }
}

void DeviceBootloader::close() {
    if(closed.exchange(true)) return;

    using namespace std::chrono;
    auto t1 = steady_clock::now();
    logger::debug("DeviceBootloader about to be closed...");

    connection->close();  // Close first: XLink calls then unblock semaphore waits, which unblocks the queues

    destroyWatchdog();

    stream = nullptr;

    logger::debug("DeviceBootloader closed, {}", duration_cast<milliseconds>(steady_clock::now() - t1).count());
}

bool DeviceBootloader::isClosed() const {  // The result can be stale as soon as it returns
    return closed || !watchdogRunning;
}

DeviceBootloader::~DeviceBootloader() {
    close();
}

DeviceBootloader::Version DeviceBootloader::getEmbeddedBootloaderVersion() {
    return DeviceBootloader::Version(DEPTHAI_BOOTLOADER_VERSION);
}

DeviceBootloader::Version DeviceBootloader::getVersion() const {
    return version;
}

DeviceBootloader::Version DeviceBootloader::requestVersion() {
    if(!sendRequest(Request::GetBootloaderVersion{})) {
        throw std::runtime_error("Couldn't get bootloader version");
    }

    Response::BootloaderVersion ver;
    if(!receiveResponse(ver)) {
        throw std::runtime_error("Couldn't parse version response");
    }

    Version blVersion(ver.major, ver.minor, ver.patch);

    if(blVersion >= Version(Request::GetBootloaderCommit::VERSION)) {
        Request::GetBootloaderCommit request{};
        stream->write({(uint8_t*)&request, sizeof(request)});

        Response::BootloaderCommit commit;
        if(!receiveResponse(commit)) {
            throw std::runtime_error("Couldn't get bootloader commit");
        }

        blVersion = Version(ver.major, ver.minor, ver.patch, commit.commitStr);
    }

    return blVersion;
}

DeviceBootloader::Type DeviceBootloader::getType() const {
    return bootloaderType;
}

bool DeviceBootloader::isAllowedFlashingBootloader() const {
    return allowFlashingBootloader;
}

DeviceBootloader::ApplicationInfo DeviceBootloader::readApplicationInfo(Memory mem) {
    Request::GetApplicationDetails appDetails;
    appDetails.memory = mem;

    sendRequestThrow(appDetails);

    Response::ApplicationDetails details;
    receiveResponseThrow(details);

    ApplicationInfo info;
    info.memory = mem;
    info.firmwareVersion = "";
    info.applicationName = "";

    info.hasApplication = details.hasApplication;
    if(details.hasFirmwareVersion) {
        info.firmwareVersion = std::string(details.firmwareVersionStr);
    }
    if(details.hasApplicationName) {
        info.applicationName = std::string(details.applicationNameStr);
    }

    if(!details.success) {
        throw std::runtime_error(details.errorMsg);
    }

    return info;
}

DeviceBootloader::MemoryInfo DeviceBootloader::getMemoryInfo(Memory memory) {
    if(memory == Memory::EMMC && bootloaderType == Type::USB) {
        logger::warn("USB Bootloader type does NOT support eMMC");
    }

    Request::GetMemoryDetails req{};
    req.memory = memory;
    sendRequestThrow(req);

    Response::MemoryDetails details;
    receiveResponseThrow(details);

    MemoryInfo mem;
    mem.available = details.hasMemory;
    mem.size = details.memorySize;
    mem.info = std::string(details.memoryInfo);

    return mem;
}

bool DeviceBootloader::isUserBootloaderSupported() {
    if(getType() != Type::NETWORK) {
        return false;
    }

    if(getVersion() < Version(Request::IsUserBootloader::VERSION)) {
        return false;
    }

    return true;
}

bool DeviceBootloader::isUserBootloader() {
    if(getVersion() < Version(Request::IsUserBootloader::VERSION)) {
        return false;
    }

    Request::IsUserBootloader req{};
    sendRequestThrow(req);

    Response::IsUserBootloader user;
    receiveResponseThrow(user);

    return user.isUserBootloader;
}

std::tuple<bool, std::string> DeviceBootloader::flashClear(Memory memory) {
    std::vector<uint8_t> clear;
    for(size_t i = 0; i < SBR_RAW_SIZE; i++) {
        clear.push_back(0xFF);
    }
    return flashCustom(memory, bootloader::getStructure(getType()).offset.at(Section::APPLICATION), clear);
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(const std::function<void(float)>& progressCb, const fs::path& path) {
    return flashBootloader(Memory::FLASH, bootloaderType, progressCb, path);
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(Memory memory, Type type, std::function<void(float)> progressCb, const fs::path& path) {
    if(!allowFlashingBootloader) {
        throw std::invalid_argument("DeviceBootloader wasn't initialized to allow flashing bootloader. Set 'allowFlashingBootloader' in constructor");
    }

    if(type == Type::AUTO) {
        type = getType();
    }

    if(memory != Memory::FLASH) {
        throw std::invalid_argument("Only FLASH memory is supported for now");
    }
    if(bootloaderType != type && getVersion() < Version(Request::UpdateFlashEx2::VERSION)) {
        throw std::runtime_error("Current bootloader version doesn't support flashing different type of bootloader");
    }

    std::vector<uint8_t> package;
    if(!path.empty()) {
        std::ifstream fwStream(path, std::ios::binary);
        if(!fwStream.is_open()) throw std::runtime_error(fmt::format("Cannot flash bootloader, binary at path: {} doesn't exist", path));
        package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        package = getEmbeddedBootloaderBinary(type);
    }

    if(bootloaderType == type) {  // Different types need UpdateFlashEx2 to flash properly
        Request::UpdateFlash updateFlash;
        updateFlash.storage = Request::UpdateFlash::BOOTLOADER;
        updateFlash.totalSize = static_cast<uint32_t>(package.size());
        updateFlash.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
        if(!sendRequest(updateFlash)) return {false, "Couldn't send bootloader flash request"};

    } else {
        Request::UpdateFlashEx2 updateFlashEx2;
        updateFlashEx2.memory = memory;
        updateFlashEx2.offset = dai::bootloader::getStructure(type).offset.at(Section::BOOTLOADER);
        updateFlashEx2.totalSize = static_cast<uint32_t>(package.size());
        updateFlashEx2.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
        if(!sendRequest(updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};
    }

    stream->writeSplit(package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    do {
        std::vector<uint8_t> data;
        if(!receiveResponseData(data)) return {false, "Couldn't receive bootloader response"};

        Response::FlashStatusUpdate update;
        if(parseResponse(data, update)) {
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
        } else if(parseResponse(data, result)) {
            break;
        } else {
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashUserBootloader(std::function<void(float)> progressCb, const fs::path& path) {
    const auto type = Type::NETWORK;
    if(getType() != Type::NETWORK) {
        throw std::runtime_error("Flashing User Bootloader is only available for NETWORK bootloaders");
    }
    const auto memory = Memory::FLASH;

    if(getVersion() < Version(Request::IsUserBootloader::VERSION)) {
        throw std::runtime_error("Current bootloader version doesn't support User Bootloader");
    }

    std::vector<uint8_t> package;
    if(!path.empty()) {
        std::ifstream fwStream(path, std::ios::binary);
        if(!fwStream.is_open()) throw std::runtime_error(fmt::format("Cannot flash User Bootloader, binary at path: {} doesn't exist", path));
        package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        package = getEmbeddedBootloaderBinary(type);
    }

    Request::UpdateFlashEx2 updateFlashEx2;
    updateFlashEx2.memory = memory;
    updateFlashEx2.offset = dai::bootloader::getStructure(type).offset.at(Section::USER_BOOTLOADER);
    updateFlashEx2.totalSize = static_cast<uint32_t>(package.size());
    updateFlashEx2.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;

    const auto MAX_USER_BOOTLOADER_SIZE = dai::bootloader::getStructure(type).size.at(Section::USER_BOOTLOADER);
    if(static_cast<long>(updateFlashEx2.totalSize) > MAX_USER_BOOTLOADER_SIZE) {
        throw std::runtime_error(fmt::format("Selected User Bootloader is too large {} / {}B", updateFlashEx2.totalSize, MAX_USER_BOOTLOADER_SIZE));
    }

    if(!sendRequest(updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};

    stream->writeSplit(package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    do {
        std::vector<uint8_t> data;
        if(!receiveResponseData(data)) return {false, "Couldn't receive bootloader response"};

        Response::FlashStatusUpdate update;
        if(parseResponse(data, update)) {
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
        } else if(parseResponse(data, result)) {
            break;
        } else {
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    if(!result.success) {  // Do not register a failed user bootloader: the factory one stays the fallback
        return {false, result.errorMsg};
    }

    nlohmann::json configJson;
    try {
        configJson = readConfigData();
    } catch(const std::exception& ex) {
        logger::debug("Error while trying to read existing bootloader configuration: {}", ex.what());
    }
    const auto userBlSize = static_cast<std::uint32_t>(package.size());
    const auto userBlChecksum = sbr_compute_checksum(package.data(), static_cast<uint32_t>(package.size()));
    configJson["userBlSize"] = userBlSize;
    configJson["userBlChecksum"] = userBlChecksum;
    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = flashConfigData(configJson);
    if(success) {
        logger::debug("Success flashing the configuration userBlSize to '{}' and userBlChecksum to '{}'", userBlSize, userBlChecksum);
    } else {
        throw std::runtime_error(errorMsg);
    }

    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashGpioModeBootHeader(Memory memory, int gpioMode) {
    // TODO(themarpe) - use memory param
    (void)memory;

    Request::UpdateFlashBootHeader updateBootHeader;
    updateBootHeader.type = Request::UpdateFlashBootHeader::GPIO_MODE;
    updateBootHeader.gpioMode = gpioMode;

    if(!sendRequest(updateBootHeader)) return {false, "Couldn't send request to flash boot header"};
    Response::FlashComplete resp = {};
    receiveResponse(resp);
    return {resp.success, resp.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashUsbRecoveryBootHeader(Memory memory) {
    // TODO(themarpe) - use memory param
    (void)memory;

    Request::UpdateFlashBootHeader updateBootHeader;
    updateBootHeader.type = Request::UpdateFlashBootHeader::USB_RECOVERY;

    if(!sendRequest(updateBootHeader)) return {false, "Couldn't send request to flash boot header"};
    Response::FlashComplete resp = {};
    receiveResponse(resp);
    return {resp.success, resp.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashBootHeader(Memory memory, int32_t frequency, int64_t location, int32_t dummyCycles, int64_t offset) {
    // TODO(themarpe) - use memory param
    (void)memory;

    Request::UpdateFlashBootHeader updateBootHeader;
    updateBootHeader.type = Request::UpdateFlashBootHeader::NORMAL;
    updateBootHeader.offset = offset;
    updateBootHeader.location = location;
    updateBootHeader.dummyCycles = dummyCycles;
    updateBootHeader.frequency = frequency;
    updateBootHeader.gpioMode = 0x7;  // 0x7: optimized GPIO boot mode, which allows the above parameters

    if(!sendRequest(updateBootHeader)) return {false, "Couldn't send request to flash boot header"};
    Response::FlashComplete resp = {};
    receiveResponse(resp);
    return {resp.success, resp.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashFastBootHeader(Memory memory, int32_t frequency, int64_t location, int32_t dummyCycles, int64_t offset) {
    // TODO(themarpe) - use memory param
    (void)memory;

    Request::UpdateFlashBootHeader updateBootHeader;
    updateBootHeader.type = Request::UpdateFlashBootHeader::FAST;
    updateBootHeader.offset = offset;
    updateBootHeader.location = location;
    updateBootHeader.dummyCycles = dummyCycles;
    updateBootHeader.frequency = frequency;
    updateBootHeader.gpioMode = 0x7;  // 0x7: optimized GPIO boot mode, which allows the above parameters

    if(!sendRequest(updateBootHeader)) return {false, "Couldn't send request to flash boot header"};
    Response::FlashComplete resp = {};
    receiveResponse(resp);
    return {resp.success, resp.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashCustom(Memory memory,
                                                            size_t offset,
                                                            const std::vector<uint8_t>& data,
                                                            const std::function<void(float)>& progressCb) {
    if(data.size() == 0) {
        throw std::invalid_argument("Size to flash is zero");
    }
    return flashCustom(memory, offset, data.data(), data.size(), "", progressCb);
}
std::tuple<bool, std::string> DeviceBootloader::flashCustom(
    Memory memory, size_t offset, const uint8_t* data, size_t size, const std::function<void(float)>& progressCb) {
    if(data == nullptr || size == 0) {
        throw std::invalid_argument("Data is nullptr or size is zero");
    }
    return flashCustom(memory, offset, data, size, "", progressCb);
}
std::tuple<bool, std::string> DeviceBootloader::flashCustom(Memory memory,
                                                            size_t offset,
                                                            const std::string& filename,
                                                            const std::function<void(float)>& progressCb) {
    return flashCustom(memory, offset, nullptr, 0, filename, progressCb);
}
std::tuple<bool, std::string> DeviceBootloader::flashCustom(
    Memory memory, size_t offset, const uint8_t* data, size_t size, std::string filename, std::function<void(float)> progressCb) {
    if(getVersion() < Version(0, 0, 12)) {
        throw std::runtime_error("Current bootloader version doesn't support custom flashing");
    }

    std::vector<uint8_t> optFileData;
    if(!filename.empty()) {
        std::ifstream optFile(filename, std::ios::in | std::ios::binary);
        optFileData = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(optFile), {});
        data = optFileData.data();
        size = optFileData.size();
    }

    Request::UpdateFlashEx2 updateFlashEx2;
    updateFlashEx2.memory = memory;
    updateFlashEx2.offset = static_cast<uint32_t>(offset);
    updateFlashEx2.totalSize = static_cast<uint32_t>(size);
    updateFlashEx2.numPackets = ((static_cast<uint32_t>(size) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendRequest(updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};

    stream->writeSplit(data, size, bootloader::XLINK_STREAM_MAX_SIZE);

    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    do {
        std::vector<uint8_t> responseData;
        if(!receiveResponseData(responseData)) return {false, "Couldn't receive bootloader response"};

        Response::FlashStatusUpdate update;
        if(parseResponse(responseData, update)) {
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
        } else if(parseResponse(responseData, result)) {
            break;
        } else {
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::readCustom(
    Memory memory, size_t offset, size_t size, std::vector<uint8_t>& data, const std::function<void(float)>& progressCb) {
    if(data.size() < size) {
        data.resize(size);
    }
    return readCustom(memory, offset, size, data.data(), "", progressCb);
}
std::tuple<bool, std::string> DeviceBootloader::readCustom(
    Memory memory, size_t offset, size_t size, uint8_t* data, const std::function<void(float)>& progressCb) {
    return readCustom(memory, offset, size, data, "", progressCb);
}
std::tuple<bool, std::string> DeviceBootloader::readCustom(
    Memory memory, size_t offset, size_t size, const std::string& filename, const std::function<void(float)>& progressCb) {
    return readCustom(memory, offset, size, nullptr, filename, progressCb);
}
std::tuple<bool, std::string, std::vector<uint8_t>> DeviceBootloader::readCustom(Memory memory,
                                                                                 size_t offset,
                                                                                 size_t size,
                                                                                 const std::function<void(float)>& progressCb) {
    std::vector<uint8_t> data;
    auto ret = readCustom(memory, offset, size, data, progressCb);
    return {std::get<0>(ret), std::get<1>(ret), data};
}

std::tuple<bool, std::string> DeviceBootloader::readCustom(
    Memory memory, size_t offset, size_t size, uint8_t* data, std::string filename, std::function<void(float)> progressCb) {
    Request::ReadFlash readFlash;
    readFlash.memory = memory;
    readFlash.offset = static_cast<uint32_t>(offset);
    readFlash.totalSize = static_cast<uint32_t>(size);
    if(!sendRequest(readFlash)) return {false, "Couldn't send bootloader flash request"};

    Response::ReadFlash response;
    receiveResponse(response);
    if(!response.success) {
        return {false, std::string(response.errorMsg)};
    }

    if(filename.empty()) {
        size_t dataOffset = 0;
        for(unsigned i = 0; i < response.numPackets; i++) {
            auto d = stream->read();
            memcpy(data + dataOffset, d.data(), d.size());
            dataOffset += d.size();
            if(progressCb) progressCb((1.0f / response.numPackets) * (i + 1));
        }
    } else {
        std::ofstream outputFile(filename);
        for(unsigned i = 0; i < response.numPackets; i++) {
            auto d = stream->read();
            outputFile.write(reinterpret_cast<char*>(d.data()), d.size());
            if(progressCb) progressCb((1.0f / response.numPackets) * (i + 1));
        }
    }

    return {response.success, response.errorMsg};
}

nlohmann::json DeviceBootloader::readConfigData(Memory memory, Type type) {
    Request::GetBootloaderConfig getConfigReq;
    getConfigReq.memory = memory;

    if(type != Type::AUTO) {
        const auto confStructure = bootloader::getStructure(type);
        getConfigReq.offset = confStructure.offset.at(bootloader::Section::BOOTLOADER_CONFIG);
        getConfigReq.maxSize = confStructure.size.at(bootloader::Section::BOOTLOADER_CONFIG);
    } else {
    }  // Default values mean AUTO

    sendRequestThrow(getConfigReq);

    Response::GetBootloaderConfig resp;
    resp.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4

    receiveResponseThrow(resp);
    if(resp.success) {
        auto bsonConfig = stream->read();  // One packet max
        return nlohmann::json::from_bson(bsonConfig);
    } else {
        throw std::runtime_error(resp.errorMsg);
    }
}

std::tuple<bool, std::string> DeviceBootloader::flashConfigClear(Memory memory, Type type) {
    Request::SetBootloaderConfig setConfigReq;
    setConfigReq.memory = memory;
    if(type != Type::AUTO) {
        setConfigReq.offset = bootloader::getStructure(type).offset.at(bootloader::Section::BOOTLOADER_CONFIG);
    }

    setConfigReq.numPackets = 0;
    setConfigReq.totalSize = 0;
    setConfigReq.clearConfig = 1;
    if(!sendRequest(setConfigReq)) return {false, "Couldn't send request to flash configuration clear"};

    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    if(!receiveResponse(result)) {
        return {false, "Couldn't receive response to flash configuration clear"};
    }

    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashConfigData(const nlohmann::json& configData, Memory memory, Type type) {
    auto bson = nlohmann::json::to_bson(configData);

    Request::SetBootloaderConfig setConfigReq;
    setConfigReq.memory = memory;
    if(type != Type::AUTO) {
        setConfigReq.offset = bootloader::getStructure(type).offset.at(bootloader::Section::BOOTLOADER_CONFIG);
    }
    setConfigReq.numPackets = 1;
    setConfigReq.totalSize = static_cast<decltype(setConfigReq.totalSize)>(bson.size());
    setConfigReq.clearConfig = 0;
    if(!sendRequest(setConfigReq)) return {false, "Couldn't send request to flash configuration data"};

    stream->write(bson);

    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    if(!receiveResponse(result)) {
        return {false, "Couldn't receive response to flash configuration data"};
    }

    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashConfigFile(const fs::path& configPath, Memory memory, Type type) {
    std::ifstream configInputStream(configPath);
    if(!configInputStream.is_open()) throw std::runtime_error(fmt::format("Cannot flash configuration, JSON at path: {} doesn't exist", configPath));
    nlohmann::json configJson;
    configInputStream >> configJson;
    return flashConfigData(configJson, memory, type);
}

DeviceBootloader::Config DeviceBootloader::readConfig(Memory memory, Type type) {
    auto json = readConfigData(memory, type);
    return Config::fromJson(json);
}

std::tuple<bool, std::string> DeviceBootloader::flashConfig(const Config& config, Memory memory, Type type) {
    return flashConfigData(config.toJson(), memory, type);
}

void DeviceBootloader::bootMemory(const std::vector<uint8_t>& embeddedFw) {
    Request::BootMemory bootMemory;
    bootMemory.totalSize = static_cast<uint32_t>(embeddedFw.size());
    bootMemory.numPackets = ((static_cast<uint32_t>(embeddedFw.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendRequest(bootMemory)) {
        throw std::runtime_error("Error trying to connect to device");
    }

    stream->writeSplit(embeddedFw.data(), embeddedFw.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    try {  // Waits until the link falls down
        stream->read();
    } catch(const std::exception&) {
    }
}

void DeviceBootloader::bootUsbRomBootloader() {
    if(!sendRequest(Request::UsbRomBoot{})) {
        throw std::runtime_error("Error trying to connect to device");
    }

    try {  // Waits until the link falls down
        stream->read();
    } catch(const std::exception&) {
    }
}

bool DeviceBootloader::isEmbeddedVersion() const {
    return isEmbedded;
}

std::vector<std::uint8_t> DeviceBootloader::getEmbeddedBootloaderBinary(Type type) {
    return Resources::getInstance().getBootloaderFirmware(type);
}

template <typename T>
bool DeviceBootloader::sendRequest(const T& request) {
    if(stream == nullptr) return false;

    if(getVersion() < Version(T::VERSION)) {
        throw std::runtime_error(
            fmt::format("Bootloader version {} required to send request '{}'. Current version {}", T::VERSION, T::NAME, getVersion().toString()));
    }

    try {
        stream->write({(uint8_t*)&request, sizeof(T)});
    } catch(const std::exception&) {
        return false;
    }

    return true;
}

template <typename T>
void DeviceBootloader::sendRequestThrow(const T& request) {
    if(stream == nullptr) throw std::runtime_error("Couldn't send request. Stream is null");

    if(getVersion() < Version(T::VERSION)) {
        throw std::runtime_error(
            fmt::format("Bootloader version {} required to send request '{}'. Current version {}", T::VERSION, T::NAME, getVersion().toString()));
    }

    try {
        stream->write({(uint8_t*)&request, sizeof(T)});
    } catch(const std::exception&) {
        throw std::runtime_error("Couldn't send " + std::string(T::NAME) + " request");
    }
}

bool DeviceBootloader::receiveResponseData(std::vector<uint8_t>& data) {
    if(stream == nullptr) return false;

    data = stream->read();
    return true;
}

template <typename T>
bool DeviceBootloader::parseResponse(const std::vector<uint8_t>& data, T& response) {
    Response::Command command;
    if(data.size() < sizeof(command)) return false;
    memcpy(&command, data.data(), sizeof(command));
    if(response.cmd != command) return false;
    if(data.size() < sizeof(response)) return false;

    memcpy(&response, data.data(), sizeof(response));
    return true;
}

template <typename T>
bool DeviceBootloader::receiveResponse(T& response) {
    if(stream == nullptr) return false;
    std::vector<uint8_t> data;
    if(!receiveResponseData(data)) return false;

    if(!parseResponse(data, response)) return false;

    return true;
}

template <typename T>
void DeviceBootloader::receiveResponseThrow(T& response) {
    if(stream == nullptr) throw std::runtime_error("Couldn't receive response. Stream is null");

    std::vector<uint8_t> data;
    if(!receiveResponseData(data)) {
        throw std::runtime_error("Couldn't receive " + std::string(T::NAME) + " response");
    }

    if(!parseResponse(data, response)) {
        throw std::runtime_error("Couldn't parse " + std::string(T::NAME) + " response");
    }
}

void DeviceBootloader::Config::setStaticIPv4(const std::string& ip, const std::string& mask, const std::string& gateway) {
    network.ipv4 = platform::getIPv4AddressAsBinary(ip);
    network.ipv4Mask = platform::getIPv4AddressAsBinary(mask);
    network.ipv4Gateway = platform::getIPv4AddressAsBinary(gateway);
    network.staticIpv4 = true;
}
void DeviceBootloader::Config::setDynamicIPv4(const std::string& ip, const std::string& mask, const std::string& gateway) {
    network.ipv4 = platform::getIPv4AddressAsBinary(ip);
    network.ipv4Mask = platform::getIPv4AddressAsBinary(mask);
    network.ipv4Gateway = platform::getIPv4AddressAsBinary(gateway);
    network.staticIpv4 = false;
}

bool DeviceBootloader::Config::isStaticIPV4() {
    return network.staticIpv4;
}

std::string DeviceBootloader::Config::getIPv4() {
    return platform::getIPv4AddressAsString(network.ipv4);
}
std::string DeviceBootloader::Config::getIPv4Mask() {
    return platform::getIPv4AddressAsString(network.ipv4Mask);
}
std::string DeviceBootloader::Config::getIPv4Gateway() {
    return platform::getIPv4AddressAsString(network.ipv4Gateway);
}

void DeviceBootloader::Config::setDnsIPv4(const std::string& dns, const std::string& dnsAlt) {
    network.ipv4Dns = platform::getIPv4AddressAsBinary(dns);
    network.ipv4DnsAlt = platform::getIPv4AddressAsBinary(dnsAlt);
}

std::string DeviceBootloader::Config::getDnsIPv4() {
    return platform::getIPv4AddressAsString(network.ipv4Dns);
}

std::string DeviceBootloader::Config::getDnsAltIPv4() {
    return platform::getIPv4AddressAsString(network.ipv4DnsAlt);
}

void DeviceBootloader::Config::setUsbTimeout(std::chrono::milliseconds ms) {
    usb.timeoutMs = static_cast<decltype(usb.timeoutMs)>(ms.count());
}

std::chrono::milliseconds DeviceBootloader::Config::getUsbTimeout() {
    return std::chrono::milliseconds(usb.timeoutMs);
}

void DeviceBootloader::Config::setNetworkTimeout(std::chrono::milliseconds ms) {
    network.timeoutMs = static_cast<decltype(network.timeoutMs)>(ms.count());
}

std::chrono::milliseconds DeviceBootloader::Config::getNetworkTimeout() {
    return std::chrono::milliseconds(network.timeoutMs);
}

void DeviceBootloader::Config::setUsbMaxSpeed(UsbSpeed speed) {
    usb.maxUsbSpeed = static_cast<int>(speed);
}

UsbSpeed DeviceBootloader::Config::getUsbMaxSpeed() {
    return static_cast<UsbSpeed>(usb.maxUsbSpeed);
}

void DeviceBootloader::Config::setMacAddress(const std::string& mac) {
    std::array<uint8_t, 6> a = {0, 0, 0, 0, 0, 0};
    if(mac != "") {
        int last = -1;
        int rc = std::sscanf(mac.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx%n", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &last);
        if(rc != 6 || static_cast<long>(mac.size()) != last) {
            throw std::invalid_argument("Invalid MAC address format " + mac);
        }
    }

    network.mac = a;
}
std::string DeviceBootloader::Config::getMacAddress() {
    std::array<char, 32> macStr = {};
    std::snprintf(macStr.data(),
                  macStr.size(),
                  "%02X:%02X:%02X:%02X:%02X:%02X",
                  network.mac[0],
                  network.mac[1],
                  network.mac[2],
                  network.mac[3],
                  network.mac[4],
                  network.mac[5]);

    return std::string(macStr.data());
}

nlohmann::json DeviceBootloader::Config::toJson() const {
    nlohmann::json configValues = *this;
    auto dataCopy = data;
    dataCopy.update(configValues);
    return dataCopy;
}

DeviceBootloader::Config DeviceBootloader::Config::fromJson(const nlohmann::json& json) {
    Config cfg = json;
    cfg.data = json;  // Keeps unknown values
    return cfg;
}

}  // namespace dai
