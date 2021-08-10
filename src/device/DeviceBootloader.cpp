#include "device/DeviceBootloader.hpp"

// std
#include <fstream>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/SBR.h"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/pipeline/Assets.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "device/Device.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/BootloaderHelper.hpp"
#include "utility/Resources.hpp"

// libraries
#include "spdlog/spdlog.h"

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {

// constants
constexpr const DeviceBootloader::Type DeviceBootloader::DEFAULT_TYPE;

// static api

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceBootloader::getFirstAvailableDevice() {
    bool found;
    DeviceInfo dev;
    std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!found) {
        std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_BOOTLOADER);
    }
    return {found, dev};
}

// Returns all devices which aren't already booted
std::vector<DeviceInfo> DeviceBootloader::getAllAvailableDevices() {
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices();
    for(const auto& d : connectedDevices) {
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

std::vector<uint8_t> DeviceBootloader::createDepthaiApplicationPackage(Pipeline& pipeline, std::string pathToCmd) {
    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    OpenVINO::Version version;
    pipeline.serialize(schema, assets, assetStorage, version);

    // Prepare device firmware
    std::vector<uint8_t> deviceFirmware;
    if(pathToCmd != "") {
        std::ifstream fwStream(pathToCmd, std::ios::binary);
        if(!fwStream.is_open()) throw std::runtime_error("Cannot create application package, device firmware at path: " + pathToCmd + " doesn't exist");
        deviceFirmware = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        // TODO(themarpe) - specify OpenVINO version
        deviceFirmware = Resources::getInstance().getDeviceFirmware(false, version);
    }

    // Create msgpacks
    std::vector<uint8_t> pipelineBinary, assetsBinary;
    {
        nlohmann::json j = schema;
        pipelineBinary = nlohmann::json::to_msgpack(j);
    }
    {
        nlohmann::json j = assets;
        assetsBinary = nlohmann::json::to_msgpack(j);
    }

    // Prepare SBR structure
    SBR sbr = {};
    SBR_SECTION* fwSection = &sbr.sections[0];
    SBR_SECTION* pipelineSection = &sbr.sections[1];
    SBR_SECTION* assetsSection = &sbr.sections[2];
    SBR_SECTION* assetStorageSection = &sbr.sections[3];
    SBR_SECTION* lastSection = assetStorageSection;

    // Alignup for easier updating
    auto getSectionAlignedOffset = [](long S) {
        constexpr long SECTION_ALIGNMENT_SIZE = 1 * 1024 * 1024;  // 1MiB for easier updating
        return ((((S) + (SECTION_ALIGNMENT_SIZE)-1)) & ~((SECTION_ALIGNMENT_SIZE)-1));
    };

    // First section, MVCMD, name '__firmware'
    sbr_section_set_name(fwSection, "__firmware");
    sbr_section_set_bootable(fwSection, true);
    sbr_section_set_size(fwSection, static_cast<uint32_t>(deviceFirmware.size()));
    sbr_section_set_checksum(fwSection, sbr_compute_checksum(deviceFirmware.data(), static_cast<uint32_t>(deviceFirmware.size())));
    sbr_section_set_offset(fwSection, SBR_RAW_SIZE);

    // Second section, pipeline schema, name 'pipeline'
    sbr_section_set_name(pipelineSection, "pipeline");
    sbr_section_set_size(pipelineSection, static_cast<uint32_t>(pipelineBinary.size()));
    sbr_section_set_checksum(pipelineSection, sbr_compute_checksum(pipelineBinary.data(), static_cast<uint32_t>(pipelineBinary.size())));
    sbr_section_set_offset(pipelineSection, getSectionAlignedOffset(fwSection->offset + fwSection->size));

    // Third section, assets map, name 'assets'
    sbr_section_set_name(assetsSection, "assets");
    sbr_section_set_size(assetsSection, static_cast<uint32_t>(assetsBinary.size()));
    sbr_section_set_checksum(assetsSection, sbr_compute_checksum(assetsBinary.data(), static_cast<uint32_t>(assetsBinary.size())));
    sbr_section_set_offset(assetsSection, getSectionAlignedOffset(pipelineSection->offset + pipelineSection->size));

    // Fourth section, asset storage, name 'asset_storage'
    sbr_section_set_name(assetStorageSection, "asset_storage");
    sbr_section_set_size(assetStorageSection, static_cast<uint32_t>(assetStorage.size()));
    sbr_section_set_checksum(assetStorageSection, sbr_compute_checksum(assetStorage.data(), static_cast<uint32_t>(assetStorage.size())));
    sbr_section_set_offset(assetStorageSection, getSectionAlignedOffset(assetsSection->offset + assetsSection->size));

    // TODO(themarpe) - Add additional sections (Pipeline nodes will be able to use sections)

    // Create a vector to hold whole dap package
    std::vector<uint8_t> fwPackage;
    fwPackage.resize(lastSection->offset + lastSection->size);

    // Serialize SBR
    sbr_serialize(&sbr, fwPackage.data(), static_cast<uint32_t>(fwPackage.size()));

    // Write to fwPackage
    for(unsigned i = 0; i < deviceFirmware.size(); i++) fwPackage[fwSection->offset + i] = deviceFirmware[i];
    for(unsigned i = 0; i < pipelineBinary.size(); i++) fwPackage[pipelineSection->offset + i] = pipelineBinary[i];
    for(unsigned i = 0; i < assetsBinary.size(); i++) fwPackage[assetsSection->offset + i] = assetsBinary[i];
    for(unsigned i = 0; i < assetStorage.size(); i++) fwPackage[assetStorageSection->offset + i] = assetStorage[i];

    return fwPackage;
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init(true, "", tl::nullopt);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, Type type) : deviceInfo(devInfo) {
    init(true, "", type);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, const char* pathToBootloader) : deviceInfo(devInfo) {
    init(false, std::string(pathToBootloader), tl::nullopt);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, const std::string& pathToBootloader) : deviceInfo(devInfo) {
    init(false, pathToBootloader, tl::nullopt);
}

void DeviceBootloader::init(bool embeddedMvcmd, const std::string& pathToMvcmd, tl::optional<bootloader::Type> type) {
    stream = nullptr;

    bootloaderType = type.value_or(DEFAULT_TYPE);

    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED) {
        // Unbooted device found, boot to BOOTLOADER and connect with XLinkConnection constructor
        if(embeddedMvcmd) {
            connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(bootloaderType), X_LINK_BOOTLOADER);
        } else {
            connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
        }

        // Device wasn't already in bootloader, that means that embedded bootloader is booted
        isEmbedded = true;
    } else if(deviceInfo.state == X_LINK_BOOTLOADER) {
        // In this case boot specified bootloader only if current bootloader isn't of correct type
        // Check version first, if >= 0.0.12 then check type and then either bootmemory to correct BL or continue as is

        // Device already in bootloader mode.
        // Connect without booting
        connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_BOOTLOADER);

        // If type is specified, try to boot into that BL type
        stream = std::unique_ptr<XLinkStream>(new XLinkStream(*connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE));

        // Send request for bootloader version
        if(!sendBootloaderRequest(stream->getStreamId(), bootloader::request::GetBootloaderVersion{})) {
            throw std::runtime_error("Error trying to connect to device");
        }

        // Receive response
        bootloader::response::BootloaderVersion ver;
        if(!receiveBootloaderResponse(stream->getStreamId(), ver)) throw std::runtime_error("Error trying to connect to device");
        DeviceBootloader::Version version(ver.major, ver.minor, ver.patch);

        // If version is adequite
        if(version >= Version(0, 0, 12)) {
            // Send request for bootloader type
            if(!sendBootloaderRequest(stream->getStreamId(), bootloader::request::GetBootloaderType{})) {
                throw std::runtime_error("Error trying to connect to device");
            }
            // Receive response
            bootloader::response::BootloaderType runningBootloaderType;
            if(!receiveBootloaderResponse(stream->getStreamId(), runningBootloaderType)) throw std::runtime_error("Error trying to connect to device");

            // Modify actual bootloader type
            bootloaderType = runningBootloaderType.type;

            // Boot memory correct type of BL
            if(type && runningBootloaderType.type != *type) {
                // prepare watchdog thread, which will keep device alive
                std::atomic<bool> wdRunning{true};
                std::thread wd = std::thread([&]() {
                    // prepare watchdog thread
                    XLinkStream stream(*connection, bootloader::XLINK_CHANNEL_WATCHDOG, 64);
                    std::vector<uint8_t> watchdogKeepalive = {0, 0, 0, 0};
                    while(wdRunning) {
                        try {
                            stream.write(watchdogKeepalive);
                        } catch(const std::exception&) {
                            break;
                        }
                        // Ping with a period half of that of the watchdog timeout
                        std::this_thread::sleep_for(bootloader::XLINK_WATCHDOG_TIMEOUT / 2);
                    }
                });

                // Send request to boot firmware directly from bootloader
                dai::bootloader::request::BootMemory bootMemory;
                auto binary = getEmbeddedBootloaderBinary(*type);
                bootMemory.totalSize = static_cast<uint32_t>(binary.size());
                bootMemory.numPackets = ((static_cast<uint32_t>(binary.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
                if(!sendBootloaderRequest(stream->getStreamId(), bootMemory)) {
                    throw std::runtime_error("Error trying to connect to device");
                }

                // After that send numPackets of data
                stream->writeSplit(binary.data(), binary.size(), bootloader::XLINK_STREAM_MAX_SIZE);

                // Stop watchdog
                wdRunning = false;
                wd.join();

                // Dummy read, until link falls down and it returns an error code
                streamPacketDesc_t* pPacket;
                XLinkReadData(stream->getStreamId(), &pPacket);

                // Now connect
                connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_BOOTLOADER);

                isEmbedded = false;
            } else {
                isEmbedded = true;
            }

        } else {
            if(type && *type != Type::USB) {
                // Send request to jump to USB bootloader
                // Boot into USB ROM BOOTLOADER NOW
                if(!sendBootloaderRequest(stream->getStreamId(), dai::bootloader::request::UsbRomBoot{})) {
                    throw std::runtime_error("Error trying to connect to device");
                }

                // Dummy read, until link falls down and it returns an error code
                streamPacketDesc_t* pPacket;
                XLinkReadData(stream->getStreamId(), &pPacket);

                // Unbooted device found, boot to BOOTLOADER and connect with XLinkConnection constructor
                if(embeddedMvcmd) {
                    connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(*type), X_LINK_BOOTLOADER);
                } else {
                    connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
                }

                bootloaderType = *type;

                // Device wasn't already in bootloader, that means that embedded bootloader is booted
                isEmbedded = true;

            } else {
                bootloaderType = dai::bootloader::Type::USB;
                // Device was already in bootloader, that means that embedded isn't running
                isEmbedded = false;
            }
        }

    } else {
        throw std::runtime_error("Device not in UNBOOTED or BOOTLOADER state");
    }

    deviceInfo.state = X_LINK_BOOTLOADER;

    // prepare watchdog thread, which will keep device alive
    watchdogThread = std::thread([this]() {
        // prepare watchdog thread
        XLinkStream stream(*connection, bootloader::XLINK_CHANNEL_WATCHDOG, 64);

        std::shared_ptr<XLinkConnection> conn = this->connection;
        std::vector<uint8_t> watchdogKeepalive = {0, 0, 0, 0};
        std::vector<uint8_t> reset = {1, 0, 0, 0};
        while(watchdogRunning) {
            try {
                stream.write(watchdogKeepalive);
            } catch(const std::exception&) {
                break;
            }
            // Ping with a period half of that of the watchdog timeout
            std::this_thread::sleep_for(bootloader::XLINK_WATCHDOG_TIMEOUT / 2);
        }

        try {
            // Send reset request
            stream.write(reset);
            // Dummy read (wait till link falls down)
            stream.readRaw();
        } catch(const std::exception&) {
        }  // ignore

        // Sleep a bit, so device isn't available anymore
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    });

    // prepare bootloader stream
    if(stream == nullptr) {
        stream = std::unique_ptr<XLinkStream>(new XLinkStream(*connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE));
    }
}

void DeviceBootloader::close() {
    // Only allow to close once
    if(closed.exchange(true)) return;

    using namespace std::chrono;
    auto t1 = steady_clock::now();
    spdlog::debug("DeviceBootloader about to be closed...");

    // Close connection first (so queues unblock)
    connection->close();
    connection = nullptr;

    // Stop watchdog
    watchdogRunning = false;

    // Stop watchdog first (this resets and waits for link to fall down)
    if(watchdogThread.joinable()) watchdogThread.join();

    // Close stream
    stream = nullptr;

    spdlog::debug("DeviceBootloader closed, {}", duration_cast<milliseconds>(steady_clock::now() - t1).count());
}

bool DeviceBootloader::isClosed() const {
    return closed || !watchdogRunning;
}

void DeviceBootloader::checkClosed() const {
    if(isClosed()) throw std::invalid_argument("DeviceBootloader already closed or disconnected");
}

DeviceBootloader::~DeviceBootloader() {
    close();
}

DeviceBootloader::Version DeviceBootloader::getEmbeddedBootloaderVersion() {
    return DeviceBootloader::Version(DEPTHAI_BOOTLOADER_VERSION);
}

DeviceBootloader::Version DeviceBootloader::getVersion() {
    streamId_t streamId = stream->getStreamId();

    // Send request to jump to USB bootloader
    if(!sendBootloaderRequest(streamId, bootloader::request::GetBootloaderVersion{})) {
        throw std::runtime_error("Couldn't get bootloader version");
    }

    // Receive response
    dai::bootloader::response::BootloaderVersion ver;
    if(!receiveBootloaderResponse(streamId, ver)) {
        throw std::runtime_error("Couldn't get bootloader version");
    }

    // Create bootloader::Version object and return
    return DeviceBootloader::Version(ver.major, ver.minor, ver.patch);
}

std::tuple<bool, std::string> DeviceBootloader::flash(std::function<void(float)> progressCb, Pipeline& pipeline) {
    return flashDepthaiApplicationPackage(progressCb, createDepthaiApplicationPackage(pipeline));
}

void DeviceBootloader::saveDepthaiApplicationPackage(std::string path, Pipeline& pipeline, std::string pathToCmd) {
    auto dap = createDepthaiApplicationPackage(pipeline, pathToCmd);
    std::ofstream outfile(path, std::ios::binary);
    outfile.write(reinterpret_cast<const char*>(dap.data()), dap.size());
}

std::tuple<bool, std::string> DeviceBootloader::flashDepthaiApplicationPackage(std::function<void(float)> progressCb, std::vector<uint8_t> package) {
    streamId_t streamId = stream->getStreamId();

    // Bug in NETWORK bootloader in version 0.0.12 < 0.1.0 - flashing can cause a soft brick
    auto version = getVersion();
    if(bootloaderType == Type::NETWORK && version < Version(0, 1, 0)) {
        throw std::invalid_argument("Network bootloader requires version 0.1.0 or higher to flash applications. Current version: " + version.toString());
    }

    // send request to FLASH BOOTLOADER
    dai::bootloader::request::UpdateFlash updateFlash;
    updateFlash.storage = dai::bootloader::request::UpdateFlash::SBR;
    updateFlash.totalSize = static_cast<uint32_t>(package.size());
    updateFlash.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendBootloaderRequest(streamId, updateFlash)) return {false, "Couldn't send bootloader flash request"};

    // After that send numPackets of data
    stream->writeSplit(package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return {false, "Couldn't receive bootloader response"};

        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
        } else if(parseBootloaderResponse(data, result)) {
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(std::function<void(float)> progressCb, std::string path) {
    return flashBootloader(Memory::FLASH, bootloaderType, progressCb, path);
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(Memory memory, Type type, std::function<void(float)> progressCb, std::string path) {
    // Only flash memory is supported for now
    if(memory != Memory::FLASH) {
        throw std::invalid_argument("Only FLASH memory is supported for now");
    }
    if(bootloaderType != type && getVersion() < Version(0, 0, 12)) {
        std::runtime_error("Current bootloader version doesn't support flashing different type of bootloader");
    }

    std::vector<uint8_t> package;
    if(path != "") {
        std::ifstream fwStream(path, std::ios::binary);
        if(!fwStream.is_open()) throw std::runtime_error("Cannot flash bootloader, binary at path: " + path + " doesn't exist");
        package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        package = getEmbeddedBootloaderBinary(type);
    }

    // get streamId
    streamId_t streamId = stream->getStreamId();

    // If booted and desired bootloader types don't match
    // Use UpdateFlashEx2 instead to properly flash
    if(bootloaderType == type) {
        // Old command

        // send request to FLASH BOOTLOADER
        dai::bootloader::request::UpdateFlash updateFlash;
        updateFlash.storage = dai::bootloader::request::UpdateFlash::BOOTLOADER;
        updateFlash.totalSize = static_cast<uint32_t>(package.size());
        updateFlash.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
        if(!sendBootloaderRequest(streamId, updateFlash)) return {false, "Couldn't send bootloader flash request"};

    } else {
        // send request to FLASH BOOTLOADER
        dai::bootloader::request::UpdateFlashEx2 updateFlashEx2;
        updateFlashEx2.memory = memory;
        updateFlashEx2.offset = dai::bootloader::getStructure(type).offset.at(Section::BOOTLOADER);
        updateFlashEx2.totalSize = static_cast<uint32_t>(package.size());
        updateFlashEx2.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
        if(!sendBootloaderRequest(streamId, updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};
    }

    // After that send numPackets of data
    stream->writeSplit(package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return {false, "Couldn't receive bootloader response"};

        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
            // if flash complete response arrived, break from while loop
        } else if(parseBootloaderResponse(data, result)) {
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

/* TODO(themarpe)
std::tuple<bool, std::string> DeviceBootloader::flashCustom(Memory memory, uint32_t offset, std::function<void(float)> progressCb, std::vector<uint8_t> data) {
    // Only flash memory is supported for now
    if(memory != Memory::FLASH) {
        throw std::invalid_argument("Only FLASH memory is supported for now");
    }
    if(getVersion() < Version(0, 0, 12)) {
        std::runtime_error("Current bootloader version doesn't support custom flashing");
    }

    // get streamId
    streamId_t streamId = stream->getStreamId();

    // send request to FLASH BOOTLOADER
    dai::bootloader::request::UpdateFlashEx2 updateFlashEx2;
    updateFlashEx2.memory = memory;
    updateFlashEx2.offset = offset;
    updateFlashEx2.totalSize = static_cast<uint32_t>(data.size());
    updateFlashEx2.numPackets = ((static_cast<uint32_t>(data.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendBootloaderRequest(streamId, updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};

    // After that send numPackets of data
    stream->writeSplit(data.data(), data.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return {false, "Couldn't receive bootloader response"};

        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
            // if flash complete response arrived, break from while loop
        } else if(parseBootloaderResponse(data, result)) {
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}
*/

bool DeviceBootloader::isEmbeddedVersion() const {
    return isEmbedded;
}

std::vector<std::uint8_t> DeviceBootloader::getEmbeddedBootloaderBinary(Type type) {
    return Resources::getInstance().getBootloaderFirmware(type);
}

DeviceBootloader::Version::Version(const std::string& v) : versionMajor(0), versionMinor(0), versionPatch(0) {
    // Parse string
    if(std::sscanf(v.c_str(), "%u.%u.%u", &versionMajor, &versionMinor, &versionPatch) != 3) throw std::runtime_error("Cannot parse version: " + v);
}

DeviceBootloader::Version::Version(unsigned vmajor, unsigned vminor, unsigned vpatch) {
    this->versionMajor = vmajor;
    this->versionMinor = vminor;
    this->versionPatch = vpatch;
}

bool DeviceBootloader::Version::operator==(const Version& other) const {
    if(versionMajor == other.versionMajor && versionMinor == other.versionMinor && versionPatch == other.versionPatch) return true;
    return false;
}

bool DeviceBootloader::Version::operator<(const Version& other) const {
    if(versionMajor < other.versionMajor) {
        return true;
    } else {
        if(versionMinor < other.versionMinor) {
            return true;
        } else {
            if(versionPatch < other.versionPatch) {
                return true;
            }
        }
    }
    return false;
}

std::string DeviceBootloader::Version::toString() const {
    return std::to_string(versionMajor) + "." + std::to_string(versionMinor) + "." + std::to_string(versionPatch);
}

}  // namespace dai
