#include "device/DeviceBootloader.hpp"

// std
#include <fstream>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/SBR.h"
#include "depthai-bootloader-shared/Structure.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/pipeline/Assets.hpp"
#include "depthai-shared/utility/Serialization.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "device/Device.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/Platform.hpp"
#include "utility/Resources.hpp"
#include "utility/spdlog-fmt.hpp"

// libraries
#include "spdlog/fmt/chrono.h"
#include "spdlog/spdlog.h"
#include "zlib.h"

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {

// Using
namespace Request = bootloader::request;
namespace Response = bootloader::response;

// constants
constexpr const DeviceBootloader::Type DeviceBootloader::DEFAULT_TYPE;

// static api

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceBootloader::getFirstAvailableDevice() {
    // Get all connected devices
    auto devices = XLinkConnection::getAllConnectedDevices();
    // Search order - first unbooted, then bootloader and last flash booted
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED}) {
        for(const auto& device : devices) {
            if(device.state == searchState) {
                return {true, device};
            }
        }
    }
    return {false, {}};
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

std::vector<uint8_t> DeviceBootloader::createDepthaiApplicationPackage(const Pipeline& pipeline, const dai::Path& pathToCmd, bool compress) {
    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    // Get openvino version
    OpenVINO::Version version = pipeline.getOpenVINOVersion();

    // Prepare device firmware
    std::vector<uint8_t> deviceFirmware;
    if(!pathToCmd.empty()) {
        std::ifstream fwStream(pathToCmd, std::ios::binary);
        if(!fwStream.is_open())
            throw std::runtime_error(fmt::format("Cannot create application package, device firmware at path: {} doesn't exist", pathToCmd));
        deviceFirmware = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    } else {
        // TODO(themarpe) - specify OpenVINO version
        deviceFirmware = Resources::getInstance().getDeviceFirmware(false, version);
    }
    if(deviceFirmware.empty()) {
        throw std::runtime_error("Error getting device firmware");
    }

    // Serialize data
    std::vector<uint8_t> pipelineBinary, assetsBinary;
    utility::serialize(schema, pipelineBinary);
    utility::serialize(assets, assetsBinary);

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

    // Should compress firmware?
    if(compress) {
        using namespace std::chrono;

        auto t1 = steady_clock::now();
        auto compressBufferSize = compressBound(static_cast<decltype(compressBound(1))>(deviceFirmware.size()));
        std::vector<uint8_t> compressBuffer(compressBufferSize);
        // Chosen impirically
        constexpr int COMPRESSION_LEVEL = 9;
        if(compress2(compressBuffer.data(),
                     &compressBufferSize,
                     deviceFirmware.data(),
                     static_cast<decltype(compressBufferSize)>(deviceFirmware.size()),
                     COMPRESSION_LEVEL)
           != Z_OK) {
            throw std::runtime_error("Error while compressing device firmware\n");
        }

        // Resize output buffer
        compressBuffer.resize(compressBufferSize);

        // Set the compressed firmware
        auto prevSize = deviceFirmware.size();
        deviceFirmware = std::move(compressBuffer);

        auto diff = duration_cast<milliseconds>(steady_clock::now() - t1);
        spdlog::debug("Compressed firmware for Dephai Application Package. Took {}, size reduced from {:.2f}MiB to {:.2f}MiB",
                      diff,
                      prevSize / (1024.0f * 1024.0f),
                      deviceFirmware.size() / (1024.0f * 1024.0f));
    }

    // First section, MVCMD, name '__firmware'
    sbr_section_set_name(fwSection, "__firmware");
    sbr_section_set_bootable(fwSection, true);
    sbr_section_set_size(fwSection, static_cast<uint32_t>(deviceFirmware.size()));
    sbr_section_set_checksum(fwSection, sbr_compute_checksum(deviceFirmware.data(), static_cast<uint32_t>(deviceFirmware.size())));
    sbr_section_set_offset(fwSection, SBR_RAW_SIZE);
    // Ignore checksum to allow faster booting (images are verified after flashing, low risk)
    sbr_section_set_ignore_checksum(fwSection, true);
    // Set compression flags
    if(compress) {
        sbr_section_set_compression(fwSection, SBR_COMPRESSION_ZLIB);
    } else {
        sbr_section_set_compression(fwSection, SBR_NO_COMPRESSION);
    }

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
    for(std::size_t i = 0; i < deviceFirmware.size(); i++) fwPackage[fwSection->offset + i] = deviceFirmware[i];
    for(std::size_t i = 0; i < pipelineBinary.size(); i++) fwPackage[pipelineSection->offset + i] = pipelineBinary[i];
    for(std::size_t i = 0; i < assetsBinary.size(); i++) fwPackage[assetsSection->offset + i] = assetsBinary[i];
    for(std::size_t i = 0; i < assetStorage.size(); i++) fwPackage[assetStorageSection->offset + i] = assetStorage[i];

    return fwPackage;
}

std::vector<uint8_t> DeviceBootloader::createDepthaiApplicationPackage(const Pipeline& pipeline, bool compress) {
    return createDepthaiApplicationPackage(pipeline, {}, compress);
}

void DeviceBootloader::saveDepthaiApplicationPackage(const dai::Path& path, const Pipeline& pipeline, const dai::Path& pathToCmd, bool compress) {
    auto dap = createDepthaiApplicationPackage(pipeline, pathToCmd, compress);
    std::ofstream outfile(path, std::ios::binary);
    outfile.write(reinterpret_cast<const char*>(dap.data()), dap.size());
}

void DeviceBootloader::saveDepthaiApplicationPackage(const dai::Path& path, const Pipeline& pipeline, bool compress) {
    auto dap = createDepthaiApplicationPackage(pipeline, compress);
    std::ofstream outfile(path, std::ios::binary);
    outfile.write(reinterpret_cast<const char*>(dap.data()), dap.size());
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init(true, {}, tl::nullopt, false);
}

template <>
DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, bool allowFlashingBootloader) : deviceInfo(devInfo) {
    init(true, {}, tl::nullopt, allowFlashingBootloader);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, Type type, bool allowFlashingBootloader) : deviceInfo(devInfo) {
    init(true, {}, type, allowFlashingBootloader);
}

DeviceBootloader::DeviceBootloader(const DeviceInfo& devInfo, const dai::Path& pathToBootloader, bool allowFlashingBootloader) : deviceInfo(devInfo) {
    init(false, pathToBootloader, tl::nullopt, allowFlashingBootloader);
}

void DeviceBootloader::init(bool embeddedMvcmd, const dai::Path& pathToMvcmd, tl::optional<bootloader::Type> type, bool allowBlFlash) {
    stream = nullptr;
    allowFlashingBootloader = allowBlFlash;

    bootloaderType = type.value_or(DEFAULT_TYPE);

    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED) {
        // Unbooted device found, boot to BOOTLOADER and connect with XLinkConnection constructor
        if(embeddedMvcmd) {
            connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(bootloaderType), X_LINK_BOOTLOADER);
        } else {
            connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
        }

        // prepare bootloader stream
        stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

        // Retrieve bootloader version
        version = requestVersion();

        // Device wasn't already in bootloader, that means that embedded bootloader is booted
        isEmbedded = true;
    } else if(deviceInfo.state == X_LINK_BOOTLOADER || deviceInfo.state == X_LINK_FLASH_BOOTED) {
        // If device is in flash booted state, reset to bootloader and then continue by booting appropriate FW
        if(deviceInfo.state == X_LINK_FLASH_BOOTED) {
            // Boot bootloader and set current deviceInfo to new device state
            deviceInfo = XLinkConnection::bootBootloader(deviceInfo);
        }

        // In this case boot specified bootloader only if current bootloader isn't of correct type
        // Check version first, if >= 0.0.12 then check type and then either bootmemory to correct BL or continue as is

        // Device already in bootloader mode.
        // Connect without booting
        connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_BOOTLOADER);

        // If type is specified, try to boot into that BL type
        stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

        // Retrieve bootloader version
        flashedVersion = version = requestVersion();
        if(version >= Version(0, 0, 12)) {
            // If version is adequate, do an in memory boot.

            // Send request for bootloader type
            if(!sendRequest(Request::GetBootloaderType{})) {
                throw std::runtime_error("Error trying to connect to device");
            }
            // Receive response
            Response::BootloaderType runningBootloaderType;
            if(!receiveResponse(runningBootloaderType)) throw std::runtime_error("Error trying to connect to device");

            // Modify actual bootloader type
            bootloaderType = runningBootloaderType.type;

            Type desiredBootloaderType = type.value_or(bootloaderType);

            // If not correct type OR if allowFlashingBootloader is set, then boot internal (latest) bootloader of correct type
            if((desiredBootloaderType != bootloaderType) || allowFlashingBootloader) {
                // prepare watchdog thread, which will keep device alive
                std::atomic<bool> wdRunning{true};
                std::thread wd = std::thread([&]() {
                    // prepare watchdog thread
                    try {
                        // constructor can throw in rare+quick start/stop scenarios because
                        // the connection is close() eg. by DeviceBootloader::close()
                        XLinkStream stream(connection, bootloader::XLINK_CHANNEL_WATCHDOG, 64);
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
                    } catch(const std::exception&) {
                        // ignore, probably invalid connection or stream
                    }
                });

                // Send request to boot firmware directly from bootloader
                Request::BootMemory bootMemory;
                auto binary = getEmbeddedBootloaderBinary(desiredBootloaderType);
                bootMemory.totalSize = static_cast<uint32_t>(binary.size());
                bootMemory.numPackets = ((static_cast<uint32_t>(binary.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
                if(!sendRequest(bootMemory)) {
                    throw std::runtime_error("Error trying to connect to device");
                }

                // After that send numPackets of data
                stream->writeSplit(binary.data(), binary.size(), bootloader::XLINK_STREAM_MAX_SIZE);
                // Close existing stream first
                stream = nullptr;
                // Stop watchdog
                wdRunning = false;
                wd.join();
                // Close connection
                connection->close();

                // Now reconnect
                connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_BOOTLOADER);

                // prepare new bootloader stream
                stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

                // Retrieve bootloader version
                version = requestVersion();

                // The type of bootloader is now 'desiredBootloaderType'
                bootloaderType = desiredBootloaderType;

                // Embedded bootloader was used to boot, set to true
                isEmbedded = true;
            } else {
                // Just connected to existing bootloader on device. Set embedded to false
                isEmbedded = false;
            }

        } else {
            // If version isn't adequate to do an in memory boot - do regular Bootloader -> USB ROM -> Boot transition.
            Type desiredBootloaderType = type.value_or(Type::USB);

            // If not correct type OR if allowFlashingBootloader is set, then boot internal (latest) bootloader of correct type
            if((desiredBootloaderType != Type::USB) || allowFlashingBootloader) {
                // Send request to jump to USB bootloader and wait for link down
                bootUsbRomBootloader();
                // Close existing stream first
                stream = nullptr;
                // Close connection
                connection->close();
                // Unbooted state at this point
                deviceInfo.state = X_LINK_UNBOOTED;

                // Now reconnect
                // Unbooted device found, boot to BOOTLOADER and connect with XLinkConnection constructor
                if(embeddedMvcmd) {
                    connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(desiredBootloaderType), X_LINK_BOOTLOADER);
                } else {
                    connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
                }

                // prepare bootloader stream
                stream = std::make_unique<XLinkStream>(connection, bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

                // Retrieve bootloader version
                version = requestVersion();

                // The type of bootloader is now 'desiredBootloaderType'
                bootloaderType = desiredBootloaderType;

                // Embedded bootloader was used to boot, set to true
                isEmbedded = true;

            } else {
                bootloaderType = dai::bootloader::Type::USB;

                // Just connected to existing bootloader on device. Set embedded to false
                isEmbedded = false;
            }
        }

    } else {
        throw std::runtime_error("Device not in UNBOOTED, BOOTLOADER or FLASH_BOOTED state");
    }

    deviceInfo.state = X_LINK_BOOTLOADER;

    // prepare watchdog thread, which will keep device alive
    watchdogThread = std::thread([this]() {
        try {
            // constructor often throws in quick start/stop scenarios because
            // the connection is close()...usually by DeviceBootloader::close()
            XLinkStream stream(connection, bootloader::XLINK_CHANNEL_WATCHDOG, 64);
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
                const auto dummy = stream.readMove();
            } catch(const std::exception&) {
                // ignore
            }
        } catch(const std::exception&) {
            // ignore
        }

        // Sleep a bit, so device isn't available anymore
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    });
}

void DeviceBootloader::close() {
    // Only allow to close once
    if(closed.exchange(true)) return;

    using namespace std::chrono;
    auto t1 = steady_clock::now();
    spdlog::debug("DeviceBootloader about to be closed...");

    // Close connection first; causes Xlink internal calls to unblock semaphore waits and
    // return error codes, which then allows queues to unblock
    // always manage ownership because other threads (e.g. watchdog) are running and need to
    // keep the shared_ptr valid (even if closed). Otherwise leads to using null pointers,
    // invalid memory, etc. which hard crashes main app
    connection->close();

    // Stop watchdog
    watchdogRunning = false;

    // Stop watchdog first (this resets and waits for link to fall down)
    if(watchdogThread.joinable()) watchdogThread.join();

    // Close stream
    // BUGBUG investigate ownership; can another thread accessing this at the same time?
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

DeviceBootloader::Version DeviceBootloader::getVersion() const {
    return version;
}

DeviceBootloader::Version DeviceBootloader::getFlashedVersion() const {
    return flashedVersion;
}

DeviceBootloader::Version DeviceBootloader::requestVersion() {
    // Send request to retrieve bootloader version
    if(!sendRequest(Request::GetBootloaderVersion{})) {
        throw std::runtime_error("Couldn't get bootloader version");
    }

    // Receive response
    Response::BootloaderVersion ver;
    if(!receiveResponse(ver)) {
        throw std::runtime_error("Couldn't parse version response");
    }

    Version blVersion(ver.major, ver.minor, ver.patch);

    if(blVersion >= Version(Request::GetBootloaderCommit::VERSION)) {
        // Send request to retrieve bootloader commit (skip version check)
        Request::GetBootloaderCommit request{};
        stream->write((uint8_t*)&request, sizeof(request));

        // Receive response
        Response::BootloaderCommit commit;
        if(!receiveResponse(commit)) {
            throw std::runtime_error("Couldn't get bootloader commit");
        }

        // Workaround for older bootloader versions not tagged, default to 0.0.4
        if(ver.major == 0 && ver.minor == 0 && ver.patch == 0) {
            ver.patch = 4;
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

std::tuple<bool, std::string> DeviceBootloader::flash(std::function<void(float)> progressCb, const Pipeline& pipeline, bool compress) {
    return flashDepthaiApplicationPackage(progressCb, createDepthaiApplicationPackage(pipeline, compress));
}

std::tuple<bool, std::string> DeviceBootloader::flash(const Pipeline& pipeline, bool compress) {
    return flashDepthaiApplicationPackage(createDepthaiApplicationPackage(pipeline, compress));
}

std::tuple<bool, std::string> DeviceBootloader::flashDepthaiApplicationPackage(std::function<void(float)> progressCb, std::vector<uint8_t> package) {
    // Bug in NETWORK bootloader in version 0.0.12 < 0.0.14 - flashing can cause a soft brick
    if(bootloaderType == Type::NETWORK && version < Version(0, 0, 14)) {
        throw std::invalid_argument("Network bootloader requires version 0.0.14 or higher to flash applications. Current version: " + version.toString());
    }

    // Older versions have a limitation on the maximum firmware size that can be booted from NOR flash,
    // due to 24-bit addressing. Check only if a bootloader is flashed (version > 0.0.0)
    if(flashedVersion < Version(0, 0, 14) && !(flashedVersion < Version(0, 0, 1))) {
        // TODO should check firmware section only, and make sure it's not compressed either
        constexpr uint32_t MAX_24BIT_SIZE = 1 << 24;
        uint32_t dapLastOffset = bootloader::getStructure(bootloaderType).offset.at(bootloader::Section::APPLICATION) + package.size();
        if(dapLastOffset > MAX_24BIT_SIZE) {
            throw std::invalid_argument("Application to flash is too large, unsupported by current flashed bootloader " + flashedVersion.toString()
                                        + ". Please upgrade the bootloader");
        }
    }

    // send request to FLASH BOOTLOADER
    Request::UpdateFlash updateFlash;
    updateFlash.storage = Request::UpdateFlash::SBR;
    updateFlash.totalSize = static_cast<uint32_t>(package.size());
    updateFlash.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendRequest(updateFlash)) return {false, "Couldn't send bootloader flash request"};

    // After that send numPackets of data
    stream->writeSplit(package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    do {
        std::vector<uint8_t> data;
        if(!receiveResponseData(data)) return {false, "Couldn't receive bootloader response"};

        Response::FlashStatusUpdate update;
        if(parseResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
        } else if(parseResponse(data, result)) {
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashDepthaiApplicationPackage(std::vector<uint8_t> package) {
    return flashDepthaiApplicationPackage(nullptr, package);
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(std::function<void(float)> progressCb, const dai::Path& path) {
    return flashBootloader(Memory::FLASH, bootloaderType, progressCb, path);
}

std::tuple<bool, std::string> DeviceBootloader::flashBootloader(Memory memory, Type type, std::function<void(float)> progressCb, const dai::Path& path) {
    // Check if 'allowFlashingBootloader' is set to true.
    if(!allowFlashingBootloader) {
        throw std::invalid_argument("DeviceBootloader wasn't initialized to allow flashing bootloader. Set 'allowFlashingBootloader' in constructor");
    }

    // Only flash memory is supported for now
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

    // If booted and desired bootloader types don't match
    // Use UpdateFlashEx2 instead to properly flash
    if(bootloaderType == type) {
        // Old command

        // send request to FLASH BOOTLOADER
        Request::UpdateFlash updateFlash;
        updateFlash.storage = Request::UpdateFlash::BOOTLOADER;
        updateFlash.totalSize = static_cast<uint32_t>(package.size());
        updateFlash.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
        if(!sendRequest(updateFlash)) return {false, "Couldn't send bootloader flash request"};

    } else {
        // send request to FLASH BOOTLOADER
        Request::UpdateFlashEx2 updateFlashEx2;
        updateFlashEx2.memory = memory;
        updateFlashEx2.offset = dai::bootloader::getStructure(type).offset.at(Section::BOOTLOADER);
        updateFlashEx2.totalSize = static_cast<uint32_t>(package.size());
        updateFlashEx2.numPackets = ((static_cast<uint32_t>(package.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
        if(!sendRequest(updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};
    }

    // After that send numPackets of data
    stream->writeSplit(package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    do {
        std::vector<uint8_t> data;
        if(!receiveResponseData(data)) return {false, "Couldn't receive bootloader response"};

        Response::FlashStatusUpdate update;
        if(parseResponse(data, update)) {
            // if progress callback is set
            if(progressCb != nullptr) {
                progressCb(update.progress);
            }
            // if flash complete response arrived, break from while loop
        } else if(parseResponse(data, result)) {
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
    Request::UpdateFlashEx2 updateFlashEx2;
    updateFlashEx2.memory = memory;
    updateFlashEx2.offset = offset;
    updateFlashEx2.totalSize = static_cast<uint32_t>(data.size());
    updateFlashEx2.numPackets = ((static_cast<uint32_t>(data.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendRequest(updateFlashEx2)) return {false, "Couldn't send bootloader flash request"};

    // After that send numPackets of data
    stream->writeSplit(data.data(), data.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    do {
        std::vector<uint8_t> data;
        if(!receiveResponseData(data)) return {false, "Couldn't receive bootloader response"};

        Response::FlashStatusUpdate update;
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

nlohmann::json DeviceBootloader::readConfigData(Memory memory, Type type) {
    // Send request to GET_BOOTLOADER_CONFIG
    Request::GetBootloaderConfig getConfigReq;
    getConfigReq.memory = memory;

    if(type != Type::AUTO) {
        const auto confStructure = bootloader::getStructure(type);
        getConfigReq.offset = confStructure.offset.at(bootloader::Section::BOOTLOADER_CONFIG);
        getConfigReq.maxSize = confStructure.size.at(bootloader::Section::BOOTLOADER_CONFIG);
    } else {
        // leaves as default values, which correspond to AUTO
    }

    if(!sendRequest(getConfigReq)) return {false, "Couldn't send request to get configuration data"};

    // Get response
    Response::GetBootloaderConfig resp;
    resp.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4

    if(receiveResponse(resp) && resp.success) {
        // Read back bootloader config (1 packet max)
        auto bsonConfig = stream->read();
        // Parse from BSON
        return nlohmann::json::from_bson(bsonConfig);
    } else {
        return {};
    }
}

std::tuple<bool, std::string> DeviceBootloader::flashConfigClear(Memory memory, Type type) {
    // send request to SET_BOOTLOADER_CONFIG
    Request::SetBootloaderConfig setConfigReq;
    setConfigReq.memory = memory;
    if(type != Type::AUTO) {
        setConfigReq.offset = bootloader::getStructure(type).offset.at(bootloader::Section::BOOTLOADER_CONFIG);
    }

    setConfigReq.numPackets = 0;
    setConfigReq.totalSize = 0;
    setConfigReq.clearConfig = 1;
    if(!sendRequest(setConfigReq)) return {false, "Couldn't send request to flash configuration clear"};

    // Read back response
    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    if(!receiveResponse(result)) {
        return {false, "Couldn't receive response to flash configuration clear"};
    }

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashConfigData(nlohmann::json configData, Memory memory, Type type) {
    // Parse to BSON
    auto bson = nlohmann::json::to_bson(configData);

    // Send request to SET_BOOTLOADER_CONFIG
    Request::SetBootloaderConfig setConfigReq;
    setConfigReq.memory = memory;
    if(type != Type::AUTO) {
        setConfigReq.offset = bootloader::getStructure(type).offset.at(bootloader::Section::BOOTLOADER_CONFIG);
    }
    setConfigReq.numPackets = 1;
    setConfigReq.totalSize = static_cast<decltype(setConfigReq.totalSize)>(bson.size());
    setConfigReq.clearConfig = 0;
    if(!sendRequest(setConfigReq)) return {false, "Couldn't send request to flash configuration data"};

    // Send 1 packet, of bson config data
    stream->write(bson);

    // Read back response
    Response::FlashComplete result;
    result.success = 0;  // TODO remove these inits after fix https://github.com/luxonis/depthai-bootloader-shared/issues/4
    result.errorMsg[0] = 0;
    if(!receiveResponse(result)) {
        return {false, "Couldn't receive response to flash configuration data"};
    }

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}

std::tuple<bool, std::string> DeviceBootloader::flashConfigFile(const dai::Path& configPath, Memory memory, Type type) {
    // read a JSON file
    std::ifstream configInputStream(configPath);
    if(!configInputStream.is_open()) throw std::runtime_error(fmt::format("Cannot flash configuration, JSON at path: {} doesn't exist", configPath));
    nlohmann::json configJson;
    configInputStream >> configJson;
    return flashConfigData(configJson, memory, type);
}

DeviceBootloader::Config DeviceBootloader::readConfig(Memory memory, Type type) {
    auto json = readConfigData(memory, type);
    // Implicit parse from json to Config
    return json;
}

std::tuple<bool, std::string> DeviceBootloader::flashConfig(const Config& config, Memory memory, Type type) {
    // Implicit parse from Config to json
    return flashConfigData(config, memory, type);
}

// Boot memory
void DeviceBootloader::bootMemory(const std::vector<uint8_t>& embeddedFw) {
    // Send request to boot firmware directly from bootloader
    Request::BootMemory bootMemory;
    bootMemory.totalSize = static_cast<uint32_t>(embeddedFw.size());
    bootMemory.numPackets = ((static_cast<uint32_t>(embeddedFw.size()) - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendRequest(bootMemory)) {
        throw std::runtime_error("Error trying to connect to device");
    }

    // After that send numPackets of data
    stream->writeSplit(embeddedFw.data(), embeddedFw.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for the link to fall down
    try {
        stream->read();
    } catch(const std::exception&) {
        // ignore
    }
}

void DeviceBootloader::bootUsbRomBootloader() {
    // Boot into USB ROM BOOTLOADER now
    if(!sendRequest(Request::UsbRomBoot{})) {
        throw std::runtime_error("Error trying to connect to device");
    }

    // Then wait for the link to fall down
    try {
        stream->read();
    } catch(const std::exception&) {
        // ignore
    }
}

bool DeviceBootloader::isEmbeddedVersion() const {
    return isEmbedded;
}

std::vector<std::uint8_t> DeviceBootloader::getEmbeddedBootloaderBinary(Type type) {
    return Resources::getInstance().getBootloaderFirmware(type);
}

DeviceBootloader::Version::Version(const std::string& v) : versionMajor(0), versionMinor(0), versionPatch(0), buildInfo{""} {
    // Parse string
    char buffer[256]{0};
    if(std::sscanf(v.c_str(), "%u.%u.%u+%255s", &versionMajor, &versionMinor, &versionPatch, buffer) != 4) {
        if(std::sscanf(v.c_str(), "%u.%u.%u", &versionMajor, &versionMinor, &versionPatch) != 3) {
            throw std::runtime_error("Cannot parse version: " + v);
        }
    } else {
        buildInfo = std::string{buffer};
    }
}

DeviceBootloader::Version::Version(unsigned vmajor, unsigned vminor, unsigned vpatch)
    : versionMajor(vmajor), versionMinor(vminor), versionPatch(vpatch), buildInfo{""} {}

DeviceBootloader::Version::Version(unsigned vmajor, unsigned vminor, unsigned vpatch, std::string buildInfo)
    : versionMajor(vmajor), versionMinor(vminor), versionPatch(vpatch), buildInfo(buildInfo) {}

bool DeviceBootloader::Version::operator==(const Version& other) const {
    if(versionMajor == other.versionMajor && versionMinor == other.versionMinor && versionPatch == other.versionPatch && buildInfo == other.buildInfo) {
        return true;
    }
    return false;
}

bool DeviceBootloader::Version::operator<(const Version& other) const {
    if(versionMajor < other.versionMajor) {
        return true;
    } else if(versionMajor == other.versionMajor) {
        if(versionMinor < other.versionMinor) {
            return true;
        } else if(versionMinor == other.versionMinor) {
            if(versionPatch < other.versionPatch) {
                return true;
            } else if(versionPatch == other.versionPatch) {
                if(!buildInfo.empty() && other.buildInfo.empty()) {
                    return true;
                }
            }
        }
    }
    return false;
}

std::string DeviceBootloader::Version::toString() const {
    std::string version = std::to_string(versionMajor) + "." + std::to_string(versionMinor) + "." + std::to_string(versionPatch);
    if(!buildInfo.empty()) {
        version += "+" + buildInfo;
    }
    return version;
}

std::string DeviceBootloader::Version::toStringSemver() const {
    std::string version = std::to_string(versionMajor) + "." + std::to_string(versionMinor) + "." + std::to_string(versionPatch);
    return version;
}

std::string DeviceBootloader::Version::getBuildInfo() const {
    return buildInfo;
}

DeviceBootloader::Version DeviceBootloader::Version::getSemver() const {
    return Version(versionMajor, versionMinor, versionPatch);
}

template <typename T>
bool DeviceBootloader::sendRequest(const T& request) {
    if(stream == nullptr) return false;

    // Do a version check beforehand
    if(getVersion().getSemver() < Version(T::VERSION)) {
        throw std::runtime_error(
            fmt::format("Bootloader version {} required to send request '{}'. Current version {}", T::VERSION, T::NAME, getVersion().toString()));
    }

    try {
        stream->write((uint8_t*)&request, sizeof(T));
    } catch(const std::exception&) {
        return false;
    }

    return true;
}

bool DeviceBootloader::receiveResponseData(std::vector<uint8_t>& data) {
    if(stream == nullptr) return false;

    data = stream->read();
    return true;
}

template <typename T>
bool DeviceBootloader::parseResponse(const std::vector<uint8_t>& data, T& response) {
    // Checks that 'data' is type T
    Response::Command command;
    if(data.size() < sizeof(command)) return false;
    memcpy(&command, data.data(), sizeof(command));
    if(response.cmd != command) return false;
    if(data.size() < sizeof(response)) return false;

    // If yes, memcpy to response
    memcpy(&response, data.data(), sizeof(response));
    return true;
}

template <typename T>
bool DeviceBootloader::receiveResponse(T& response) {
    if(stream == nullptr) return false;
    // Receive data first
    std::vector<uint8_t> data;
    if(!receiveResponseData(data)) return false;

    // Then try to parse
    if(!parseResponse(data, response)) return false;

    return true;
}

// Config functions
void DeviceBootloader::Config::setStaticIPv4(std::string ip, std::string mask, std::string gateway) {
    network.ipv4 = platform::getIPv4AddressAsBinary(ip);
    network.ipv4Mask = platform::getIPv4AddressAsBinary(mask);
    network.ipv4Gateway = platform::getIPv4AddressAsBinary(gateway);
    network.staticIpv4 = true;
}
void DeviceBootloader::Config::setDynamicIPv4(std::string ip, std::string mask, std::string gateway) {
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

void DeviceBootloader::Config::setDnsIPv4(std::string dns, std::string dnsAlt) {
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

void DeviceBootloader::Config::setMacAddress(std::string mac) {
    std::array<uint8_t, 6> a;
    int last = -1;
    int rc = std::sscanf(mac.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx%n", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &last);
    if(rc != 6 || static_cast<long>(mac.size()) != last) {
        throw std::invalid_argument("Invalid MAC address format " + mac);
    }

    // Set the parsed mac address
    network.mac = a;
}
std::string DeviceBootloader::Config::getMacAddress() {
    // 32 characters is adequite for MAC address representation
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

}  // namespace dai
