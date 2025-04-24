#include "Resources.hpp"

#include <array>
#include <cassert>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <iostream>
#include <thread>

// libarchive
#include "archive.h"
#include "archive_entry.h"

// spdlog
#include "spdlog/details/os.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"

// shared
#include "depthai/device/BoardConfig.hpp"
#include "depthai/utility/Checksum.hpp"
#include "depthai/utility/Serialization.hpp"

// project
#include "utility/ArchiveUtil.hpp"
#include "utility/Environment.hpp"
#include "utility/ErrorMacros.hpp"
#include "utility/spdlog-fmt.hpp"

extern "C" {
#include "bspatch/bspatch.h"
}

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {

TarXzAccessor::TarXzAccessor(const std::vector<std::uint8_t>& tarGzFile) {
    // Load tar.xz archive from memory
    struct archive* archive = archive_read_new();
    assert(archive != nullptr);

    auto err = archive_read_support_format_tar(archive);  // Support for tar format
    assert(err == ARCHIVE_OK);

    err = archive_read_support_filter_xz(archive);  // Support for xz compression
    assert(err == ARCHIVE_OK);

    // Open the memory archive
    int r = archive_read_open_memory(archive, tarGzFile.data(), tarGzFile.size());
    assert(r == ARCHIVE_OK);

    // Read through the archive and store all the file contents
    struct archive_entry* entry;
    while(archive_read_next_header(archive, &entry) == ARCHIVE_OK) {
        std::string path = archive_entry_pathname(entry);
        std::vector<std::uint8_t> fileData;

        // Read entry data
        const size_t entrySize = archive_entry_size(entry);
        fileData.resize(entrySize);
        archive_read_data(archive, fileData.data(), fileData.size());

        // Store the data in a map
        resourceMap[path] = std::move(fileData);
    }

    archive_read_free(archive);
}

// Method to get file data by path
std::optional<std::vector<std::uint8_t>> TarXzAccessor::getFile(const std::string& path) const {
    auto it = resourceMap.find(path);
    if(it != resourceMap.end()) {
        return it->second;  // Return the file data
    }
    return std::nullopt;  // Return empty optional if file not found
}

TarXzAccessor Resources::getEmbeddedVisualizer() const {
#ifdef DEPTHAI_EMBED_FRONTEND
    // Load visualizer tar.gz archive from memory
    auto fs = cmrc::depthai::get_filesystem();
    constexpr static auto FILE_NAME = "depthai-visualizer-" DEPTHAI_VISUALIZER_VERSION ".tar.xz";
    if(!fs.exists(FILE_NAME)) {
        throw std::runtime_error("Visualizer not found in embedded resources");
    }
    auto visualizerTarGz = fs.open(FILE_NAME);
    std::vector<std::uint8_t> visualizerTarGzData(visualizerTarGz.begin(), visualizerTarGz.end());

    // Create and return TarGzAccessor
    return TarXzAccessor(visualizerTarGzData);
#else
    throw std::runtime_error("Visualizer not embedded in resources");
#endif
}

static std::vector<std::uint8_t> createPrebootHeader(const std::vector<uint8_t>& payload, uint32_t magic1, uint32_t magic2);

constexpr static auto CMRC_DEPTHAI_DEVICE_TAR_XZ = "depthai-device-fwp-" DEPTHAI_DEVICE_VERSION ".tar.xz";

// Main FW
constexpr static auto DEPTHAI_CMD_OPENVINO_UNIVERSAL_PATH = "depthai-device-openvino-universal-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto MAIN_FW_PATH = DEPTHAI_CMD_OPENVINO_UNIVERSAL_PATH;
constexpr static auto MAIN_FW_VERSION = OpenVINO::VERSION_UNIVERSAL;

// Patches from Main FW
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_4_PATCH_PATH = "depthai-device-openvino-2020.4-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_1_PATCH_PATH = "depthai-device-openvino-2021.1-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_2_PATCH_PATH = "depthai-device-openvino-2021.2-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH = "depthai-device-openvino-2021.3-" DEPTHAI_DEVICE_VERSION ".patch";

// Creates std::array without explicitly needing to state the size
template <typename V, typename... T>
static constexpr auto array_of(T&&... t) -> std::array<V, sizeof...(T)> {
    return {{std::forward<T>(t)...}};
}

constexpr static auto RESOURCE_LIST_DEVICE = array_of<const char*>(DEPTHAI_CMD_OPENVINO_UNIVERSAL_PATH,
                                                                   DEPTHAI_CMD_OPENVINO_2020_4_PATCH_PATH,
                                                                   DEPTHAI_CMD_OPENVINO_2021_1_PATCH_PATH,
                                                                   DEPTHAI_CMD_OPENVINO_2021_2_PATCH_PATH,
                                                                   DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH);

std::vector<std::uint8_t> Resources::getDeviceFirmware(Device::Config config, dai::Path pathToMvcmd) const {
// First check if device fw is enabled
#ifndef DEPTHAI_ENABLE_DEVICE_FW
    throw std::invalid_argument("DepthAI compiled without support for MyriadX Device FW");
#endif

    // Wait until lazy load is complete
    {
        std::unique_lock<std::mutex> lock(mtxDevice);
        cvDevice.wait(lock, [this]() { return readyDevice; });
    }

    std::vector<std::uint8_t> finalFwBinary;

    // Get OpenVINO version
    auto& version = config.version;

    // Check if pathToMvcmd variable is set
    dai::Path finalFwBinaryPath;
    if(!pathToMvcmd.empty()) {
        finalFwBinaryPath = pathToMvcmd;
    }
    // Override if env variable DEPTHAI_DEVICE_BINARY is set
    dai::Path fwBinaryPath = utility::getEnvAs<std::string>("DEPTHAI_DEVICE_BINARY", "");
    if(!fwBinaryPath.empty()) {
        finalFwBinaryPath = fwBinaryPath;
    }
    // Return binary from file if any of above paths are present
    if(!finalFwBinaryPath.empty()) {
        // Load binary file at path
        std::ifstream stream(finalFwBinaryPath, std::ios::binary);
        if(!stream.is_open()) {
            // Throw an error
            // TODO(themarpe) - Unify exceptions into meaningful groups
            throw std::runtime_error(
                fmt::format("File at path {}{} doesn't exist.", finalFwBinaryPath, !fwBinaryPath.empty() ? " pointed to by DEPTHAI_DEVICE_BINARY" : ""));
        }
        logger::warn("Overriding firmware: {}", finalFwBinaryPath);
        // Read the file and return its contents
        finalFwBinary = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {});
    } else {
// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

        std::unordered_set<OpenVINO::Version> deprecatedVersions(
            {OpenVINO::VERSION_2020_4, OpenVINO::VERSION_2021_1, OpenVINO::VERSION_2021_2, OpenVINO::VERSION_2021_3});

        if(deprecatedVersions.count(version)) {
            logger::warn("OpenVINO {} is deprecated!", OpenVINO::getVersionName(version));
        }

        // Main FW
        std::vector<std::uint8_t> depthaiBinary;
        // Patch from main to specified
        std::vector<std::uint8_t> depthaiPatch;

        switch(version) {
            case OpenVINO::VERSION_2020_3:
                throw std::runtime_error(fmt::format("OpenVINO {} is not available anymore", OpenVINO::getVersionName(version)));
                break;

            case OpenVINO::VERSION_2020_4:
                depthaiPatch = resourceMapDevice.at(DEPTHAI_CMD_OPENVINO_2020_4_PATCH_PATH);
                break;

            case OpenVINO::VERSION_2021_1:
                depthaiPatch = resourceMapDevice.at(DEPTHAI_CMD_OPENVINO_2021_1_PATCH_PATH);
                break;

            case OpenVINO::VERSION_2021_2:
                depthaiPatch = resourceMapDevice.at(DEPTHAI_CMD_OPENVINO_2021_2_PATCH_PATH);
                break;

            case OpenVINO::VERSION_2021_3:
                depthaiPatch = resourceMapDevice.at(DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH);
                break;

            case OpenVINO::VERSION_2021_4:
            case OpenVINO::VERSION_2022_1:
            case MAIN_FW_VERSION:
                depthaiBinary = resourceMapDevice.at(MAIN_FW_PATH);
                break;
        }

        // is patching required?
        if(!depthaiPatch.empty()) {
            logger::debug("Patching OpenVINO FW version from {} to {}", OpenVINO::getVersionName(MAIN_FW_VERSION), OpenVINO::getVersionName(version));

            // Load full binary for patch
            depthaiBinary = resourceMapDevice.at(MAIN_FW_PATH);

            // Get new size
            int64_t patchedSize = bspatch_mem_get_newsize(depthaiPatch.data(), depthaiPatch.size());

            // Reserve space for patched binary
            std::vector<std::uint8_t> tmpDepthaiBinary{};
            tmpDepthaiBinary.resize(patchedSize);

            // Patch
            int error = bspatch_mem(depthaiBinary.data(), depthaiBinary.size(), depthaiPatch.data(), depthaiPatch.size(), tmpDepthaiBinary.data());

            // if patch not successful
            if(error > 0) {
                throw std::runtime_error(fmt::format(
                    "Error while patching OpenVINO FW version from {} to {}", OpenVINO::getVersionName(MAIN_FW_VERSION), OpenVINO::getVersionName(version)));
            }

            // Change depthaiBinary to tmpDepthaiBinary
            depthaiBinary = std::move(tmpDepthaiBinary);
        }

        finalFwBinary = std::move(depthaiBinary);

#else
        // Binaries from default path (TODO)

#endif
    }

    // Prepend preboot config
    // Serialize preboot
    auto prebootPayload = utility::serialize(config.board);
    auto prebootHeader = createPrebootHeader(prebootPayload, BOARD_CONFIG_MAGIC1, BOARD_CONFIG_MAGIC2);
    finalFwBinary.insert(finalFwBinary.begin(), prebootHeader.begin(), prebootHeader.end());

    // Return created firmware
    return finalFwBinary;
}

constexpr static auto CMRC_DEPTHAI_BOOTLOADER_TAR_XZ = "depthai-bootloader-fwp-" DEPTHAI_BOOTLOADER_VERSION ".tar.xz";
constexpr static auto DEVICE_BOOTLOADER_USB_PATH = "depthai-bootloader-usb.cmd";
constexpr static auto DEVICE_BOOTLOADER_ETH_PATH = "depthai-bootloader-eth.cmd";

constexpr static std::array<const char*, 2> RESOURCE_LIST_BOOTLOADER = {
    DEVICE_BOOTLOADER_USB_PATH,
    DEVICE_BOOTLOADER_ETH_PATH,
};

std::vector<std::uint8_t> Resources::getBootloaderFirmware(dai::bootloader::Type type) const {
// First check if device bootloader fw is enabled
#ifndef DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW
    throw std::invalid_argument("DepthAI compiled without support for MyriadX Device Bootloader FW");
#endif

    // Wait until lazy load is complete
    {
        std::unique_lock<std::mutex> lock(mtxBootloader);
        cvDevice.wait(lock, [this]() { return readyBootloader; });
    }

    // Check if env variable DEPTHAI_BOOTLOADER_BINARY_USB/_ETH is set
    std::string blEnvVar;
    if(type == dai::bootloader::Type::USB) {
        blEnvVar = "DEPTHAI_BOOTLOADER_BINARY_USB";
    } else if(type == dai::bootloader::Type::NETWORK) {
        blEnvVar = "DEPTHAI_BOOTLOADER_BINARY_ETH";
    }
    dai::Path blBinaryPath = utility::getEnvAs<std::string>(blEnvVar, "");
    if(!blBinaryPath.empty()) {
        // Load binary file at path
        std::ifstream stream(blBinaryPath, std::ios::binary);
        if(!stream.is_open()) {
            // Throw an error
            // TODO(themarpe) - Unify exceptions into meaningful groups
            throw std::runtime_error(fmt::format("File at path {} pointed to by {} doesn't exist.", blBinaryPath, blEnvVar));
        }
        logger::warn("Overriding bootloader {}: {}", blEnvVar, blBinaryPath);
        // Read the file and return its content
        return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {});
    }

    switch(type) {
        case dai::bootloader::Type::AUTO:
            throw std::invalid_argument("DeviceBootloader::Type::AUTO not allowed, when getting bootloader firmware.");
            break;

        case dai::bootloader::Type::USB:
            return resourceMapBootloader.at(DEVICE_BOOTLOADER_USB_PATH);
            break;

        case dai::bootloader::Type::NETWORK:
            return resourceMapBootloader.at(DEVICE_BOOTLOADER_ETH_PATH);
            break;

        default:
            throw std::invalid_argument("Invalid Bootloader Type specified.");
            break;
    }
}

#ifdef DEPTHAI_ENABLE_DEVICE_RVC3_FW
constexpr static auto CMRC_DEPTHAI_DEVICE_KB_FWP_TAR_XZ = "depthai-device-kb-fwp-" DEPTHAI_DEVICE_RVC3_VERSION ".tar.xz";
#endif

#ifdef DEPTHAI_ENABLE_DEVICE_RVC4_FW
constexpr static auto CMRC_DEPTHAI_DEVICE_RVC4_FWP_TAR_XZ = "depthai-device-rvc4-fwp-" DEPTHAI_DEVICE_RVC4_VERSION ".tar.xz";
#endif

std::vector<std::uint8_t> Resources::getDeviceRVC3Fwp() const {
// First check if device bootloader fw is enabled
#ifndef DEPTHAI_ENABLE_DEVICE_RVC3_FW
    throw std::invalid_argument("DepthAI compiled without support for RVC3 Device FW");
#else
    return getDeviceFwp(CMRC_DEPTHAI_DEVICE_KB_FWP_TAR_XZ, "DEPTHAI_DEVICE_KB_FWP");
#endif
}

std::vector<std::uint8_t> Resources::getDeviceRVC4Fwp() const {
// First check if device bootloader fw is enabled
#ifndef DEPTHAI_ENABLE_DEVICE_RVC4_FW
    throw std::invalid_argument("DepthAI compiled without support for RVC3 Device FW");
#else
    return getDeviceFwp(CMRC_DEPTHAI_DEVICE_RVC4_FWP_TAR_XZ, "DEPTHAI_DEVICE_RVC4_FWP");
#endif
}

std::vector<std::uint8_t> Resources::getDeviceFwp(const std::string& fwPath, const std::string& envPath) const {
    std::string pathToFwp;

    // Check if pathToMvcmd variable is set
    dai::Path finalFwpPath;
    if(!pathToFwp.empty()) {
        finalFwpPath = pathToFwp;
    }

    // Override if env variable DEPTHAI_DEVICE_KB_FWP is set
    dai::Path fwpPathEnv = utility::getEnvAs<std::string>(envPath, "");
    if(!fwpPathEnv.empty()) {
        finalFwpPath = fwpPathEnv;
        spdlog::warn("Overriding device fwp: {}", finalFwpPath);
    }

    // Return binary from file if any of above paths are present
    if(!finalFwpPath.empty()) {
        // Load binary file at path
        std::ifstream stream(finalFwpPath, std::ios::binary);
        if(!stream.is_open()) {
            // Throw an error
            // TODO(themarpe) - Unify exceptions into meaningful groups
            throw std::runtime_error(fmt::format("File at path {}{} doesn't exist.", finalFwpPath));
        }
        // Read the file and return its contents
        return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {});
    } else {
        // Load from resources
        auto fs = cmrc::depthai::get_filesystem();
        auto tarXz = fs.open(fwPath);
        return {tarXz.begin(), tarXz.end()};
    }
}

Resources& Resources::getInstance() {
    static Resources instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

template <typename CV, typename BOOL, typename MTX, typename PATH, typename LIST, typename MAP>
std::function<void()> getLazyTarXzFunction(MTX& mtx, CV& cv, BOOL& ready, PATH cmrcPath, LIST& resourceList, MAP& resourceMap) {
    return [&mtx, &cv, &ready, cmrcPath, &resourceList, &resourceMap] {
        using namespace std::chrono;

        // Get binaries from internal sources
        auto fs = cmrc::depthai::get_filesystem();
        auto tarXz = fs.open(cmrcPath);

        auto t1 = steady_clock::now();

        // Load tar.xz archive from memory
        struct archive* aPtr = archive_read_new();
        DAI_CHECK_IN(aPtr);
        dai::utility::ArchiveUtil archive(aPtr);
        archive_read_support_filter_xz(archive.getA());
        archive_read_support_format_tar(archive.getA());
        int r = archive_read_open_memory(archive.getA(), tarXz.begin(), tarXz.size());
        assert(r == ARCHIVE_OK);

        auto t2 = steady_clock::now();

        struct archive_entry* entry;
        while(archive_read_next_header(archive.getA(), &entry) == ARCHIVE_OK) {
            // Check whether filename matches to one of required resources
            for(const auto& cpath : resourceList) {
                std::string resPath(cpath);
                if(resPath == std::string(archive_entry_pathname(entry))) {
                    archive.readEntry(entry, resourceMap[resPath]);
                    // Entry found - go to next required resource
                    break;
                }
            }
        }

        // Check that all resources were read
        for(const auto& cpath : resourceList) {
            std::string resPath(cpath);
            assert(resourceMap.count(resPath) > 0);
        }

        auto t3 = steady_clock::now();

        // Debug - logs loading times
        logger::debug(
            "Resources - Archive '{}' open: {}, archive read: {}", cmrcPath, duration_cast<milliseconds>(t2 - t1), duration_cast<milliseconds>(t3 - t2));

        // Notify that that preload is finished
        {
            std::unique_lock<std::mutex> l(mtx);
            ready = true;
        }
        cv.notify_all();
    };
}

Resources::Resources() {
    // Preinit libarchive
    struct archive* a = archive_read_new();
    auto r = archive_read_free(a);
    assert(r == ARCHIVE_OK);
    // Ignore 'r' variable when in Release build
    (void)r;

// First check if device kb fw is enabled
#ifdef DEPTHAI_ENABLE_DEVICE_FW
    // Device resources
    // Create a thread which lazy-loads firmware resources package
    lazyThreadDevice = std::thread(getLazyTarXzFunction(mtxDevice, cvDevice, readyDevice, CMRC_DEPTHAI_DEVICE_TAR_XZ, RESOURCE_LIST_DEVICE, resourceMapDevice));
#endif

// First check if device bootloader fw is enabled
#ifdef DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW
    // Bootloader resources
    // Create a thread which lazy-loads firmware resources package
    lazyThreadBootloader = std::thread(
        getLazyTarXzFunction(mtxBootloader, cvBootloader, readyBootloader, CMRC_DEPTHAI_BOOTLOADER_TAR_XZ, RESOURCE_LIST_BOOTLOADER, resourceMapBootloader));
#endif
}

Resources::~Resources() {
    // join the lazy threads
#ifdef DEPTHAI_ENABLE_DEVICE_FW
    if(lazyThreadDevice.joinable()) lazyThreadDevice.join();
#endif
#ifdef DEPTHAI_ENABLE_DEVICE_BOOTLOADER_FW
    if(lazyThreadBootloader.joinable()) lazyThreadBootloader.join();
#endif
}

// Get device firmware
std::vector<std::uint8_t> Resources::getDeviceFirmware(bool usb2Mode, OpenVINO::Version version) const {
    Device::Config cfg;
    if(usb2Mode) {
        cfg.board.usb.maxSpeed = UsbSpeed::HIGH;
    } else {
        cfg.board.usb.maxSpeed = Device::DEFAULT_USB_SPEED;
    }
    cfg.version = version;

    return getDeviceFirmware(cfg);
}

std::vector<std::uint8_t> createPrebootHeader(const std::vector<uint8_t>& payload, uint32_t magic1, uint32_t magic2) {
    // clang-format off
    const std::uint8_t HEADER[] = {77, 65, 50, 120,
                                   // WD Protection
                                   // TODO(themarpe) - expose timings
                                   0x9A, 0xA8, 0x00, 0x32, 0x20, 0xAD, 0xDE, 0xD0, 0xF1,
                                   0x9A, 0x9C, 0x00, 0x32, 0x20, 0xFF, 0xFF, 0xFF, 0xFF,
                                   0x9A, 0xA8, 0x00, 0x32, 0x20, 0xAD, 0xDE, 0xD0, 0xF1,
                                   0x9A, 0xA4, 0x00, 0x32, 0x20, 0x01, 0x00, 0x00, 0x00,
                                   0x8A,
                                   static_cast<uint8_t>((magic1 >> 0) & 0xFF),
                                   static_cast<uint8_t>((magic1 >> 8) & 0xFF),
                                   static_cast<uint8_t>((magic1 >> 16) & 0xFF),
                                   static_cast<uint8_t>((magic1 >> 24) & 0xFF)};
    // clang-format on

    // Store the constructed board information
    std::vector<std::uint8_t> prebootHeader;

    // Store initial header
    prebootHeader.insert(prebootHeader.begin(), std::begin(HEADER), std::end(HEADER));

    // Calculate size
    std::size_t totalPayloadSize = payload.size() + sizeof(magic2) + sizeof(uint32_t) + sizeof(uint32_t);
    std::size_t toAddBytes = 0;
    if(totalPayloadSize % 4 != 0) {
        toAddBytes = 4 - (totalPayloadSize % 4);
    }
    std::size_t totalSize = totalPayloadSize + toAddBytes;
    std::size_t totalSizeWord = totalSize / 4;

    // Write size in words in little endian
    prebootHeader.push_back((totalSizeWord >> 0) & 0xFF);
    prebootHeader.push_back((totalSizeWord >> 8) & 0xFF);

    // Compute payload checksum
    auto checksum = utility::checksum(payload.data(), payload.size());

    // Write checksum & payload size as uint32_t LE
    for(const auto& field : {magic2, checksum, static_cast<uint32_t>(payload.size())}) {
        for(int i = 0; i < 4; i++) {
            prebootHeader.push_back((field >> (i * 8)) & 0xFF);
        }
    }

    // Copy payload
    prebootHeader.insert(prebootHeader.end(), payload.begin(), payload.end());
    // Add missing bytes
    for(std::size_t i = 0; i < toAddBytes; i++) {
        prebootHeader.push_back(0x00);
    }

    return prebootHeader;
}

}  // namespace dai
