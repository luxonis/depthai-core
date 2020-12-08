#include "Resources.hpp"

#include <cassert>
#include <thread>

// libarchive
#include "archive.h"
#include "archive_entry.h"

// spdlog
#include "spdlog/spdlog.h"

extern "C" {
#include "bspatch/bspatch.h"
}

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {

static std::vector<std::uint8_t> getEmbeddedBootloaderBinary();

#ifdef DEPTHAI_RESOURCES_TAR_XZ
constexpr static auto CMRC_DEPTHAI_DEVICE_TAR_XZ = "depthai-device-" DEPTHAI_DEVICE_VERSION ".tar.xz";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_1_PATH = "depthai-device-openvino-2020.1-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_2_PATH = "depthai-device-openvino-2020.2-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_3_PATH = "depthai-device-openvino-2020.3-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_4_PATH = "depthai-device-openvino-2020.4-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_1_PATH = "depthai-device-openvino-2021.1-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static std::array<const char*, 5> resourcesListTarXz = {
    DEPTHAI_CMD_OPENVINO_2020_1_PATH,
    DEPTHAI_CMD_OPENVINO_2020_2_PATH,
    DEPTHAI_CMD_OPENVINO_2020_3_PATH,
    DEPTHAI_CMD_OPENVINO_2020_4_PATH,
    DEPTHAI_CMD_OPENVINO_2021_1_PATH,
};
#else
// TODO - DEPRECATE

constexpr static auto CMRC_DEPTHAI_CMD_PATH = "depthai-" DEPTHAI_DEVICE_VERSION ".cmd";
    #ifdef DEPTHAI_PATCH_ONLY_MODE
constexpr static auto CMRC_DEPTHAI_USB2_PATCH_PATH = "depthai-usb2-patch-" DEPTHAI_DEVICE_VERSION ".patch";
    #else
constexpr static auto CMRC_DEPTHAI_USB2_CMD_PATH = "depthai-usb2-" DEPTHAI_DEVICE_VERSION ".cmd";
    #endif

static std::vector<std::uint8_t> getEmbeddedDeviceBinary(bool usb2Mode) {
    std::vector<std::uint8_t> finalCmd;

    // Binaries are resource compiled
    #ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    if(usb2Mode) {
        #ifdef DEPTHAI_PATCH_ONLY_MODE

        // Get size of original
        auto depthaiBinary = fs.open(CMRC_DEPTHAI_CMD_PATH);

        // Open patch
        auto depthaiUsb2Patch = fs.open(CMRC_DEPTHAI_USB2_PATCH_PATH);

        // Get new size
        int64_t patchedSize = bspatch_mem_get_newsize(reinterpret_cast<const uint8_t*>(depthaiUsb2Patch.begin()), depthaiUsb2Patch.size());

        // Reserve space for patched binary
        finalCmd.resize(patchedSize);

        // Patch
        int error = bspatch_mem(reinterpret_cast<const uint8_t*>(depthaiBinary.begin()),
                                depthaiBinary.size(),
                                reinterpret_cast<const uint8_t*>(depthaiUsb2Patch.begin()),
                                depthaiUsb2Patch.size(),
                                finalCmd.data());

        // if patch not successful
        if(error > 0) throw std::runtime_error("Error while patching cmd for usb2 mode");

        #else

        auto depthaiUsb2Binary = fs.open(CMRC_DEPTHAI_USB2_CMD_PATH);
        finalCmd = std::vector<std::uint8_t>(depthaiUsb2Binary.begin(), depthaiUsb2Binary.end());

        #endif

    } else {
        auto depthaiBinary = fs.open(CMRC_DEPTHAI_CMD_PATH);
        finalCmd = std::vector<std::uint8_t>(depthaiBinary.begin(), depthaiBinary.end());
    }

    #else
        // Binaries from default path (TODO)

    #endif

    return finalCmd;
}

#endif

Resources& Resources::getInstance() {
    static Resources instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

Resources::Resources() {
#ifdef DEPTHAI_RESOURCES_TAR_XZ

    // Create a thread which lazy-loads firmware resources package
    lazyThread = std::thread([&mtx]() {
        // Hold 'mtx' until initial preload is finished
        std::unique_lock<std::mutex> lock(mtx);

        // Get binaries from internal sources
        auto fs = cmrc::depthai::get_filesystem();
        auto deviceTarXz = fs.open(CMRC_DEPTHAI_DEVICE_TAR_XZ);

        auto t1 = steady_clock::now();

        // Load tar.xz archive from memory
        struct archive* a = archive_read_new();
        archive_read_support_filter_xz(a);
        archive_read_support_format_tar(a);
        int r = archive_read_open_memory(a, deviceTarXz.begin(), deviceTarXz.size());
        if(r != ARCHIVE_OK) {
            // TODO(themarpe) - error handling on libarchive errors
        }

        auto t2 = steady_clock::now();

        struct archive_entry* entry;
        while(archive_read_next_header(a, &entry) == ARCHIVE_OK) {
            // Check whether filename matches to one of required resources
            for(const auto& cpath : resourcesListTarXz) {
                std::string resPath(cpath);
                if(resPath == std::string(archive_entry_pathname(entry))) {
                    // Create an emtpy entry
                    resourceMap[resPath] = std::vector<std::uint8_t>();

                    // Read size, 16KiB
                    std::size_t readSize = 16 * 1024;
                    if(archive_entry_size_is_set(entry)) {
                        // if size is specified, use that for read size
                        readSize = archive_entry_size(entry);
                    }

                    while(true) {
                        // Current size, as a offset to write next data to
                        auto currentSize = resourceMap[resPath].size();

                        // Resize to accomodate for extra data
                        resourceMap[resPath].resize(currentSize + readSize);
                        long long size = archive_read_data(a, &resourceMap[resPath][currentSize], readSize);

                        // Assert that no errors occured
                        assert(size >= 0);

                        // All bytes were read
                        if(size == 0) {
                            break;
                        }
                    }

                    break;
                }
            }
        }
        r = archive_read_free(a);  // Note 3

        // Check that all resources were read
        for(const auto& cpath : resourcesListTarXz) {
            std::string resPath(cpath);
            assert(resourceMap.count(resPath) > 0);
        }

        auto t3 = steady_clock::now();

        // Debug - logs loading times
        spdlog::debug("Resources - archive open: {}, archive read: {}", t2 - t1, t3 - t2);
    });

#endif
}

// Get device firmware
std::vector<std::uint8_t> Resources::getDeviceFirmware(bool usb2Mode, OpenVINO::Version version) {
    // Acquire mutex
    std::unique_lock<std::mutex> lock(mtx);

#ifdef DEPTHAI_RESOURCES_TAR_XZ
// Retrieve firmware from resourceMap
// TODO(themarpe)
#else

    // Return device firmware
    return getEmbeddedDeviceBinary(usb2Mode);

#endif
}

std::vector<std::uint8_t> Resources::getBootloaderFirmware() {
    return getEmbeddedBootloaderBinary();
}

std::vector<std::uint8_t> getEmbeddedBootloaderBinary() {
// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    constexpr static auto CMRC_DEPTHAI_BOOTLOADER_PATH = "depthai-bootloader-" DEPTHAI_BOOTLOADER_VERSION ".cmd";

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    auto bootloaderBinary = fs.open(CMRC_DEPTHAI_BOOTLOADER_PATH);
    return std::vector<std::uint8_t>(bootloaderBinary.begin(), bootloaderBinary.end());

#else
    static_assert(0 && "Unsupported");
    return {};
#endif
}

}  // namespace dai
