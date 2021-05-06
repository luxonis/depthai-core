#include "Resources.hpp"

#include <array>
#include <cassert>
#include <condition_variable>
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

constexpr static auto CMRC_DEPTHAI_DEVICE_TAR_XZ = "depthai-device-fwp-" DEPTHAI_DEVICE_VERSION ".tar.xz";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_1_PATH = "depthai-device-openvino-2020.1-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_3_PATH = "depthai-device-openvino-2020.3-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_2_PATH = DEPTHAI_CMD_OPENVINO_2020_3_PATH;
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_4_PATH = "depthai-device-openvino-2020.4-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_1_PATH = "depthai-device-openvino-2021.1-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_2_PATH = "depthai-device-openvino-2021.2-" DEPTHAI_DEVICE_VERSION ".cmd";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_3_PATH = "depthai-device-openvino-2021.3-" DEPTHAI_DEVICE_VERSION ".cmd";

constexpr static auto DEPTHAI_CMD_OPENVINO_2020_1_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2020.1-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2020.3-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_2_USB2_PATCH_PATH = DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH;
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_4_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2020.4-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_1_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.1-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_2_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.2-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.3-" DEPTHAI_DEVICE_VERSION ".patch";

constexpr static std::array<const char*, 14> resourcesListTarXz = {
    DEPTHAI_CMD_OPENVINO_2020_1_PATH,
    DEPTHAI_CMD_OPENVINO_2020_2_PATH,
    DEPTHAI_CMD_OPENVINO_2020_3_PATH,
    DEPTHAI_CMD_OPENVINO_2020_4_PATH,
    DEPTHAI_CMD_OPENVINO_2021_1_PATH,
    DEPTHAI_CMD_OPENVINO_2021_2_PATH,
    DEPTHAI_CMD_OPENVINO_2021_3_PATH,
    DEPTHAI_CMD_OPENVINO_2020_1_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2020_2_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2020_4_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_1_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_2_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH,
};

std::vector<std::uint8_t> Resources::getDeviceBinary(OpenVINO::Version version, bool usb2Mode) {
    std::vector<std::uint8_t> finalCmd;

    // Check if env variable DEPTHAI_DEVICE_BINARY is set
    auto fwBinaryPath = spdlog::details::os::getenv("DEPTHAI_DEVICE_BINARY");
    if(!fwBinaryPath.empty()) {
        // Load binary file at path
        std::ifstream stream(fwBinaryPath, std::ios::binary);
        if(!stream.is_open()) {
            // Throw an error
            // TODO(themarpe) - Unify exceptions into meaningful groups
            throw std::runtime_error(fmt::format("File at path {} pointed to by DEPTHAI_DEVICE_BINARY doesn't exist.", fwBinaryPath));
        }

        return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {});
    }

    // Binaries are resource compiled
    #ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    std::vector<std::uint8_t>& depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2021_3_PATH];
    std::vector<std::uint8_t>& depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH];

    switch(version) {
        case OpenVINO::VERSION_2020_1:
            spdlog::warn("OpenVino version 2020.1 is deprecated and will be removed in the next release!");
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2020_1_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2020_1_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2020_2:
            spdlog::warn("OpenVino version 2020.2 is deprecated and will be removed in the next release!");
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2020_2_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2020_2_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2020_3:
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2020_3_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2020_4:
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2020_4_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2020_4_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2021_1:
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2021_1_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2021_1_USB2_PATCH_PATH];
            break;
        case OpenVINO::VERSION_2021_2:
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2021_2_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2021_2_USB2_PATCH_PATH];
            break;
        case OpenVINO::VERSION_2021_3:
            depthaiBinary = resourceMap[DEPTHAI_CMD_OPENVINO_2021_3_PATH];
            depthaiUsb2Patch = resourceMap[DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH];
            break;
    }

    if(usb2Mode) {
        #ifdef DEPTHAI_PATCH_ONLY_MODE

        // Get new size
        int64_t patchedSize = bspatch_mem_get_newsize(depthaiUsb2Patch.data(), depthaiUsb2Patch.size());

        // Reserve space for patched binary
        finalCmd.resize(patchedSize);

        // Patch
        int error = bspatch_mem(depthaiBinary.data(), depthaiBinary.size(), depthaiUsb2Patch.data(), depthaiUsb2Patch.size(), finalCmd.data());

        // if patch not successful
        if(error > 0) throw std::runtime_error("Error while patching cmd for usb2 mode");

        #else

        static_assert("Unsupported currently");

        #endif

    } else {
        return depthaiBinary;
    }

    #else
        // Binaries from default path (TODO)

    #endif

    return finalCmd;
}

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

    // condition variable to let this thread know when the mutex was acquired
    std::mutex mtxCv;
    std::condition_variable cv;
    bool mutexAcquired = false;

    // Create a thread which lazy-loads firmware resources package
    lazyThread = std::thread([this, &cv, &mutexAcquired, &mtxCv]() {
        using namespace std::chrono;

        // Hold 'mtx' until initial preload is finished
        std::unique_lock<std::mutex> lock(mtx);

        // Let the calling thread know that it may continue
        {
            std::unique_lock<std::mutex> cvLock(mtxCv);
            mutexAcquired = true;
            cv.notify_all();
        }

        // Get binaries from internal sources
        auto fs = cmrc::depthai::get_filesystem();
        auto deviceTarXz = fs.open(CMRC_DEPTHAI_DEVICE_TAR_XZ);

        auto t1 = steady_clock::now();

        // Load tar.xz archive from memory
        struct archive* a = archive_read_new();
        archive_read_support_filter_xz(a);
        archive_read_support_format_tar(a);
        int r = archive_read_open_memory(a, deviceTarXz.begin(), deviceTarXz.size());
        assert(r == ARCHIVE_OK);

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

                    // Record number of bytes actually read
                    long long finalSize = 0;

                    while(true) {
                        // Current size, as a offset to write next data to
                        auto currentSize = resourceMap[resPath].size();

                        // Resize to accomodate for extra data
                        resourceMap[resPath].resize(currentSize + readSize);
                        long long size = archive_read_data(a, &resourceMap[resPath][currentSize], readSize);

                        // Assert that no errors occured
                        assert(size >= 0);

                        // Append number of bytes actually read to finalSize
                        finalSize += size;

                        // All bytes were read
                        if(size == 0) {
                            break;
                        }
                    }

                    // Resize vector to actual read size
                    resourceMap[resPath].resize(finalSize);

                    // Entry found - go to next required resource
                    break;
                }
            }
        }
        r = archive_read_free(a);  // Note 3
        assert(r == ARCHIVE_OK);
        // Ignore 'r' variable when in Release build
        (void)r;

        // Check that all resources were read
        for(const auto& cpath : resourcesListTarXz) {
            std::string resPath(cpath);
            assert(resourceMap.count(resPath) > 0);
        }

        auto t3 = steady_clock::now();

        // Debug - logs loading times
        spdlog::debug("Resources - archive open: {}, archive read: {}", t2 - t1, t3 - t2);
    });

    // Wait for 'cv' to signal
    std::unique_lock<std::mutex> l(mtxCv);
    cv.wait(l, [&mutexAcquired]() { return mutexAcquired; });

#endif
}

Resources::~Resources() {
    // join the lazy thread
    if(lazyThread.joinable()) lazyThread.join();
}

// Get device firmware
std::vector<std::uint8_t> Resources::getDeviceFirmware(bool usb2Mode, OpenVINO::Version version) {
    // Acquire mutex (this mutex signifies that lazy load is complete)
    // It is necessary when accessing resourceMap variable
    std::unique_lock<std::mutex> lock(mtx);

#ifdef DEPTHAI_RESOURCES_TAR_XZ

    // Return device firmware
    return getDeviceBinary(version, usb2Mode);

#else

    // Return device firmware
    return getEmbeddedDeviceBinary(usb2Mode);

#endif
}

std::vector<std::uint8_t> Resources::getBootloaderFirmware() {
    // Acquire mutex (this mutex signifies that lazy load is complete)
    // It is necessary when accessing resourceMap variable
    std::unique_lock<std::mutex> lock(mtx);

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
