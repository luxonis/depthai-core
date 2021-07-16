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

constexpr static auto CMRC_DEPTHAI_DEVICE_TAR_XZ = "depthai-device-fwp-" DEPTHAI_DEVICE_VERSION ".tar.xz";

// Main FW
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_4_PATH = "depthai-device-openvino-2021.4-" DEPTHAI_DEVICE_VERSION ".cmd";

// Patches from Main FW

constexpr static auto DEPTHAI_CMD_OPENVINO_2020_3_PATCH_PATH = "depthai-device-openvino-2020.3-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_4_PATCH_PATH = "depthai-device-openvino-2020.4-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_1_PATCH_PATH = "depthai-device-openvino-2021.1-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_2_PATCH_PATH = "depthai-device-openvino-2021.2-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH = "depthai-device-openvino-2021.3-" DEPTHAI_DEVICE_VERSION ".patch";

// Usb2 patches
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2020.3-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2020_4_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2020.4-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_1_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.1-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_2_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.2-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.3-" DEPTHAI_DEVICE_VERSION ".patch";
constexpr static auto DEPTHAI_CMD_OPENVINO_2021_4_USB2_PATCH_PATH = "depthai-device-usb2-patch-openvino-2021.4-" DEPTHAI_DEVICE_VERSION ".patch";

constexpr static std::array<const char*, 12> RESOURCE_LIST_DEVICE = {
    DEPTHAI_CMD_OPENVINO_2020_3_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2020_4_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_1_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_4_PATH,
    DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2020_4_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_1_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_2_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH,
    DEPTHAI_CMD_OPENVINO_2021_4_USB2_PATCH_PATH,

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
        // Read the file and return its contents
        return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {});
    }

// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    // Temporary binary
    std::vector<std::uint8_t> tmpDepthaiBinary;
    // Main FW
    std::vector<std::uint8_t> depthaiBinary = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_4_PATH];
    // Patch from main to specified
    std::vector<std::uint8_t> depthaiPatch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH];
    // Patch from specified to usb2 specified
    std::vector<std::uint8_t> depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_4_USB2_PATCH_PATH];

    switch(version) {
        case OpenVINO::VERSION_2020_3:
            depthaiPatch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2020_3_PATCH_PATH];
            depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2020_3_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2020_4:
            depthaiPatch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2020_4_PATCH_PATH];
            depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2020_4_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2021_1:
            depthaiPatch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_1_PATCH_PATH];
            depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_1_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2021_2:
            depthaiPatch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_2_PATCH_PATH];
            depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_2_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2021_3:
            depthaiPatch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_3_PATCH_PATH];
            depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_3_USB2_PATCH_PATH];
            break;

        case OpenVINO::VERSION_2021_4:
            depthaiBinary = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_4_PATH];
            depthaiUsb2Patch = resourceMapDevice[DEPTHAI_CMD_OPENVINO_2021_4_USB2_PATCH_PATH];
            break;
    }

    // is patching required?
    if(version != OpenVINO::VERSION_2021_4) {
        spdlog::debug("Patching OpenVINO FW version from {} to {}", OpenVINO::getVersionName(OpenVINO::VERSION_2021_4), OpenVINO::getVersionName(version));

        // Get new size
        int64_t patchedSize = bspatch_mem_get_newsize(depthaiPatch.data(), depthaiPatch.size());

        // Reserve space for patched binary
        tmpDepthaiBinary.resize(patchedSize);

        // Patch
        int error = bspatch_mem(depthaiBinary.data(), depthaiBinary.size(), depthaiPatch.data(), depthaiPatch.size(), tmpDepthaiBinary.data());

        // if patch not successful
        if(error > 0) throw std::runtime_error("Error while patching cmd for usb2 mode");

        // Change depthaiBinary to tmpDepthaiBinary
        depthaiBinary = tmpDepthaiBinary;
    }

    if(usb2Mode) {
    #ifdef DEPTHAI_PATCH_ONLY_MODE

        spdlog::debug("Patching FW version {} to USB2 mode", OpenVINO::getVersionName(version));

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

constexpr static auto CMRC_DEPTHAI_BOOTLOADER_TAR_XZ = "depthai-bootloader-fwp-" DEPTHAI_BOOTLOADER_VERSION ".tar.xz";
constexpr static auto DEVICE_BOOTLOADER_USB_PATH = "depthai-bootloader-usb.cmd";
constexpr static auto DEVICE_BOOTLOADER_ETH_PATH = "depthai-bootloader-eth.cmd";

constexpr static std::array<const char*, 2> RESOURCE_LIST_BOOTLOADER = {
    DEVICE_BOOTLOADER_USB_PATH,
    DEVICE_BOOTLOADER_ETH_PATH,
};

std::vector<std::uint8_t> Resources::getBootloaderFirmware(dai::bootloader::Type type) {
    // Acquire mutex (this mutex signifies that lazy load is complete)
    // It is necessary when accessing resourceMap variable
    std::unique_lock<std::mutex> lock(mtxBootloader);

    switch(type) {
        case dai::bootloader::Type::USB:
            return resourceMapBootloader[DEVICE_BOOTLOADER_USB_PATH];
            break;

        case dai::bootloader::Type::NETWORK:
            return resourceMapBootloader[DEVICE_BOOTLOADER_ETH_PATH];
            break;

        default:
            throw std::invalid_argument("Invalid Bootloader Type specified.");
            break;
    }
}

Resources& Resources::getInstance() {
    static Resources instance;  // Guaranteed to be destroyed, instantiated on first use.
    return instance;
}

template <typename CV, typename BOOL, typename MTX, typename PATH, typename LIST, typename MAP>
std::function<void()> getLazyTarXzFunction(MTX& lazyMtx, CV& cv, BOOL& mutexAcquired, MTX& mtxCv, PATH cmrcPath, LIST& resourceList, MAP& resourceMap) {
    return [&lazyMtx, &cv, &mutexAcquired, &mtxCv, cmrcPath, &resourceList, &resourceMap] {
        using namespace std::chrono;

        // Hold 'lazyMtx' until initial preload is finished
        std::unique_lock<std::mutex> lock(lazyMtx);

        // Let the calling thread know that it may continue
        {
            std::unique_lock<std::mutex> cvLock(mtxCv);
            mutexAcquired = true;
            cv.notify_all();
        }

        // Get binaries from internal sources
        auto fs = cmrc::depthai::get_filesystem();
        auto tarXz = fs.open(cmrcPath);

        auto t1 = steady_clock::now();

        // Load tar.xz archive from memory
        struct archive* a = archive_read_new();
        archive_read_support_filter_xz(a);
        archive_read_support_format_tar(a);
        int r = archive_read_open_memory(a, tarXz.begin(), tarXz.size());
        assert(r == ARCHIVE_OK);

        auto t2 = steady_clock::now();

        struct archive_entry* entry;
        while(archive_read_next_header(a, &entry) == ARCHIVE_OK) {
            // Check whether filename matches to one of required resources
            for(const auto& cpath : resourceList) {
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
        for(const auto& cpath : resourceList) {
            std::string resPath(cpath);
            assert(resourceMap.count(resPath) > 0);
        }

        auto t3 = steady_clock::now();

        // Debug - logs loading times
        spdlog::debug(
            "Resources - Archive '{}' open: {}, archive read: {}", cmrcPath, duration_cast<milliseconds>(t2 - t1), duration_cast<milliseconds>(t3 - t2));
    };
}

Resources::Resources() {
    // Device resources
    {
        // condition variable to let this thread know when the mutex was acquired
        std::mutex mtxCv;
        std::condition_variable cv;
        bool mutexAcquired = false;

        // Create a thread which lazy-loads firmware resources package
        lazyThreadDevice =
            std::thread(getLazyTarXzFunction(mtxDevice, cv, mutexAcquired, mtxCv, CMRC_DEPTHAI_DEVICE_TAR_XZ, RESOURCE_LIST_DEVICE, resourceMapDevice));

        // Wait for 'cv' to signal
        std::unique_lock<std::mutex> l(mtxCv);
        cv.wait(l, [&mutexAcquired]() { return mutexAcquired; });
    }

    // Bootloader resources
    {
        // condition variable to let this thread know when the mutex was acquired
        std::mutex mtxCv;
        std::condition_variable cv;
        bool mutexAcquired = false;

        // Create a thread which lazy-loads firmware resources package
        lazyThreadBootloader = std::thread(
            getLazyTarXzFunction(mtxBootloader, cv, mutexAcquired, mtxCv, CMRC_DEPTHAI_BOOTLOADER_TAR_XZ, RESOURCE_LIST_BOOTLOADER, resourceMapBootloader));

        // Wait for 'cv' to signal
        std::unique_lock<std::mutex> l(mtxCv);
        cv.wait(l, [&mutexAcquired]() { return mutexAcquired; });
    }
}

Resources::~Resources() {
    // join the lazy threads
    if(lazyThreadDevice.joinable()) lazyThreadDevice.join();
    if(lazyThreadBootloader.joinable()) lazyThreadBootloader.join();
}

// Get device firmware
std::vector<std::uint8_t> Resources::getDeviceFirmware(bool usb2Mode, OpenVINO::Version version) {
    // Acquire mutex (this mutex signifies that lazy load is complete)
    // It is necessary when accessing resourceMapDevice variable
    std::unique_lock<std::mutex> lock(mtxDevice);

    // Return device firmware
    return getDeviceBinary(version, usb2Mode);
}

}  // namespace dai
