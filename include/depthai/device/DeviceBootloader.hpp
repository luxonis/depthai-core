#pragma once

// std
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// libraries
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"

namespace dai {

// DeviceBootloader (RAII), connects to device and maintains watchdog ...

/**
 * Represents the DepthAI bootloader with the methods to interact with it.
 */
class DeviceBootloader {
   public:
    /// Bootloader version structure
    struct Version {
        /// Construct Version from string
        explicit Version(const std::string& v);
        /// Construct Version major, minor and patch numbers
        Version(unsigned major, unsigned minor, unsigned patch);
        bool operator==(const Version& other) const;
        bool operator>(const Version& other) const;
        bool operator<(const Version& other) const;
        /// Convert Version to string
        std::string toString() const;

       private:
        unsigned versionMajor, versionMinor, versionPatch;
    };

    // Static API
    /**
     * Searches for connected devices in either UNBOOTED or BOOTLOADER states and returns first available.
     * @returns Tuple of boolean and DeviceInfo. If found boolean is true and DeviceInfo describes the device. Otherwise false
     */
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();

    /**
     * Searches for connected devices in either UNBOOTED or BOOTLOADER states.
     * @returns Vector of all found devices
     */
    static std::vector<DeviceInfo> getAllAvailableDevices();

    /**
     * Creates application package which can be flashed to depthai device.
     * @param pipeline Pipeline from which to create the application package
     * @param pathToCmd Optional path to custom device firmware
     * @returns Depthai application package
     */
    static std::vector<uint8_t> createDepthaiApplicationPackage(Pipeline& pipeline, std::string pathToCmd = "");

    /**
     * Saves application package to a file which can be flashed to depthai device.
     * @param path Path where to save the application package
     * @param pipeline Pipeline from which to create the application package
     * @param pathToCmd Optional path to custom device firmware
     */
    static void saveDepthaiApplicationPackage(std::string path, Pipeline& pipeline, std::string pathToCmd = "");

    /**
     * @returns Embedded bootloader version
     */
    static Version getEmbeddedBootloaderVersion();

    /**
     * @returns Embedded bootloader binary
     */
    static std::vector<std::uint8_t> getEmbeddedBootloaderBinary();
    //

    DeviceBootloader() = delete;

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state.
     * @param devInfo DeviceInfo of which to boot or connect to
     */
    explicit DeviceBootloader(const DeviceInfo& devInfo);

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state with a custom bootloader firmware.
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param pathToBootloader Custom bootloader firmware to boot
     */
    DeviceBootloader(const DeviceInfo& devInfo, const std::string& pathToBootloader);

    /**
     * @overload
     */
    DeviceBootloader(const DeviceInfo& devInfo, const char* pathToBootloader);
    ~DeviceBootloader();

    /**
     * Flashes a give pipeline to the board.
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param pipeline Pipeline to flash to the board
     */
    std::tuple<bool, std::string> flash(std::function<void(float)> progressCallback, Pipeline& pipeline);

    /**
     * Flashes a specific depthai application package that was generated using createDepthaiApplicationPackage or saveDepthaiApplicationPackage
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param package Depthai application package to flash to the board
     */
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::function<void(float)> progressCallback, std::vector<uint8_t> package);

    /**
     * Flashes bootloader to the current board
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param path Optional parameter to custom bootloader to flash
     */
    std::tuple<bool, std::string> flashBootloader(std::function<void(float)> progressCallback, std::string path = "");

    /**
     * @returns Version of current running bootloader
     */
    Version getVersion();

    /**
     * @returns True whether the bootloader running is flashed or booted by library
     */
    bool isEmbeddedVersion();

    /**
     * Explicitly closes connection to device.
     * @note This function does not need to be explicitly called
     * as destructor closes the device automatically
     */
    void close();

    /**
     * Is the device already closed (or disconnected)
     */
    bool isClosed() const;

   private:
    // private static

    // private variables
    void init(bool embeddedMvcmd, const std::string& pathToMvcmd);
    void checkClosed() const;

    std::shared_ptr<XLinkConnection> connection;
    DeviceInfo deviceInfo = {};

    bool isEmbedded = false;

    // closed
    std::atomic<bool> closed{false};

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // bootloader stream
    std::unique_ptr<XLinkStream> stream;
};

}  // namespace dai
