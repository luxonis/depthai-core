#pragma once

// std
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// shared
#include "depthai-bootloader-shared/Config.hpp"
#include "depthai-bootloader-shared/Memory.hpp"
#include "depthai-bootloader-shared/Section.hpp"
#include "depthai-bootloader-shared/Type.hpp"

namespace dai {

// DeviceBootloader (RAII), connects to device and maintains watchdog ...

/**
 * Represents the DepthAI bootloader with the methods to interact with it.
 */
class DeviceBootloader {
   public:
    // Alias
    using Type = dai::bootloader::Type;
    using Memory = dai::bootloader::Memory;
    using Section = dai::bootloader::Section;
    using UsbConfig = dai::bootloader::UsbConfig;
    using NetworkConfig = dai::bootloader::NetworkConfig;

    // Derive and extend bootloader::Config for easier usage
    struct Config : public bootloader::Config {
        /// Setting a static IPv4 won't start DHCP client
        void setStaticIPv4(std::string ip, std::string mask, std::string gateway);
        /// Setting a dynamic IPv4 will set that IP as well as start DHCP client
        void setDynamicIPv4(std::string ip, std::string mask, std::string gateway);
        /// Get if static IPv4 configuration is set
        bool isStaticIPV4();
        /// Get IPv4
        std::string getIPv4();
        /// Get IPv4 mask
        std::string getIPv4Mask();
        /// Get IPv4 gateway
        std::string getIPv4Gateway();
        /// Set IPv4 DNS options
        void setDnsIPv4(std::string dns, std::string dnsAlt = "");
        /// Get primary IPv4 DNS server
        std::string getDnsIPv4();
        /// Get alternate IPv4 DNS server
        std::string getDnsAltIPv4();

        /// Set USB timeout
        void setUsbTimeout(std::chrono::milliseconds ms);
        /// Get USB timeout
        std::chrono::milliseconds getUsbTimeout();

        /// Set NETWOR timeout
        void setNetworkTimeout(std::chrono::milliseconds ms);
        /// Get NETWORK timeout
        std::chrono::milliseconds getNetworkTimeout();

        /// Set MAC address if not flashed on controller
        void setMacAddress(std::string mac);
        /// Get MAC address if not flashed on controller
        std::string getMacAddress();

        /// Set maxUsbSpeed
        void setUsbMaxSpeed(UsbSpeed speed);
        /// Get maxUsbSpeed
        UsbSpeed getUsbMaxSpeed();
    };

    /// Bootloader version structure
    struct Version {
        /// Construct Version from string
        explicit Version(const std::string& v);
        /// Construct Version major, minor and patch numbers
        Version(unsigned major, unsigned minor, unsigned patch);
        bool operator==(const Version& other) const;
        bool operator<(const Version& other) const;
        inline bool operator!=(const Version& rhs) const {
            return !(*this == rhs);
        }
        inline bool operator>(const Version& rhs) const {
            return rhs < *this;
        }
        inline bool operator<=(const Version& rhs) const {
            return !(*this > rhs);
        }
        inline bool operator>=(const Version& rhs) const {
            return !(*this < rhs);
        }
        /// Convert Version to string
        std::string toString() const;

       private:
        unsigned versionMajor, versionMinor, versionPatch;
    };

    // constants

    /// Default Bootloader type
    static constexpr const Type DEFAULT_TYPE{Type::USB};

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
     * @param compress Optional boolean which specifies if contents should be compressed
     * @returns Depthai application package
     */
    static std::vector<uint8_t> createDepthaiApplicationPackage(const Pipeline& pipeline, std::string pathToCmd = "", bool compress = false);

    /**
     * Creates application package which can be flashed to depthai device.
     * @param pipeline Pipeline from which to create the application package
     * @param compress Specifies if contents should be compressed
     * @returns Depthai application package
     */
    static std::vector<uint8_t> createDepthaiApplicationPackage(const Pipeline& pipeline, bool compress);

    /**
     * Saves application package to a file which can be flashed to depthai device.
     * @param path Path where to save the application package
     * @param pipeline Pipeline from which to create the application package
     * @param pathToCmd Optional path to custom device firmware
     * @param compress Optional boolean which specifies if contents should be compressed
     */
    static void saveDepthaiApplicationPackage(std::string path, const Pipeline& pipeline, std::string pathToCmd = "", bool compress = false);

    /**
     * Saves application package to a file which can be flashed to depthai device.
     * @param path Path where to save the application package
     * @param pipeline Pipeline from which to create the application package
     * @param compress Specifies if contents should be compressed
     */
    static void saveDepthaiApplicationPackage(std::string path, const Pipeline& pipeline, bool compress);

    /**
     * @returns Embedded bootloader version
     */
    static Version getEmbeddedBootloaderVersion();

    /**
     * @returns Embedded bootloader binary
     */
    static std::vector<std::uint8_t> getEmbeddedBootloaderBinary(Type type = DEFAULT_TYPE);

    DeviceBootloader() = delete;

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state.
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param allowFlashingBootloader Set to true to allow flashing the devices bootloader. Defaults to false
     */
    explicit DeviceBootloader(const DeviceInfo& devInfo, bool allowFlashingBootloader = false);

    /**
     * Connects to device in bootloader of specified type. Throws if it wasn't possible.
     * This constructor will automatically boot into specified bootloader type if not already running
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param type Type of bootloader to boot/connect to.
     * @param allowFlashingBootloader Set to true to allow flashing the devices bootloader. Defaults to false
     */
    DeviceBootloader(const DeviceInfo& devInfo, Type type, bool allowFlashingBootloader = false);

    /**
     * Connects to or boots device in bootloader mode depending on devInfo state with a custom bootloader firmware.
     * @param devInfo DeviceInfo of which to boot or connect to
     * @param pathToBootloader Custom bootloader firmware to boot
     * @param allowFlashingBootloader Set to true to allow flashing the devices bootloader. Defaults to false
     */
    DeviceBootloader(const DeviceInfo& devInfo, const std::string& pathToBootloader, bool allowFlashingBootloader = false);

    /**
     * @overload
     */
    DeviceBootloader(const DeviceInfo& devInfo, const char* pathToBootloader, bool allowFlashingBootloader = false);
    ~DeviceBootloader();

    /**
     * Flashes a given pipeline to the device.
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param pipeline Pipeline to flash to the board
     */
    std::tuple<bool, std::string> flash(std::function<void(float)> progressCallback, const Pipeline& pipeline, bool compress = false);

    /**
     * Flashes a given pipeline to the device.
     * @param pipeline Pipeline to flash to the board
     */
    std::tuple<bool, std::string> flash(const Pipeline& pipeline, bool compress = false);

    /**
     * Flashes a specific depthai application package that was generated using createDepthaiApplicationPackage or saveDepthaiApplicationPackage
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param package Depthai application package to flash to the board
     */
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::function<void(float)> progressCallback, std::vector<uint8_t> package);

    /**
     * Flashes a specific depthai application package that was generated using createDepthaiApplicationPackage or saveDepthaiApplicationPackage
     * @param package Depthai application package to flash to the board
     */
    std::tuple<bool, std::string> flashDepthaiApplicationPackage(std::vector<uint8_t> package);

    /**
     * Flashes bootloader to the current board
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param path Optional parameter to custom bootloader to flash
     */
    std::tuple<bool, std::string> flashBootloader(std::function<void(float)> progressCallback, std::string path = "");

    /**
     * Flash selected bootloader to the current board
     * @param memory Memory to flash
     * @param type Bootloader type to flash
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param path Optional parameter to custom bootloader to flash
     */
    std::tuple<bool, std::string> flashBootloader(Memory memory, Type type, std::function<void(float)> progressCallback, std::string path = "");

    /**
     * Flash arbitrary data at custom offset in specified memory
     * @param memory Memory to flash
     * @param offset Offset at which to flash the given data in bytes
     * @param progressCallback Callback that sends back a value between 0..1 which signifies current flashing progress
     * @param data Data to flash
     */
    // std::tuple<bool, std::string> flashCustom(Memory memory, uint32_t offset, std::function<void(float)> progressCb, std::vector<uint8_t> data);

    /**
     * Reads configuration data from bootloader
     * @returns Unstructured configuration data
     * @param memory Optional - from which memory to read configuration data
     * @param type Optional - from which type of bootloader to read configuration data
     */
    nlohmann::json readConfigData(Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Flashes configuration data to bootloader
     * @param configData Unstructured configuration data
     * @param memory Optional - to which memory flash configuration
     * @param type Optional - for which type of bootloader to flash configuration
     */
    std::tuple<bool, std::string> flashConfigData(nlohmann::json configData, Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Flashes configuration data to bootloader
     * @param configPath Unstructured configuration data
     * @param memory Optional - to which memory flash configuration
     * @param type Optional - for which type of bootloader to flash configuration
     */
    std::tuple<bool, std::string> flashConfigFile(std::string configPath, Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Clears configuration data
     * @param memory Optional - on which memory to clear configuration data
     * @param type Optional - for which type of bootloader to clear configuration data
     */
    std::tuple<bool, std::string> flashConfigClear(Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Reads configuration from bootloader
     * @param memory Optional - from which memory to read configuration
     * @param type Optional - from which type of bootloader to read configuration
     * @returns Configuration structure
     */
    Config readConfig(Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Flashes configuration to bootloader
     * @param configData Configuration structure
     * @param memory Optional - to which memory flash configuration
     * @param type Optional - for which type of bootloader to flash configuration
     */
    std::tuple<bool, std::string> flashConfig(const Config& config, Memory memory = Memory::AUTO, Type type = Type::AUTO);

    /**
     * Boots a custom FW in memory
     * @param fw
     * @throws A runtime exception if there are any communication issues
     */
    void bootMemory(const std::vector<uint8_t>& fw);

    /**
     * Boots into integrated ROM bootloader in USB mode
     * @throws A runtime exception if there are any communication issues
     */
    void bootUsbRomBootloader();

    /**
     * @returns Version of current running bootloader
     */
    Version getVersion() const;

    /**
     * @returns True when bootloader was booted using latest bootloader integrated in the library.
     * False when bootloader is already running on the device and just connected to.
     */
    bool isEmbeddedVersion() const;

    /**
     * @returns Type of currently connected bootloader
     */
    Type getType() const;

    /**
     * @returns True if allowed to flash bootloader
     */
    bool isAllowedFlashingBootloader() const;

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

    // private methods
    void init(bool embeddedMvcmd, const std::string& pathToMvcmd, tl::optional<bootloader::Type> type, bool allowBlFlash);
    void checkClosed() const;
    template <typename T>
    bool sendRequest(const T& request);
    bool receiveResponseData(std::vector<uint8_t>& data);
    template <typename T>
    bool parseResponse(const std::vector<uint8_t>& data, T& response);
    template <typename T>
    bool receiveResponse(T& response);
    Version requestVersion();

    // private variables
    std::shared_ptr<XLinkConnection> connection;
    DeviceInfo deviceInfo = {};

    bool isEmbedded = false;
    Type bootloaderType;

    // closed
    std::atomic<bool> closed{false};

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // bootloader stream
    std::unique_ptr<XLinkStream> stream;

    // Allow flashing bootloader flag
    bool allowFlashingBootloader = false;

    // Current connected bootloader version
    Version version{0, 0, 2};
};

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Type& type) {
    switch(type) {
        case dai::DeviceBootloader::Type::AUTO:
            out << "AUTO";
            break;
        case dai::DeviceBootloader::Type::USB:
            out << "USB";
            break;
        case dai::DeviceBootloader::Type::NETWORK:
            out << "NETWORK";
            break;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Memory& memory) {
    switch(memory) {
        case dai::DeviceBootloader::Memory::AUTO:
            out << "AUTO";
            break;
        case dai::DeviceBootloader::Memory::FLASH:
            out << "FLASH";
            break;
        case dai::DeviceBootloader::Memory::EMMC:
            out << "EMMC";
            break;
    }
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const dai::DeviceBootloader::Section& type) {
    switch(type) {
        case dai::DeviceBootloader::Section::AUTO:
            out << "AUTO";
            break;
        case dai::DeviceBootloader::Section::HEADER:
            out << "HEADER";
            break;
        case dai::DeviceBootloader::Section::BOOTLOADER:
            out << "BOOTLOADER";
            break;
        case dai::DeviceBootloader::Section::BOOTLOADER_CONFIG:
            out << "BOOTLOADER_CONFIG";
            break;
        case dai::DeviceBootloader::Section::APPLICATION:
            out << "APPLICATION";
            break;
    }
    return out;
}