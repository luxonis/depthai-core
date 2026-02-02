#include "depthai/device/CrashDumpManager.hpp"

#include <stdexcept>

// project
#include "depthai/build/version.hpp"
#include "depthai/device/DeviceGate.hpp"
#include "utility/Platform.hpp"

namespace dai {

static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t inTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#ifdef _WIN32
    localtime_s(&localTime, &inTime);  // thread-safe, Windows compliant
#else
    localtime_r(&inTime, &localTime);  // thread-safe, POSIX compliant
#endif
    std::stringstream ss;
    ss << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

CrashDumpManager::CrashDumpManager(DeviceBase* devicePtr) : devicePtr(devicePtr) {
    if(devicePtr == nullptr) {
        throw std::invalid_argument("Device pointer cannot be null");
    }
}

std::unique_ptr<CrashDump> CrashDumpManager::collectCrashDump(bool clear) {
    // Collect the correct crash dump based on the platform
    std::unique_ptr<CrashDump> dump;
    Platform platform = this->devicePtr->getPlatform();

    switch(platform) {
        case Platform::RVC2: {
            dump = collectDumpRVC2(clear);
        } break;

        case Platform::RVC3: {
            throw std::runtime_error("RVC3 crash dumps are not supported!");
        } break;

        case Platform::RVC4: {
            dump = collectDumpRVC4();
        } break;

        default:
            throw std::runtime_error("Unsupported platform");
    }

    // Add metadata and other helpful information
    dump->depthaiVersion = build::VERSION;
    dump->depthaiVersionMajor = std::to_string(build::VERSION_MAJOR);
    dump->depthaiVersionMinor = std::to_string(build::VERSION_MINOR);
    dump->depthaiVersionPatch = std::to_string(build::VERSION_PATCH);
    dump->depthaiVersionPreReleaseType = build::PRE_RELEASE_TYPE;
    dump->depthaiVersionPreReleaseVersion = std::to_string(build::PRE_RELEASE_VERSION);
    dump->depthaiVersionBuildInfo = build::BUILD_DATETIME;
    dump->depthaiCommitHash = build::COMMIT;
    dump->depthaiCommitDatetime = build::COMMIT_DATETIME;
    dump->depthaiDeviceVersion = build::DEVICE_VERSION;
    dump->depthaiBootloaderVersion = build::BOOTLOADER_VERSION;
    dump->depthaiDeviceRVC3Version = build::DEVICE_RVC3_VERSION;
    dump->depthaiDeviceRVC4Version = build::DEVICE_RVC4_VERSION;
    dump->osPlatform = platform::getOSPlatform();

    // Device
    dump->deviceId = this->devicePtr->deviceInfo.getDeviceId();

    // Crashdump collection time
    dump->crashdumpTimestamp = getCurrentTimestamp();

    return dump;
}

std::unique_ptr<CrashDump> CrashDumpManager::collectDumpRVC2(bool clear) {
    auto dump = std::make_unique<CrashDumpRVC2>();
    if(hasCrashDump()) {
        dump->crashReports = this->devicePtr->getCrashDump(clear);
    }
    return dump;
}

std::unique_ptr<CrashDump> CrashDumpManager::collectDumpRVC4() {
    if(!this->devicePtr->gate) {
        throw std::runtime_error("RVC4 device has no gate, cannot collect crashdump");
    }
    auto dump = std::make_unique<CrashDumpRVC4>();
    auto gateDump = this->devicePtr->gate->getCrashDump();
    if(gateDump) {
        dump->data = std::move(gateDump->data);
        dump->filename = std::move(gateDump->filename);
    }
    return dump;
}

bool CrashDumpManager::hasCrashDump() {
    return this->devicePtr->hasCrashDump();
}

}  // namespace dai
