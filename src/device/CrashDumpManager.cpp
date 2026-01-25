#include "depthai/device/CrashDumpManager.hpp"

// project
#include "depthai/build/version.hpp"

namespace dai {

static std::string getStringTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto inTime = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&inTime), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

CrashDumpManager::CrashDumpManager(DeviceBase* devicePtr) : device(devicePtr) {
    if(devicePtr == nullptr) {
        throw std::invalid_argument("Device pointer cannot be null");
    }
}

std::unique_ptr<CrashDump> CrashDumpManager::collectCrashDump(bool clear) {
    // Make sure a crash dump is available
    if(!hasCrashDump()) {
        return nullptr;
    }

    // Collect the correct crash dump based on the platform
    std::unique_ptr<CrashDump> dump;
    Platform platform = this->device->getPlatform();

    switch(platform) {
        case Platform::RVC2: {
            auto dumpRVC2 = std::make_unique<CrashDumpRVC2>();
            dumpRVC2->crashReports = this->device->getCrashDump(clear);
            dump = std::move(dumpRVC2);
        } break;

        case Platform::RVC3: {
            throw std::runtime_error("RVC3 crash dumps are not supported!");
        } break;

        case Platform::RVC4: {
            auto dumpRVC4 = std::make_unique<CrashDumpRVC4>();
            // TODO(lnotspotl): add missing data retrieval from device
            dump = std::move(dumpRVC4);
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

    // Device
    dump->deviceId = this->device->getDeviceId();
    dump->crashdumpTimestamp = getStringTimestamp();

    return dump;
}

bool CrashDumpManager::hasCrashDump() {
    return this->device->hasCrashDump();
}

}  // namespace dai
