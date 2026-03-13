#pragma once

// std
#include <memory>

// project
#include "depthai/device/CrashDump.hpp"
#include "depthai/device/DeviceBase.hpp"

namespace dai {

/**
 * @brief Manager class for handling device crash dumps
 */
class CrashDumpManager {
   public:
    explicit CrashDumpManager(DeviceBase* devicePtr);

    ~CrashDumpManager() = default;

    // Non-copyable
    CrashDumpManager(const CrashDumpManager&) = delete;
    CrashDumpManager& operator=(const CrashDumpManager&) = delete;

    // Movable
    CrashDumpManager(CrashDumpManager&&) = default;
    CrashDumpManager& operator=(CrashDumpManager&&) = default;

    /**
     * @brief Collect crash dump from the device
     * @param clear Clear the cached crash dump on device after collection
     * @return Unique pointer to the CrashDump, or nullptr if no crash dump available
     */
    std::unique_ptr<CrashDump> collectCrashDump(bool clear = true);

    /**
     * @brief Check if a crash dump is available on the device
     */
    bool hasCrashDump();

   private:
    std::unique_ptr<CrashDump> collectDumpRVC2(bool clear);
    std::unique_ptr<CrashDump> collectDumpRVC4();

    DeviceBase* devicePtr;
};

}  // namespace dai
