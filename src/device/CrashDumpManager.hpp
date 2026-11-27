#pragma once

#include <memory>

#include "depthai/device/CrashDump.hpp"
#include "depthai/device/DeviceBase.hpp"

namespace dai {

class CrashDumpManager {
   public:
    explicit CrashDumpManager(DeviceBase* devicePtr);

    ~CrashDumpManager() = default;

    CrashDumpManager(const CrashDumpManager&) = delete;
    CrashDumpManager& operator=(const CrashDumpManager&) = delete;

    CrashDumpManager(CrashDumpManager&& other) noexcept;
    CrashDumpManager& operator=(CrashDumpManager&& other) noexcept;

    std::unique_ptr<CrashDump> collectCrashDump(bool clear = true);  // clear: delete the dump on the device; the payload is empty when there is no dump

    bool hasCrashDump();

   private:
    std::unique_ptr<CrashDump> collectDumpRVC2(bool clear);
    std::unique_ptr<CrashDump> collectDumpRVC4();

    DeviceBase* devicePtr;
};

}  // namespace dai
