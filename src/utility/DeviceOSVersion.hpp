#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "depthai/device/Version.hpp"

namespace dai::utility {

// OS release strings may include an image revision prefix, unlike SDK versions.
inline std::optional<Version> parseDeviceOSVersion(std::string version) {
    if(version.size() > 1 && version.front() == 'r') {
        const auto separator = version.find('-');
        if(separator == std::string::npos || separator == 1 || version.find_first_not_of("0123456789", 1) != separator) return std::nullopt;
        version = version.substr(separator + 1);
    }
    try {
        return Version(version);
    } catch(const std::invalid_argument&) {
        return std::nullopt;
    }
}

// Readers retain an immutable snapshot while a new connection refreshes the cache.
class DeviceOSVersionCache {
   public:
    struct Snapshot {
        std::string raw;
        std::optional<Version> parsed;
    };

    std::shared_ptr<const Snapshot> get() const {
        return std::atomic_load(&snapshot);
    }

    template <typename ReadVersion>
    void refresh(ReadVersion&& readVersion) {
        auto next = std::make_shared<Snapshot>();
        try {
            next->raw = readVersion();
            next->parsed = parseDeviceOSVersion(next->raw);
        } catch(...) {
            // Never retain metadata from an earlier connection after a failed RPC.
            std::atomic_store(&snapshot, std::shared_ptr<const Snapshot>(std::move(next)));
            throw;
        }
        std::atomic_store(&snapshot, std::shared_ptr<const Snapshot>(std::move(next)));
    }

   private:
    std::shared_ptr<const Snapshot> snapshot = std::make_shared<const Snapshot>();
};

}  // namespace dai::utility
