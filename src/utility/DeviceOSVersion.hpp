#pragma once

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

}  // namespace dai::utility
