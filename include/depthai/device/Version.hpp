#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "depthai/utility/spimpl.h"
namespace dai {

/// Version structure
struct Version {
    enum class PreReleaseType : uint16_t {
        ALPHA = 0,
        BETA = 1,
        RC = 2,
        NONE = 3,
    };
    /// Construct Version from string
    explicit Version(const std::string& v);
    /// Construct Version major, minor, patch, and pre-release information
    Version(unsigned major,
            unsigned minor,
            unsigned patch,
            const PreReleaseType& type = PreReleaseType::NONE,
            const std::optional<uint16_t>& preReleaseVersion = std::nullopt,
            const std::string& buildInfo = "");

    /// Construct Version from major, minor, patch, and build information, with no pre-release suffix.
    Version(unsigned major, unsigned minor, unsigned patch, const std::string& buildInfo)
        : Version(major, minor, patch, PreReleaseType::NONE, std::nullopt, buildInfo) {}
    /// Return whether the versions have equal semantic version precedence.
    bool operator==(const Version& other) const;
    /// Return whether this version precedes the other version.
    bool operator<(const Version& other) const;
    /// Return whether the versions have different semantic version precedence.
    inline bool operator!=(const Version& rhs) const {
        return !(*this == rhs);
    }
    /// Return whether this version follows the other version.
    inline bool operator>(const Version& rhs) const {
        return rhs < *this;
    }
    /// Return whether this version precedes or equals the other version.
    inline bool operator<=(const Version& rhs) const {
        return !(*this > rhs);
    }
    /// Return whether this version follows or equals the other version.
    inline bool operator>=(const Version& rhs) const {
        return !(*this < rhs);
    }
    /// Convert Version to string
    std::string toString() const;
    /// Convert Version to semver (no build information) string
    std::string toStringSemver() const;

    /// Get build info
    std::string getBuildInfo() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai
