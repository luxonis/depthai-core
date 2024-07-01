#pragma once

#include <string>
#include <spimpl.h>
#include <optional>
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
    Version(unsigned major, unsigned minor, unsigned patch, const PreReleaseType& type = PreReleaseType::NONE, const std::optional<uint16_t>& preReleaseVersion = std::nullopt, const std::string& buildInfo = "");

    Version(unsigned major, unsigned minor, unsigned patch, const std::string& buildInfo) : Version(major, minor, patch, PreReleaseType::NONE, std::nullopt, buildInfo) {}
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
    /// Convert Version to semver (no build information) string
    std::string toStringSemver() const;

    /// Get build info
    std::string getBuildInfo() const;

   private:
    class Impl;
    spimpl::impl_ptr<Impl> pimpl;
};

}  // namespace dai