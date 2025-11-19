#include <depthai/device/Version.hpp>
#include <semver.hpp>

namespace dai {

namespace {

const char* convertPreReleaseToString(Version::PreReleaseType type) {
    switch(type) {
        case Version::PreReleaseType::ALPHA:
            return "alpha";
        case Version::PreReleaseType::BETA:
            return "beta";
        case Version::PreReleaseType::RC:
            return "rc";
        case Version::PreReleaseType::NONE:
            return "";
        default:
            throw std::invalid_argument("Invalid pre-release type");
    }
}

std::string composeSemverString(unsigned major, unsigned minor, unsigned patch, Version::PreReleaseType type, const std::optional<uint16_t>& preReleaseVersion) {
    std::string semverStr = std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    const char* preRelease = convertPreReleaseToString(type);

    if(preRelease[0] != '\0') {
        semverStr += "-";
        semverStr += preRelease;
        if(preReleaseVersion) {
            semverStr += ".";
            semverStr += std::to_string(preReleaseVersion.value());
        }
    }

    return semverStr;
}

semver::version<> parseSemverString(const std::string& semverStr) {
    semver::version<> parsed{};
    auto result = semver::parse(semverStr, parsed);
    if(!result) {
        throw std::invalid_argument("Invalid version string");
    }
    return parsed;
}

}  // namespace

class Version::Impl {
   public:
    explicit Impl(const std::string& v) {
        auto posBuild = v.find('+');

        if(posBuild != std::string::npos) {
            buildInfo = v.substr(posBuild + 1);
        }

        auto semverStr = v.substr(0, posBuild);

        if(semverStr.empty() || !semver::valid(semverStr)) {
            throw std::invalid_argument("Invalid version string");
        }

        version = parseSemverString(semverStr);
    }

    Impl(unsigned major,
         unsigned minor,
         unsigned patch,
         const PreReleaseType& type,
         const std::optional<uint16_t>& preReleaseVersion,
         const std::string& buildInfo)
        : version(parseSemverString(composeSemverString(major, minor, patch, type, preReleaseVersion))), buildInfo(buildInfo) {}

    bool operator==(const Impl& other) const {
        return version == other.version;
    }

    bool operator<(const Impl& other) const {
        return version < other.version;
    }

    std::string toString() const {
        std::string result = version.to_string();
        if(!buildInfo.empty()) {
            result += "+" + buildInfo;
        }
        return result;
    }

    std::string toStringSemver() const {
        return version.to_string();
    }

    std::string getBuildInfo() const {
        return buildInfo;
    }

   private:
    semver::version<> version;
    std::string buildInfo;
};

// Definitions of Version member functions
Version::Version(const std::string& v) : pimpl(spimpl::make_impl<Impl>(v)) {}

Version::Version(
    unsigned major, unsigned minor, unsigned patch, const PreReleaseType& type, const std::optional<uint16_t>& preReleaseVersion, const std::string& buildInfo)
    : pimpl(spimpl::make_impl<Impl>(major, minor, patch, type, preReleaseVersion, buildInfo)) {}

bool Version::operator==(const Version& other) const {
    return *pimpl == *other.pimpl;
}

bool Version::operator<(const Version& other) const {
    return *pimpl < *other.pimpl;
}

std::string Version::toString() const {
    return pimpl->toString();
}

std::string Version::toStringSemver() const {
    return pimpl->toStringSemver();
}

std::string Version::getBuildInfo() const {
    return pimpl->getBuildInfo();
}

}  // namespace dai
