#pragma once

#include <algorithm>
#include <exception>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace dai {

/// Support for basic OpenVINO related actions like version identification of neural network blobs,...
class OpenVINO {
   public:
    /// OpenVINO Version supported version information
    enum Version { VERSION_2020_3, VERSION_2020_4, VERSION_2021_1, VERSION_2021_2, VERSION_2021_3, VERSION_2021_4 };

    /**
     * @returns Supported versions
     */
    static std::vector<Version> getVersions();

    /**
     * Returns string representation of a given version
     * @param version OpenVINO version
     * @returns Name of a given version
     */
    static std::string getVersionName(Version version);

    /**
     * Creates Version from string representation.
     * Throws if not possible.
     * @param versionString Version as string
     * @returns Version object if successful
     */
    static Version parseVersionName(const std::string& versionString);

    /**
     * Returns a list of potentionally supported versions for a specified blob major and minor versions.
     * @param majorVersion Major version from OpenVINO blob
     * @param minorVersion Minor version from OpenVINO blob
     * @returns Vector of potentionally supported versions
     */
    static std::vector<Version> getBlobSupportedVersions(std::uint32_t majorVersion, std::uint32_t minorVersion);

    /**
     * Returns latest potentionally supported version by a given blob version.
     * @param majorVersion Major version from OpenVINO blob
     * @param minorVersion Minor version from OpenVINO blob
     * @returns Latest potentionally supported version
     */
    static Version getBlobLatestSupportedVersion(std::uint32_t majorVersion, std::uint32_t minorVersion);

    /**
     * Checks whether two blob versions are compatible
     */
    static bool areVersionsBlobCompatible(Version v1, Version v2);

   private:
    static const std::map<std::pair<std::uint32_t, std::uint32_t>, Version> blobVersionToLatestOpenvinoMapping;
    static const std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<Version>> blobVersionToOpenvinoMapping;
};

}  // namespace dai
