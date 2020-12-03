#pragma once

#include <string>
#include <vector>
#include <exception>
#include <algorithm>
#include <map>
#include <utility>

namespace dai
{
    
// This class supports basic OpenVINO related actions like version identification of neural network blobs,... 
class OpenVINO {

public:
    // OpenVINOVersion holds supported version information
    enum Version {
        VERSION_2020_1,
        VERSION_2020_2,
        VERSION_2020_3,
        VERSION_2020_4,
        VERSION_2021_1,
    };

    static std::vector<Version> getVersions();
    static std::string getVersionName(Version version);
    static Version parseVersionName(const std::string& versionString);
    static std::vector<Version> getBlobSupportedVersions(std::uint32_t majorVersion, std::uint32_t minorVersion);
    static Version getBlobLatestSupportedVersion(std::uint32_t majorVersion, std::uint32_t minorVersion);
    static bool areVersionsBlobCompatible(Version v1, Version v2);

private:
   
    static const std::map<std::pair<std::uint32_t, std::uint32_t>, Version> blobVersionToLatestOpenvinoMapping;
    static const std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<Version>> blobVersionToOpenvinoMapping;

};


} // namespace dai
