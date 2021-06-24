
#include "depthai/openvino/OpenVINO.hpp"

#include <algorithm>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "spdlog/spdlog.h"

namespace dai {

// static member init
// {{major, minor}, 'latest openvino version to support it'}
// major and minor represent openvino NN blob version information
const std::map<std::pair<std::uint32_t, std::uint32_t>, OpenVINO::Version> OpenVINO::blobVersionToLatestOpenvinoMapping = {
    {{5, 0}, OpenVINO::VERSION_2020_3},
    {{6, 0}, OpenVINO::VERSION_2021_3},
    {{2020, 1}, OpenVINO::VERSION_2020_1},
    {{2020, 2}, OpenVINO::VERSION_2020_2},
    {{2020, 3}, OpenVINO::VERSION_2020_3},
    {{2020, 4}, OpenVINO::VERSION_2020_4},
    {{2021, 1}, OpenVINO::VERSION_2021_1},
    {{2021, 2}, OpenVINO::VERSION_2021_2},
    {{2021, 3}, OpenVINO::VERSION_2021_3},
};

const std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<OpenVINO::Version>> OpenVINO::blobVersionToOpenvinoMapping = {
    {{5, 0}, {OpenVINO::VERSION_2020_1, OpenVINO::VERSION_2020_2, OpenVINO::VERSION_2020_3}},
    {{6, 0}, {OpenVINO::VERSION_2020_4, OpenVINO::VERSION_2021_1, OpenVINO::VERSION_2021_2, OpenVINO::VERSION_2021_3}},
    {{2020, 1}, {OpenVINO::VERSION_2020_1}},
    {{2020, 2}, {OpenVINO::VERSION_2020_2}},
    {{2020, 3}, {OpenVINO::VERSION_2020_3}},
    {{2020, 4}, {OpenVINO::VERSION_2020_4}},
    {{2021, 1}, {OpenVINO::VERSION_2021_1}},
    {{2021, 2}, {OpenVINO::VERSION_2021_2}},
    {{2021, 3}, {OpenVINO::VERSION_2021_3}},
};

std::vector<OpenVINO::Version> OpenVINO::getVersions() {
    return {OpenVINO::VERSION_2020_1,
            OpenVINO::VERSION_2020_2,
            OpenVINO::VERSION_2020_3,
            OpenVINO::VERSION_2020_4,
            OpenVINO::VERSION_2021_1,
            OpenVINO::VERSION_2021_2,
            OpenVINO::VERSION_2021_3};
}

std::string OpenVINO::getVersionName(OpenVINO::Version version) {
    switch(version) {
        case OpenVINO::VERSION_2020_1:
            return "2020.1";
        case OpenVINO::VERSION_2020_2:
            return "2020.2";
        case OpenVINO::VERSION_2020_3:
            return "2020.3";
        case OpenVINO::VERSION_2020_4:
            return "2020.4";
        case OpenVINO::VERSION_2021_1:
            return "2021.1";
        case OpenVINO::VERSION_2021_2:
            return "2021.2";
        case OpenVINO::VERSION_2021_3:
            return "2021.3";
    }
    throw std::logic_error("OpenVINO - Unknown version enum specified");
}

OpenVINO::Version OpenVINO::parseVersionName(const std::string& versionString) {
    auto versions = getVersions();
    for(const auto& v : versions) {
        if(versionString == getVersionName(v)) {
            return v;
        }
    }
    throw std::logic_error("OpenVINO - Cannot parse version name: " + versionString);
}

std::vector<OpenVINO::Version> OpenVINO::getBlobSupportedVersions(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    std::pair<std::uint32_t, std::uint32_t> blobVersion;
    blobVersion.first = majorVersion;
    blobVersion.second = minorVersion;

    if(blobVersionToOpenvinoMapping.count(blobVersion) > 0) {
        return blobVersionToOpenvinoMapping.at(blobVersion);
    }
    return {};
}

OpenVINO::Version OpenVINO::getBlobLatestSupportedVersion(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    std::pair<std::uint32_t, std::uint32_t> blobVersion;
    blobVersion.first = majorVersion;
    blobVersion.second = minorVersion;

    return blobVersionToLatestOpenvinoMapping.at(blobVersion);
}

bool OpenVINO::areVersionsBlobCompatible(OpenVINO::Version v1, OpenVINO::Version v2) {
    // Check all blob versions
    for(const auto& kv : blobVersionToOpenvinoMapping) {
        bool v1Found = false;
        bool v2Found = false;

        // Check if both openvino versions are in same blob version
        for(const auto& el : blobVersionToOpenvinoMapping.at(kv.first)) {
            if(el == v1) v1Found = true;
            if(el == v2) v2Found = true;
        }

        if(v1Found && v2Found) {
            // if both were found, return true
            return true;
        } else if(!v1Found && !v2Found) {
            // If both weren't found, continue
            continue;
        } else {
            // If one was found but other wasn't, return false
            return false;
        }
    }

    // If versions weren't matched up in any of the above cases, log an error and return false
    spdlog::error("OpenVINO - version compatibility check with invalid values or unknown blob version");
    return false;
}

}  // namespace dai
