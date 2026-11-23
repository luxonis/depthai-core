#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/utility/Serialization.hpp"

namespace dai::nn_archive::v1 {

/** Implementation library for one SNPE runtime. */
struct RuntimeImplementation {
    /** SNPE runtime, such as "dsp" or "cpu". */
    std::string runtime;
    /** Normalized relative POSIX path inside the archive. */
    std::string path;
};

/** Registration library and implementations of an SNPE UDO package. */
struct SNPEUDOPackage {
    /** Archive path of the registration library. */
    std::string registration;
    /** Implementation libraries required by the package. */
    std::vector<RuntimeImplementation> implementations;
};

/** Optional native libraries bundled with an NNArchive. Only load trusted archives. */
struct RuntimeDependencies {
    /** SNPE user-defined operation packages. */
    std::vector<SNPEUDOPackage> snpeUdo;
};

namespace detail {
// Same rules as luxonis-ml: a normalized relative POSIX path that stays inside
// the archive, has no Windows drive and is not config.json.
inline void validateRuntimeDependencyPath(const std::string& path) {
    const auto invalid = [&path]() { return std::invalid_argument("Invalid NNArchive runtime dependency path: " + path); };
    if(path.empty() || path == "config.json" || path.find_first_of(std::string("\\\0", 2)) != std::string::npos || (path.size() >= 2 && path[1] == ':')) {
        throw invalid();
    }
    for(std::size_t start = 0;;) {
        const auto end = path.find('/', start);
        const auto part = path.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if(part.empty() || part == "." || part == "..") throw invalid();
        if(end == std::string::npos) break;
        start = end + 1;
    }
}
}  // namespace detail

/** Read an implementation library from NNArchive JSON. */
inline void from_json(const nlohmann::json& j, RuntimeImplementation& value) {
    j.at("runtime").get_to(value.runtime);
    j.at("path").get_to(value.path);
    detail::validateRuntimeDependencyPath(value.path);
    if(j.size() != 2 || value.runtime.empty()) throw std::invalid_argument("Invalid NNArchive runtime implementation metadata");
}
/** Write an implementation library as NNArchive JSON. */
inline void to_json(nlohmann::json& j, const RuntimeImplementation& value) {
    j = {{"runtime", value.runtime}, {"path", value.path}};
}

/** Read a UDO package from NNArchive JSON. */
inline void from_json(const nlohmann::json& j, SNPEUDOPackage& value) {
    j.at("registration").get_to(value.registration);
    j.at("implementations").get_to(value.implementations);
    detail::validateRuntimeDependencyPath(value.registration);
    if(j.size() != 2 || value.implementations.empty()) throw std::invalid_argument("Invalid NNArchive SNPE UDO package metadata");
}
/** Write a UDO package as NNArchive JSON. */
inline void to_json(nlohmann::json& j, const SNPEUDOPackage& value) {
    j = {{"registration", value.registration}, {"implementations", value.implementations}};
}

/** Read runtime dependencies from NNArchive JSON. */
inline void from_json(const nlohmann::json& j, RuntimeDependencies& value) {
    j.at("snpe_udo").get_to(value.snpeUdo);
    if(j.size() != 1 || value.snpeUdo.empty()) throw std::invalid_argument("Invalid NNArchive runtime_dependencies metadata");
}
/** Write runtime dependencies as NNArchive JSON. */
inline void to_json(nlohmann::json& j, const RuntimeDependencies& value) {
    j = {{"snpe_udo", value.snpeUdo}};
}

NOP_EXTERNAL_STRUCTURE(RuntimeImplementation, runtime, path);
NOP_EXTERNAL_STRUCTURE(SNPEUDOPackage, registration, implementations);
NOP_EXTERNAL_STRUCTURE(RuntimeDependencies, snpeUdo);

}  // namespace dai::nn_archive::v1
