#pragma once

#include <string>
#include <vector>

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

}  // namespace dai::nn_archive::v1
