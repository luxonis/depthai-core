#include <stdexcept>

#include "build/version.hpp"
#include "utility/spdlog-fmt.hpp"

// clang-format off

// Only use this one for internal errors. Clearly invalid states that shouldn't happen.
#define DAI_CHECK_IN(A) \
    if(!(A)) { \
        throw std::runtime_error(fmt::format( \
            "Internal error occured. Please report." \
            " commit: {}" \
            " | dev_v: {}" \
            " | boot_v: {}" \
            " | rvc3_v: {}" \
            " | file: {}:{}", \
            dai::build::COMMIT, \
            dai::build::DEVICE_VERSION, \
            dai::build::BOOTLOADER_VERSION, \
            dai::build::DEVICE_RVC3_VERSION, \
            __FILE__,\
            __LINE__ \
            )); \
    }

#define DAI_CHECK(A, M) \
    if(!(A)) { \
        throw std::runtime_error( M ); \
    }

#define DAI_CHECK_V(A, M, ...) \
    if(!(A)) { \
        throw std::runtime_error(fmt::format( M, ##__VA_ARGS__ )); \
    }

// clang-format on
