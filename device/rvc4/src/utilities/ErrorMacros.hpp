#pragma once

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>

// libraries
#include <spdlog/fmt/fmt.h>

// internal
#include "build/version.hpp"
#include "pipeline/PipelineBuilder.hpp"

// clang-format off

// Only use this one for internal errors. Clearly invalid states that shouldn't happen.
#define DAI_CHECK_IN(A) \
    if(!(A)) { /* NOLINT(readability-simplify-boolean-expr) */ \
        throw std::runtime_error(fmt::format( \
            "Internal error occured. Please report." \
            " commit: {}" \
            " | version: {}" \
            " | file: {}:{}", \
            dai::device::build::COMMIT,	\
            dai::device::build::VERSION, \
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

namespace dai {
namespace device {
// vendor/DeviceError.hpp
[[noreturn]] void throwDeviceError(std::int32_t status, const char* unit, const char* call, const char* detail);
}  // namespace device
}  // namespace dai

// A call of the C interface of the vendor module (vendor/dai_device_*.h): when its status is not DAI_STATUS_OK (0),
// throws dai::device::DeviceError with the text "<UNIT>: <function> failed: <status>: <text of the vendor module>".
#define DAI_CHECK_DEVICE(UNIT, CALL)                                                                         \
    do {                                                                                                     \
        const std::int32_t daiDeviceStatus_ = (CALL);                                                        \
        if(daiDeviceStatus_ != 0) ::dai::device::throwDeviceError(daiDeviceStatus_, UNIT, #CALL, nullptr); \
    } while(0)

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define DAI_ERROR(M, ...)                                                          \
    {                                                                              \
        auto logger = dai::gate::PipelineBuilder::getInstance().getSystemLogger(); \
        if(logger) {                                                               \
            logger->error(__FILE__ ":" TOSTRING(__LINE__) ": " M, ##__VA_ARGS__);  \
        } else {                                                                   \
            fprintf(stderr, "Logger was null\n");                                  \
        }                                                                          \
    }

#define DAI_WARN(M, ...)                                                           \
    {                                                                              \
        auto logger = dai::gate::PipelineBuilder::getInstance().getSystemLogger(); \
        if(logger) {                                                               \
            logger->warn(__FILE__ ":" TOSTRING(__LINE__) ": " M, ##__VA_ARGS__);   \
        } else {                                                                   \
            fprintf(stderr, "Logger was null\n");                                  \
        }                                                                          \
    }

#define DAI_INFO(M, ...)                                                           \
    {                                                                              \
        auto logger = dai::gate::PipelineBuilder::getInstance().getSystemLogger(); \
        if(logger) {                                                               \
            logger->info(__FILE__ ":" TOSTRING(__LINE__) ": " M, ##__VA_ARGS__);   \
        } else {                                                                   \
            fprintf(stderr, "Logger was null\n");                                  \
        }                                                                          \
    }

#define DAI_LOG(M, ...)                                                            \
    {                                                                              \
        auto logger = dai::gate::PipelineBuilder::getInstance().getSystemLogger(); \
        if(logger) {                                                               \
            logger->debug(__FILE__ ":" TOSTRING(__LINE__) ": " M, ##__VA_ARGS__);  \
        } else {                                                                   \
            fprintf(stderr, "Logger was null\n");                                  \
        }                                                                          \
    }

#define DAI_TRACE(M, ...)                                                          \
    {                                                                              \
        auto logger = dai::gate::PipelineBuilder::getInstance().getSystemLogger(); \
        if(logger) {                                                               \
            logger->trace(__FILE__ ":" TOSTRING(__LINE__) ": " M, ##__VA_ARGS__);  \
        } else {                                                                   \
            fprintf(stderr, "Logger was null\n");                                  \
        }                                                                          \
    }

#define DAI_LOG_TIME()                                                                                                                            \
    {                                                                                                                                             \
        auto now = std::chrono::high_resolution_clock::now();                                                                                     \
        auto time = std::chrono::system_clock::to_time_t(now);                                                                                    \
        auto tm = *std::gmtime(&time);                                                                                                            \
        auto epoch = now.time_since_epoch();                                                                                                      \
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(epoch).count() % 1000000;                                                 \
        std::cout << "Timing debug: " << __FILE__ << ":" << __LINE__ << " " << std::put_time(&tm, "%F %T.") << us << std::put_time(&tm, " %Z\n"); \
    }

/*
 * At top of the file using DAI_LOG_TIME_START/END macros add this:
 *

#define TIME_DEBUG_ON
#undef DAI_PRINTF_TIME
#ifdef TIME_DEBUG_ON
    #define DAI_PRINTF_TIME(...) printf(__VA_ARGS__)
#else
    #define DAI_PRINTF_TIME(...)
#endif

*/

#define DAI_LOG_TIME_START(ID) \
    uint64_t ID##StartTime;    \
    asm volatile("mrs %0, cntvct_el0" : "=r"(ID##StartTime));

#define DAI_LOG_TIME_END(ID)                                \
    uint64_t ID##EndTime;                                   \
    uint64_t ID##Freq;                                      \
    asm volatile("mrs %0, cntvct_el0" : "=r"(ID##EndTime)); \
    asm volatile("mrs %0, cntfrq_el0" : "=r"(ID##Freq));    \
    uint64_t ID##DiffTime = ID##EndTime - ID##StartTime;    \
    DAI_PRINTF_TIME("TIME_DBG: " #ID " took: %lu cpu cycles = %fms\n", ID##DiffTime, (double)ID##DiffTime / (double)ID##Freq * 1000.0);
