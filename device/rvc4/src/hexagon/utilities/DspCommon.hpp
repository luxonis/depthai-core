#pragma once  // Include only the C interface, std and the Hexagon SDK: the vendor module ships these libraries prebuilt

#include <cstdint>
#include <exception>
#include <stdexcept>

#include "vendor/dai_device_dsp.h"

namespace hexagon {

__attribute__((format(printf, 2, 3))) void logMessage(int32_t level, const char* format, ...) noexcept;

class DspCallError : public std::runtime_error {
   public:
    DspCallError(int aeeStatus, const char* message) : std::runtime_error(message), aeeStatus(aeeStatus) {}
    int aeeStatus;
};

dai_status toStatus(int aeeStatus, const char* call) noexcept;  // AEE_SUCCESS -> OK; positive AEE status = driver status; negative -> DAI_STATUS_INTERNAL

template <typename Fn>
dai_status guard(const char* function, Fn&& fn) noexcept {
    try {
        return fn();
    } catch(const DspCallError& e) {
        return toStatus(e.aeeStatus, function);
    } catch(const std::exception& e) {
        logMessage(DAI_LOG_LEVEL_ERROR, "%s: %s", function, e.what());
        return DAI_STATUS_INTERNAL;
    } catch(...) {
        logMessage(DAI_LOG_LEVEL_ERROR, "%s: unknown exception", function);
        return DAI_STATUS_INTERNAL;
    }
}

}  // namespace hexagon
