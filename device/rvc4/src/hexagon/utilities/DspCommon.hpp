#pragma once
// Common part of the C interface of the DSP offload (vendor/dai_device_dsp.h): the log and the status of a call.
// The StereoDepthDsp library compiles DspCommon.cpp; the ImageAlignDsp library links StereoDepthDsp. The prebuilt dsp/
// folder of the vendor module has these libraries, so this code includes only the C interface, the standard library
// and the Hexagon SDK.

#include <cstdint>
#include <exception>
#include <stdexcept>

#include "vendor/dai_device_dsp.h"

namespace hexagon {

/// Sends a printf-style message to the callback of dai_device_dsp_set_log. No callback, or level below the minimum
/// level: no message (and no formatting). Never throws.
__attribute__((format(printf, 2, 3))) void logMessage(int32_t level, const char* format, ...) noexcept;

/// A FastRPC call that failed with an AEE status, for example the open of the DSP session. The code that throws it
/// logs the details.
class DspCallError : public std::runtime_error {
   public:
    DspCallError(int aeeStatus, const char* message) : std::runtime_error(message), aeeStatus(aeeStatus) {}
    int aeeStatus;
};

/// The dai_status of the AEE status of a FastRPC call: AEE_SUCCESS gives DAI_STATUS_OK, a positive AEE status is the
/// driver status, a negative one gives DAI_STATUS_INTERNAL (and an error message with the AEE status).
dai_status toStatus(int aeeStatus, const char* call) noexcept;

/// Runs fn (returns a dai_status) so that no exception leaves the C interface: a DspCallError gives the status of its
/// AEE status, other exceptions give DAI_STATUS_INTERNAL and an error message.
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
