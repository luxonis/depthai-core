#include "DspCommon.hpp"

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace {

struct LogState {
    dai_device_log_fn fn = nullptr;
    void* user = nullptr;
    int32_t minLevel = DAI_LOG_LEVEL_TRACE;
};

std::mutex logMutex;
LogState logState;

}  // namespace

extern "C" void dai_device_dsp_set_log(dai_device_log_fn fn, void* user, int32_t min_level) {
    std::lock_guard<std::mutex> lock(logMutex);
    logState.fn = fn;
    logState.user = user;
    logState.minLevel = min_level;
}

namespace hexagon {

void logMessage(int32_t level, const char* format, ...) noexcept {
    try {
        LogState state;
        {
            std::lock_guard<std::mutex> lock(logMutex);
            state = logState;
        }
        if(state.fn == nullptr || level < state.minLevel) {
            return;
        }
        char message[512];
        va_list args;
        va_start(args, format);
        const int written = std::vsnprintf(message, sizeof(message), format, args);
        va_end(args);
        if(written < 0) {
            return;
        }
        state.fn(state.user, level, message);
    } catch(...) {  // The log must not throw out of the C interface
    }
}

dai_status toStatus(int aeeStatus, const char* call) noexcept {
    if(aeeStatus == 0) {  // AEE_SUCCESS
        return DAI_STATUS_OK;
    }
    if(aeeStatus > 0) {
        return aeeStatus;
    }
    logMessage(DAI_LOG_LEVEL_ERROR,
               "%s failed with AEE status %d (0x%08x, %s)",
               call,
               aeeStatus,
               static_cast<unsigned>(aeeStatus),
               dai_device_dsp_status_name(aeeStatus));
    return DAI_STATUS_INTERNAL;
}

}  // namespace hexagon
