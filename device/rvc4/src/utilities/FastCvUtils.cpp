#include "FastCvUtils.hpp"

#include <atomic>
#include <mutex>

std::mutex fcvRunMutex;
std::atomic<bool> fcvInitialized = false;

void fcvInit(fcvOperationMode operMode) {
    bool prevInitialized = fcvInitialized.exchange(true);
    if(!prevInitialized) {
#ifndef FORCE_FASTCV_ON_SPECIFIC_CORE
        int setOperMode = fcvSetOperationMode(operMode);  // Calling fcvSetOperationMode before SNPEBuilder::build() makes build() SIGSEGV
#else
        ForceExecutionOnSpecificCore();
#endif
    }
}

std::lock_guard<std::mutex> getFcvLock() {
    return std::lock_guard<std::mutex>(fcvRunMutex);
}
