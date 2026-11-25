#include "FastCvUtils.hpp"

#include <atomic>
#include <mutex>

std::mutex fcvRunMutex;
std::atomic<bool> fcvInitialized = false;

void fcvInit(fcvOperationMode operMode) {
    bool prevInitialized = fcvInitialized.exchange(true);
    if(!prevInitialized) {
#ifndef FORCE_FASTCV_ON_SPECIFIC_CORE
        // DON'T CALL fcvSetOperationMode(operMode) BEFORE zdl::SNPE::SNPEBuilder::build() !!!
        // else zdl::SNPE::SNPEBuilder::build() will SIGSEGV !!!
        int setOperMode = fcvSetOperationMode(operMode);
#else
        ForceExecutionOnSpecificCore();
#endif
    }
}

std::lock_guard<std::mutex> getFcvLock() {
    return std::lock_guard<std::mutex>(fcvRunMutex);
}
