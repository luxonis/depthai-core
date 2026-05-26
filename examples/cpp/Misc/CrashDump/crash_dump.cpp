#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

#include "depthai/depthai.hpp"

namespace {
void setEnvVar(const char* key, const char* value) {
#ifdef _WIN32
    _putenv_s(key, value);
#else
    setenv(key, value, 1);
#endif
}
}  // namespace

int main() {
    setEnvVar("DEPTHAI_CRASH_DEVICE", "1");
    setEnvVar("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");

    dai::Device device;
    std::mutex mtx;
    std::condition_variable cv;
    bool callbackInvoked = false;
    std::shared_ptr<dai::CrashDump> receivedDump;

    // Register crash dump callback
    device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) {
        // Basic crash dump usage
        std::cout << "Crash dump callback invoked.\n";
        std::cout << "  Platform: " << dai::platform2string(dump->getPlatform()) << std::endl;
        std::cout << "  Device ID: " << dump->deviceId << std::endl;
        std::cout << "  Crash timestamp: " << dump->crashdumpTimestamp << std::endl;
        std::cout << "  DepthAI commit: " << dump->depthaiCommitHash << std::endl;

        // Advanced extra custom user data usage
        dump->extra["example"] = "cpp_crash_dump";
        dump->extra["host_time_ms"] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        {
            std::lock_guard<std::mutex> lock(mtx);
            receivedDump = std::move(dump);
            callbackInvoked = true;
        }

        cv.notify_one();
    });

    // Trigger the crash manually. In a real scenario, this would be triggered by an actual device crash.
    std::cout << "Triggering crash manually ...\n";
    device.crashDevice();

    // Wait for the callback to be invoked (with timeout)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if(!cv.wait_for(lock, std::chrono::seconds(60), [&] { return callbackInvoked; })) {
            std::cerr << "Timed out waiting for crash dump callback." << std::endl;
            return 1;
        }
    }

    // Check if the device crash was detected by hasCrashed() within a reasonable time frame
    bool crashDetected = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while(std::chrono::steady_clock::now() < deadline) {
        if(device.hasCrashed()) {
            crashDetected = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if(!crashDetected) {
        std::cerr << "Device crash was not detected by hasCrashed() within timeout.\n";
        return 1;
    }

    // Print user defined extra data
    std::cout << "Crash dump extra data:\n";
    std::cout << receivedDump->extra.dump(2) << std::endl;

    return 0;
}
