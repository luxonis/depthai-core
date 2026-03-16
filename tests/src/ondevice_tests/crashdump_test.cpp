#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "depthai/depthai.hpp"
#include "utility/Environment.hpp"

TEST_CASE("Crashdump callback is invoked on device crash") {
    dai::utility::setEnv("DEPTHAI_CRASHDUMP", "0");  // don't save nor upload crash dump
    dai::utility::setEnv("DEPTHAI_CRASH_DEVICE", "1");

    dai::Device device;

    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> callbackInvoked{false};
    std::shared_ptr<dai::CrashDump> receivedDump;

    device.registerCrashdumpCallback([&](std::shared_ptr<dai::CrashDump> dump) {
        std::lock_guard<std::mutex> lock(mtx);
        receivedDump = dump;
        callbackInvoked = true;
        cv.notify_one();
    });

    // Crash the device
    device.crashDevice();

    // Wait for the callback to be invoked (with timeout)
    {
        std::unique_lock<std::mutex> lock(mtx);
        bool received = cv.wait_for(lock, std::chrono::seconds(60), [&] { return callbackInvoked.load(); });
        REQUIRE(received);
    }

    REQUIRE(receivedDump != nullptr);
    REQUIRE_FALSE(receivedDump->deviceId.empty());
    REQUIRE_FALSE(receivedDump->crashdumpTimestamp.empty());
    REQUIRE_FALSE(receivedDump->depthaiCommitHash.empty());
}

TEST_CASE("hasCrashed returns true after device crash") {
    dai::utility::setEnv("DEPTHAI_CRASHDUMP", "0");  // don't save nor upload crash dump
    dai::utility::setEnv("DEPTHAI_CRASH_DEVICE", "1");

    dai::Device device;
    REQUIRE_FALSE(device.hasCrashed());

    device.crashDevice();

    // Give the device time to crash and for the watchdog to detect it
    std::this_thread::sleep_for(std::chrono::seconds(12));

    REQUIRE(device.hasCrashed());
}
