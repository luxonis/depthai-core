#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "depthai/depthai.hpp"
#include "utility/Environment.hpp"

TEST_CASE("Crashdump callback is invoked on device crash") {
    dai::utility::setEnv("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");  // don't save nor upload crash dump
    dai::utility::setEnv("DEPTHAI_CRASH_DEVICE", "1");

    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> callbackInvoked{false};
    std::shared_ptr<dai::CrashDump> receivedDump;
    dai::Device device;

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
    if(auto rvc2Dump = std::dynamic_pointer_cast<dai::CrashDumpRVC2>(receivedDump)) {
        REQUIRE_FALSE(rvc2Dump->crashReports.crashReports.empty());
    } else if(auto rvc4Dump = std::dynamic_pointer_cast<dai::CrashDumpRVC4>(receivedDump)) {
        REQUIRE_FALSE(rvc4Dump->data.empty());
        REQUIRE_FALSE(rvc4Dump->filename.empty());
    } else {
        FAIL("Unknown crash dump type returned by callback");
    }
}

TEST_CASE("hasCrashed returns true after device crash") {
    dai::utility::setEnv("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");  // don't save nor upload crash dump
    dai::utility::setEnv("DEPTHAI_CRASH_DEVICE", "1");

    dai::Device device;
    REQUIRE_FALSE(device.hasCrashed());

    device.crashDevice();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(12);

    while(!device.hasCrashed() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    REQUIRE(device.hasCrashed());
}
