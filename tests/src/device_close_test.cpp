#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <system_error>

#include "depthai/depthai.hpp"

namespace {
void setEnv(const char* name, const char* value) {
#ifdef _WIN32
    _putenv_s(name, value);
#else
    setenv(name, value, 1);
#endif
}
}  // namespace

TEST_CASE("Closing a device twice is safe and RPC calls after close throw") {
    // Never upload crash dumps from tests (a leftover dump is logged on close)
    setEnv("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    dai::Device device;
    device.close();
    REQUIRE(device.isClosed());
    REQUIRE_NOTHROW(device.close());
    // close() releases the RPC client: calls must throw instead of dereferencing it
    REQUIRE_THROWS_AS(device.getMxId(), std::system_error);
}

TEST_CASE("Closing a crashed device completes and the device can be reopened") {
    setEnv("DEPTHAI_DISABLE_CRASHDUMP_COLLECTION", "1");
    setEnv("DEPTHAI_CRASH_DEVICE", "1");
    {
        dai::Device device;
        device.crashDevice();
        // close() tries to collect the crash dump, which reboots the device; it must not throw
        REQUIRE_NOTHROW(device.close());
        REQUIRE(device.isClosed());
    }
    dai::Device device;
    REQUIRE(device.getDdrMemoryUsage().total > 0);
}
