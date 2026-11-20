#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <exception>
#include <thread>

#include "depthai/depthai.hpp"

using namespace std::chrono_literals;

TEST_CASE("Device boot and communication survive concurrent discovery", "[boot][discovery]") {
    std::atomic<bool> stop{false};
    std::exception_ptr enumerationError;
    std::thread enumerationThread([&stop, &enumerationError]() {
        try {
            while(!stop.load()) {
                dai::Device::getAllAvailableDevices();
                std::this_thread::sleep_for(300ms);
            }
        } catch(...) {
            enumerationError = std::current_exception();
            stop.store(true);
        }
    });

    try {
        for(int i = 0; i < 5 && !stop.load(); ++i) {
            dai::Device device;
            // Let discovery outlive XLink's 500 ms USB serial-number cache.
            std::this_thread::sleep_for(1s);
            REQUIRE(device.getDdrMemoryUsage().total > 0);
        }
    } catch(...) {
        stop.store(true);
        enumerationThread.join();
        throw;
    }

    stop.store(true);
    enumerationThread.join();
    if(enumerationError) std::rethrow_exception(enumerationError);
}
