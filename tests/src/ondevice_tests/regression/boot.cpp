#include <atomic>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <exception>
#include <mutex>
#include <thread>

constexpr int NUMBER_OF_BOOTS = 5;
constexpr int ENUMERATION_SLEEP_MS = 300;
constexpr int BOOT_SLEEP_MS = 1000;

TEST_CASE("Boot while enumerating devices") {
    std::atomic<bool> running = true;
    std::mutex workerExceptionMtx;
    std::exception_ptr workerException = nullptr;
    std::thread getAllThread([&running, &workerExceptionMtx, &workerException]() {
        try {
            while(running) {
                auto devices = dai::Device::getAllAvailableDevices();
                (void)devices;
                std::this_thread::sleep_for(std::chrono::milliseconds(ENUMERATION_SLEEP_MS));
            }
        } catch(...) {
            std::lock_guard<std::mutex> lock(workerExceptionMtx);
            if(!workerException) {
                workerException = std::current_exception();
            }
            running = false;
        }
    });

    auto rethrowWorkerException = [&workerExceptionMtx, &workerException]() {
        std::lock_guard<std::mutex> lock(workerExceptionMtx);
        if(workerException) {
            std::rethrow_exception(workerException);
        }
    };

    try {
        for(int i = 0; i < NUMBER_OF_BOOTS; ++i) {
            rethrowWorkerException();
            {
                dai::Device dev;
                (void)dev;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(BOOT_SLEEP_MS));
            rethrowWorkerException();
            REQUIRE(true);
        }
    } catch(...) {
        running = false;
        if(getAllThread.joinable()) {
            getAllThread.join();
        }
        throw;
    }

    running = false;
    getAllThread.join();
    rethrowWorkerException();
}
