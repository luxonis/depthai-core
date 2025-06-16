#include "../../../src/utility/Platform.hpp"

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "../../../include/subprocess/subprocess.hpp"

#ifndef _WIN32
    #include <sys/wait.h>
#endif

using namespace dai;
namespace fs = std::filesystem;
using FileLock = dai::platform::FileLock;
using FolderLock = dai::platform::FolderLock;

TEST_CASE("FileLock basic functionality", "[platform]") {
    auto tempFile = fs::path(dai::platform::getTempPath()) / "test_lock_file.txt";
    std::ofstream ofs(tempFile);
    if(!ofs.is_open()) {
        throw std::runtime_error("Could not create test file");
    }
    ofs.close();

    SECTION("Lock and unlock") {
        auto lock = FileLock::lock(tempFile.string());
        REQUIRE(lock->holding());

        lock->unlock();
        REQUIRE_FALSE(lock->holding());
    }

    SECTION("Create if not exists") {
        auto nonExistentFile = fs::temp_directory_path() / "non_existent_file.txt";
        fs::remove(nonExistentFile);  // Ensure it doesn't exist

        auto lock = FileLock::lock(nonExistentFile.string(), true);  // true = create if not exists
        REQUIRE(lock->holding());
        REQUIRE(fs::exists(nonExistentFile));

        lock->unlock();
        fs::remove(nonExistentFile);
    }

    // Cleanup
    fs::remove(tempFile);
}

TEST_CASE("FolderLock basic functionality", "[platform]") {
    auto tempDir = fs::path(dai::platform::getTempPath()) / "test_lock_dir";
    fs::create_directory(tempDir);  // create directory

    SECTION("Lock and unlock") {
        auto lock = FolderLock::lock(tempDir.string());
        REQUIRE(lock->holding());

        lock->unlock();
        REQUIRE_FALSE(lock->holding());
    }

    // Cleanup
    fs::remove_all(tempDir);
}

TEST_CASE("Process-level locking with dummy", "[platform]") {
    auto tempFile = fs::path(dai::platform::getTempPath()) / "test_lock_file.txt";
    std::ofstream ofs(tempFile);
    if(!ofs.is_open()) {
        throw std::runtime_error("Could not create test file");
    }
    ofs.close();

    auto start = std::chrono::steady_clock::now();

    std::cout << "FSLOCK_DUMMY_PATH: " << FSLOCK_DUMMY_PATH << std::endl;
    std::cout << "tempFile: " << tempFile << std::endl;

    // Launch two subprocesses
    auto proc1 = subprocess::Popen({FSLOCK_DUMMY_PATH, tempFile.string(), "A"}, subprocess::output{subprocess::PIPE});
    auto proc2 = subprocess::Popen({FSLOCK_DUMMY_PATH, tempFile.string(), "B"}, subprocess::output{subprocess::PIPE});
    proc1.wait();
    proc2.wait();

    std::cout << "proc1 stdout:\n" << proc1.communicate().first.buf.data() << std::endl;
    std::cout << "proc2 stdout:\n" << proc2.communicate().first.buf.data() << std::endl;

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "duration: " << duration.count() << "ms" << std::endl;
    REQUIRE(duration.count() >= 2000);
    REQUIRE(duration.count() < 3000);
}

TEST_CASE("Thread-level locking", "[platform]") {
    auto tempFile = fs::path(dai::platform::getTempPath()) / "test_thread_lock.txt";
    std::ofstream ofs(tempFile);
    if(!ofs.is_open()) {
        throw std::runtime_error("Could not create test file");
    }
    ofs.close();

    SECTION("Multiple threads with same lock") {
        auto start = std::chrono::steady_clock::now();

        std::thread t1([&tempFile]() {
            auto lock = FileLock::lock(tempFile.string());
            REQUIRE(lock->holding());
            std::cout << "Thread 1" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        });

        std::thread t2([&tempFile]() {
            auto lock = FileLock::lock(tempFile.string());
            REQUIRE(lock->holding());
            std::cout << "Thread 2" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        });

        std::thread t3([&tempFile]() {
            auto lock = FileLock::lock(tempFile.string());
            REQUIRE(lock->holding());
            std::cout << "Thread 3" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        });

        t1.join();
        t2.join();
        t3.join();

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        REQUIRE(duration.count() >= 3000);  // Should be more than 3 seconds (1 second each thread)
    }

    SECTION("Multiple threads with different locks") {
        auto tempFile2 = fs::path(dai::platform::getTempPath()) / "test_thread_lock2.txt";
        std::ofstream ofs(tempFile2);
        if(!ofs.is_open()) {
            throw std::runtime_error("Could not create test file");
        }
        ofs.close();

        auto start = std::chrono::steady_clock::now();

        std::thread t1([&tempFile]() {
            auto lock = FileLock::lock(tempFile.string());
            REQUIRE(lock->holding());
            std::cout << "Thread 1" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        });

        std::thread t2([&tempFile]() {
            auto lock = FileLock::lock(tempFile.string());
            REQUIRE(lock->holding());
            std::cout << "Thread 2" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        });

        std::thread t3([&tempFile2]() {
            auto lock = FileLock::lock(tempFile2.string());
            REQUIRE(lock->holding());
            std::cout << "Thread 3" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        });

        t1.join();
        t2.join();
        t3.join();

        // Cleanup second file
        fs::remove(tempFile2);

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        REQUIRE(duration.count() >= 2000);
        REQUIRE(duration.count() < 2300);  // thread3 can start immediately, it's a different file
    }

    // Cleanup
    fs::remove(tempFile);
}
