#include "../../../src/utility/Platform.hpp"

#include <sys/wait.h>

#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

using namespace dai;
namespace fs = std::filesystem;
using FileLock = dai::platform::FileLock;
using FolderLock = dai::platform::FolderLock;

TEST_CASE("FileLock basic functionality", "[platform]") {
    auto tempFile = fs::path(dai::platform::getTempPath()) / "test_lock_file.txt";
    std::ofstream(tempFile).close();  // create file

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

TEST_CASE("Concurrent locking", "[platform]") {
    auto tempFile = fs::path(dai::platform::getTempPath()) / "test_lock_file.txt";
    std::ofstream(tempFile).close();  // create file
    std::cout << "tempFile: " << tempFile.string() << std::endl;

    auto start = std::chrono::steady_clock::now();

    pid_t pid1 = fork();
    if(pid1 == 0) {
        // Child process 1
        auto lock = FileLock::lock(tempFile.string());
        REQUIRE(lock->holding());
        std::cout << "Process 1 holding lock" << lock->holding() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        lock->unlock();
        exit(0);
    }

    pid_t pid2 = fork();
    if(pid2 == 0) {
        // Child process 2
        auto lock = FileLock::lock(tempFile.string());
        REQUIRE(lock->holding());
        std::cout << "Process 2 holding lock" << lock->holding() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        lock->unlock();
        exit(0);
    }

    // Parent process waits for both children
    int status;
    waitpid(pid1, &status, 0);
    waitpid(pid2, &status, 0);

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    REQUIRE(duration.count() >= 2000);  // Should be more than 2 seconds (1 second each process)
}
