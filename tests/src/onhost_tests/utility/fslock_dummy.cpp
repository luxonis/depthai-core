#include <filesystem>
#include <iostream>
#include <thread>

#include "../../../src/utility/Platform.hpp"

int main(int argc, char* argv[]) {
    if(argc != 3) {
        std::cerr << "This program expects exactly two arguments!!" << std::endl;
        return 1;
    }

    const auto filePath = std::filesystem::path(argv[1]);
    const std::string identifier = argv[2];

    // Lock file
    auto lock = dai::platform::FileLock::lock(filePath.string());
    if(lock->holding()) {
        std::cout << "Process " << identifier << " holding lock" << std::endl;
    } else {
        std::cerr << "Process " << identifier << " failed to acquire lock" << std::endl;
        return 2;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    lock->unlock();
    std::cout << "Process " << identifier << " released lock" << std::endl;

    return 0;
}