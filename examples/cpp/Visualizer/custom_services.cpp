#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

// Global flag for graceful shutdown
std::atomic<bool> quitEvent(false);

// Signal handler
void signalHandler(int signum) {
    std::cout << "You pressed Ctrl+C!" << std::endl;
    quitEvent = true;
}

// Test service function
nlohmann::json testService(const nlohmann::json& input) {
    std::cout << "Test service called with input: " << input.dump() << std::endl;
    return {{"result", "testService result"}};
}

int main() {
    // Set up signal handlers
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    try {
        // Create remote connection
        dai::RemoteConnection remoteConnection;

        // Register service
        remoteConnection.registerService("myService", testService);

        // Main loop
        while(!quitEvent) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}