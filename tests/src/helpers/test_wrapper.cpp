#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "subprocess.hpp"

int main(int argc, char* argv[]) {
    if(argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <timeout> <script> [args...]" << std::endl;
        return 1;
    }

    // Parse timeout from first argument
    int timeout = 30;  // Default fallback
    try {
        timeout = std::stoi(argv[1]);
        if(timeout <= 0) {
            std::cout << "Timeout must be a positive integer, using default (30s)" << std::endl;
            timeout = 30;
        }
    } catch(const std::exception& e) {
        std::cerr << "Failed to parse timeout, using default (30s): " << e.what() << std::endl;
    }

    // Create args vector starting from the script (second argument)
    std::vector<std::string> args;
    for(int i = 2; i < argc; ++i) {
        args.emplace_back(argv[i]);
    }

    try {
        auto devicesBefore = 0;

        while(devicesBefore < 1) {
            devicesBefore = dai::Device::getAllAvailableDevices().size();
            std::cout << "Devices now: " << devicesBefore << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        // Run the process with captured output and timeout
        auto start = std::chrono::steady_clock::now();
        bool timedOut = false;

        subprocess::Popen proc(args);

        // Wait for the specified timeout
        std::cout << "Running with timeout of " << timeout << " seconds" << std::endl;
        while(proc.poll() == -1) {
            auto now = std::chrono::steady_clock::now();
            if(std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > timeout) {
                timedOut = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if(timedOut) {
            std::cerr << "=== Test exceeded timeout of " << timeout << " seconds, terminating ===" << std::endl;

#if defined(_WIN32) || defined(__MINGW32__)

            std::cerr << "Attempting termination via proc.kill() (TerminateProcess)..." << std::endl;
            proc.kill();  // Call kill (uses TerminateProcess). Let library use its default exit code (9).
                          // Or you could use proc.kill(1); for exit code 1.

            // Wait a short time to allow the OS to terminate the process
            // TerminateProcess is generally forceful, but cleanup might take a moment.
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // 1.5 seconds

            if(proc.poll() == -1) {
                std::cerr << "Process still reported as running after TerminateProcess attempt." << std::endl;
                // No further escalation possible with this library's API on Windows.
            }

#else
            std::cerr << "Sending SIGINT..." << std::endl;
            proc.kill(SIGINT);                                     // Try graceful termination first
            std::this_thread::sleep_for(std::chrono::seconds(7));  // Wait for it to exit

            if(proc.poll() == -1) {  // Check if it's still running
                std::cerr << "Process still running after SIGINT, sending SIGKILL..." << std::endl;
                proc.kill(SIGKILL);                                    // Force kill
                std::this_thread::sleep_for(std::chrono::seconds(1));  // Short wait after SIGKILL
            }
#endif
            std::cerr << "Process terminated." << std::endl;
        }

        // Device recovery wait
        std::cout << "Devices before: " << devicesBefore << std::endl;
        while(devicesBefore > dai::Device::getAllAvailableDevices().size()) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::cout << "Devices now: " << dai::Device::getAllAvailableDevices().size() << std::endl;
        }

        // Now we can safely check the return code
        int retcode = proc.retcode();
        // 2 signifies that proc was killed by a timeout
        if(retcode == 2) {
            retcode = 0;
        }

        return retcode;

    } catch(const std::exception& e) {
        std::cerr << "[wrapper] Exception: " << e.what() << std::endl;
        return 1;
    }
}
